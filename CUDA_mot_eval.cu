// GPU anim test: evaluate RDFT-encoded motion data using CUDA
// Author: Sergey Chaban <sergey.chaban@gmail.com>

#include <cuda_runtime.h>
#include <math_functions.h>
#include <math_constants.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <omp.h>

#define WIN32_LEAN_AND_MEAN 1
#define NOMINMAX
#define _WIN32_WINNT 0x0601
#include <Windows.h>

int64_t timestamp() {
	LARGE_INTEGER ctr;
	::QueryPerformanceCounter(&ctr);
	return ctr.QuadPart;
}

//---------------------------------------------------------------
enum E_MOT_TRK { POS, ROT, SCL };
enum E_MOT_RORD { XYZ, XZY, YXZ, YZX, ZXY, ZYX };
enum E_MOT_XORD { SRT, STR, RST, RTS, TSR, TRS };

typedef char MOT_STRING[0x40];

union MOT_VEC {
	struct { float x, y, z; };
	float v[3];
};

union MOT_QUAT {
	struct { float x, y, z, w; };
	float v[4];
};

struct MOT_TRACK {
	MOT_VEC vmin;
	MOT_VEC vmax;
	uint8_t srcMask;
	uint8_t dataMask;
	uint8_t reserved[6];
};

struct MOT_NODE {
	MOT_STRING name;
	uint32_t offs[3];
	uint8_t xord;
	uint8_t rord;
	uint8_t reserved[2];
	MOT_TRACK trk[3];
};

struct MOT_CLIP {
	char sig[4];
	float fps;
	uint32_t nfrm;
	uint32_t nnod;
	MOT_STRING name;
	MOT_NODE nodes[1];
};

MOT_QUAT motQuatExp(MOT_VEC v) {
	MOT_QUAT q = {};
	float ha = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
	float s = fabsf(ha) < 1.0e-4f ? 1.0f : sinf(ha) / ha;
	q.x = v.x * s;
	q.y = v.y * s;
	q.z = v.z * s;
	q.w = cosf(ha);
	s = 1.0f / sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	q.x *= s;
	q.y *= s;
	q.z *= s;
	q.w *= s;
	return q;
}

bool motCkNodeIdx(const MOT_CLIP* pClip, int nodeIdx) { return pClip && ((uint32_t)nodeIdx < pClip->nnod); }

bool motCkFrameNo(const MOT_CLIP* pClip, int fno) { return pClip && ((uint32_t)fno < pClip->nfrm); }

float* motGetTrackData(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk) {
	float* p = nullptr;
	if (pClip && motCkNodeIdx(pClip, nodeIdx)) {
		int itrk = (int)trk;
		if (itrk < 3) {
			uint32_t offs = pClip->nodes[nodeIdx].offs[itrk];
			if (offs) {
				char* pTop = (char*)pClip;
				p = (float*)&pTop[offs];
			}
		}
	}
	return p;
}

void motGetChanData(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk, int chIdx, float** ppData, int* pStride) {
	int stride = 0;
	float* p = nullptr;
	if ((uint32_t)chIdx < 3) {
		float* pTrk = motGetTrackData(pClip, nodeIdx, trk);
		if (pTrk) {
			int dataMask = pClip->nodes[nodeIdx].trk[(int)trk].dataMask;
			for (int i = 0; i < 3; ++i) {
				if (dataMask & (1 << i)) {
					p = pTrk + stride;
					++stride;
				}
			}
		}
	}
	if (ppData) {
		*ppData = p;
	}
	if (pStride) {
		*pStride = stride;
	}
}

MOT_VEC motGetVec(const MOT_CLIP* pClip, int nodeIdx, int fno, E_MOT_TRK trk) {
	MOT_VEC v = {};
	if (pClip && motCkNodeIdx(pClip, nodeIdx) && motCkFrameNo(pClip, fno)) {
		float* p = motGetTrackData(pClip, nodeIdx, trk);
		if (p) {
			int itrk = (int)trk;
			if (itrk < 3) {
				float defVal = trk == SCL ? 1.0f : 0.0f;
				int dataMask = pClip->nodes[nodeIdx].trk[itrk].dataMask;
				int srcMask = pClip->nodes[nodeIdx].trk[itrk].srcMask;
				int vsize = 0;
				for (int i = 0; i < 3; ++i) {
					if (dataMask & (1 << i)) ++vsize;
				}
				p += fno * vsize;
				for (int i = 0; i < 3; ++i) {
					if (dataMask & (1 << i)) {
						v.v[i] = *p++;
					} else if (srcMask & (1 << i)) {
						v.v[i] = pClip->nodes[nodeIdx].trk[itrk].vmin.v[i];
					} else {
						v.v[i] = defVal;
					}
				}
			}
		}
	}
	return v;
}

MOT_QUAT motGetQuat(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	return motQuatExp(motGetVec(pClip, nodeIdx, fno, ROT));
}

MOT_CLIP* motClipLoad(const char* pPath) {
	MOT_CLIP* pClip = nullptr;
	FILE* f = fopen(pPath, "rb");
	if (f) {
		long len = 0;
		if (0 == fseek(f, 0, SEEK_END)) {
			len = ftell(f);
		}
		fseek(f, 0, SEEK_SET);
		if (len) {
			pClip = (MOT_CLIP*)malloc(len);
			if (pClip) {
				fread(pClip, len, 1, f);
			}
		}
		fclose(f);
	}
	return pClip;
}

void motClipUnload(MOT_CLIP* pClip) {
	if (pClip) { free(pClip); }
}
//---------------------------------------------------------------

void RDFT_fwd(float* pDst, const float* pSrc, int nsrc, int stride) {
	int n = (nsrc & 1) ? nsrc + 1 : nsrc;
	int hn = n / 2;
	float* pRe = pDst;
	float* pIm = pDst + hn;
	float nrm = 1.0f / (float)hn;
	float s = atanf(1.0f) * 8.0f / (float)n;
	for (int i = 0; i < n; ++i) {
		pDst[i] = 0.0f;
	}
	for (int i = 0; i < hn; ++i) {
		for (int j = 0; j < n; ++j) {
			int idx = (j % nsrc) * stride;
			float val = pSrc[idx];
			float t = s * (float)i * (float)j;
			pRe[i] += val * cosf(t);
			pIm[i] -= val * sinf(t);
		}
	}
	for (int i = 0; i < hn; ++i) {
		pRe[i] *= nrm;
	}
	pRe[0] /= 2;
	pRe[hn - 1] /= 2;
	for (int i = 0; i < hn; ++i) {
		pIm[i] *= -nrm;
	}
}

void RDFT_inv(float* pDst, const float* pSrc, int ndst) {
	int n = (ndst & 1) ? ndst + 1 : ndst;
	int hn = n / 2;
	const float* pRe = pSrc;
	const float* pIm = pSrc + hn;
	float s = atanf(1.0f) * 8.0f / (float)n;
	for (int i = 0; i < ndst; ++i) {
		pDst[i] = 0.0f;
	}
	for (int i = 0; i < ndst; ++i) {
		float t = s * (float)i;
		for (int j = 0; j < hn; ++j) {
			float re = pRe[j];
			float im = pIm[j];
			float r = t * (float)j;
			pDst[i] += re*cosf(r) + im*sinf(r);
		}
	}
}

class cMotion {
public:
	struct ROT_CHANNEL {
		float* pData;
		int stride;
		int nodeId;
		int chId;
		float* pCoefs;
		int cut;
	};

protected:
	MOT_CLIP* mpClip;
	int mPosVecsNum;
	MOT_VEC* mpPosVecs;
	int mRotChansNum;
	ROT_CHANNEL* mpRotChans;
	int mCoefsNum;
	float mParamFactor;
	int mCut;
	float* mpEvalCoefsCPU;
	float* mpEvalCoefsDev;
	float* mpEvalResCPU;
	float* mpEvalResDev;
	cudaEvent_t mEvt;

	void eval_pos_vecs(float frame);

public:
	cMotion()
	:
	mpClip(nullptr),
	mPosVecsNum(0), mpPosVecs(nullptr),
	mRotChansNum(0), mpRotChans(nullptr),
	mCoefsNum(0), mParamFactor(0.0f), mCut(0),
	mpEvalCoefsCPU(nullptr), mpEvalCoefsDev(nullptr),
	mpEvalResCPU(nullptr), mpEvalResDev(nullptr)
	{}

	~cMotion() {
		unload();
	}

	void load(const char* pPath);
	void unload();

	float frame_to_param(float frame) {
		if (!mpClip) return 0.0f;
		float n = (float)mpClip->nfrm;
		float f = ::fmodf(::fabsf(frame), n);
		return f * mParamFactor;
	}

	int get_nfrm() const { return mpClip ? mpClip->nfrm : 0; }
	int get_nrot() const { return mRotChansNum; }
	int get_npos() const { return mPosVecsNum; }
	int get_ncut() const { return mCut; }
	float* get_res_ptr() { return mpEvalResCPU; }

	void clear_res() {
		if (mpEvalResCPU) {
			::memset(mpEvalResCPU, 0, mRotChansNum * sizeof(float));
		}
	}

	void eval_cpu(float frame);
	void eval_dev(float frame);
};

void cMotion::load(const char* pPath) {
	mpClip = motClipLoad(pPath);
	if (!mpClip) return;
	int numNodes = mpClip->nnod;
	mPosVecsNum = 0;
	for (int i = 0; i < numNodes; ++i) {
		int srcMask = mpClip->nodes[i].trk[POS].dataMask;
		int dataMask = mpClip->nodes[i].trk[POS].dataMask;
		if (srcMask || dataMask) {
			++mPosVecsNum;
		}
	}
	if (mPosVecsNum > 0) {
		mpPosVecs = (MOT_VEC*)::malloc(mPosVecsNum * sizeof(MOT_VEC));
	}
	mRotChansNum = 0;
	for (int i = 0; i < numNodes; ++i) {
		int dataMask = mpClip->nodes[i].trk[ROT].dataMask;
		for (int j = 0; j < 3; ++j) {
			if (dataMask & (1 << j)) {
				++mRotChansNum;
			}
		}
	}
	if (mRotChansNum > 0) {
		mpRotChans = (ROT_CHANNEL*)::malloc(mRotChansNum * sizeof(ROT_CHANNEL));
	}
	if (!mpRotChans) return;
	int chIdx = 0;
	for (int i = 0; i < numNodes; ++i) {
		int dataMask = mpClip->nodes[i].trk[ROT].dataMask;
		for (int j = 0; j < 3; ++j) {
			if (dataMask & (1 << j)) {
				ROT_CHANNEL* pCh = &mpRotChans[chIdx++];
				pCh->nodeId = i;
				pCh->chId = j;
				motGetChanData(mpClip, i, ROT, j, &pCh->pData, &pCh->stride);
			}
		}
	}

	int nfrm = mpClip->nfrm;
	float* pTmp = 0 ? (float*)::malloc(nfrm * sizeof(float)) : nullptr;
	int ncoef = nfrm;
	if (ncoef & 1) ++ncoef;
	mCoefsNum = ncoef;
	mParamFactor = ::atanf(1.0f) * 8.0f / (float)ncoef;
	for (int i = 0; i < mRotChansNum; ++i) {
		ROT_CHANNEL* pCh = &mpRotChans[i];
		pCh->pCoefs = (float*)::malloc(ncoef * sizeof(float));
		if (pCh->pCoefs) {
			RDFT_fwd(pCh->pCoefs, pCh->pData, nfrm, pCh->stride);
			if (pTmp) {
				RDFT_inv(pTmp, pCh->pCoefs, nfrm);
				::printf("-- [%d] %s:%c\n", i, mpClip->nodes[pCh->nodeId].name, "xyz"[pCh->chId]);
				for (int k = 0; k < nfrm; ++k) {
					float ref = pCh->pData[k*pCh->stride];
					float val = pTmp[k];
					::printf("[%d]: %.4f - %.4f = %f\n", k, ref, val, ref - val);
				}
			}
		}
	}
	if (pTmp) {
		::free(pTmp);
		pTmp = nullptr;
	}
	int minCut = nfrm + 1;
	int maxCut = 0;
	for (int i = 0; i < mRotChansNum; ++i) {
		ROT_CHANNEL* pCh = &mpRotChans[i];
		pCh->cut = (nfrm & (~1)) / 2;
		float* pRe = pCh->pCoefs;
		if (pRe) {
			const float qs = 0.0005f;
			for (int j = 0; j < ncoef / 2; ++j) {
				float x = pRe[j];
				float qx = ::floorf(::fabsf(x) / qs) * qs * (x < 0.0f ? -1.0f : 1.0f);
				if (qx == 0) {
					pCh->cut = j;
					break;
				}
			}
		}
		if (pCh->cut < minCut) {
			minCut = pCh->cut;
		}
		if (pCh->cut > maxCut) {
			maxCut = pCh->cut;
		}
	}
	mCut = (int)((float)maxCut * 0.75f);
	::printf("coefs cut: %d .. %d -> %d\n", minCut, maxCut, mCut);

	size_t evalCoefsSize = mRotChansNum * (mCut + mCut - 1) * sizeof(float);
	cudaMallocHost(&mpEvalCoefsCPU, evalCoefsSize);
	if (!mpEvalCoefsCPU) return;
	float* pCoefs = mpEvalCoefsCPU;
	for (int i = 0; i < mRotChansNum; ++i) {
		ROT_CHANNEL* pCh = &mpRotChans[i];
		if (pCh->pCoefs) {
			*pCoefs++ = pCh->pCoefs[0];
			for (int j = 1; j < mCut; ++j) {
				*pCoefs++ = pCh->pCoefs[j];
			}
			for (int j = 1; j < mCut; ++j) {
				*pCoefs++ = pCh->pCoefs[j + (mCoefsNum / 2)];
			}
		}
	}
	cudaMalloc(&mpEvalCoefsDev, evalCoefsSize);
	if (mpEvalCoefsDev) {
		cudaMemcpy(mpEvalCoefsDev, mpEvalCoefsCPU, evalCoefsSize, cudaMemcpyHostToDevice);
	}

	size_t evalResSize = mRotChansNum * sizeof(float);
	cudaMallocHost(&mpEvalResCPU, evalResSize);
	clear_res();
	cudaMalloc(&mpEvalResDev, evalResSize);

	cudaEventCreateWithFlags(&mEvt, cudaEventDisableTiming);
}

void cMotion::unload() {
	cudaDeviceSynchronize();
	cudaEventDestroy(mEvt);
	if (mpEvalResDev) {
		cudaFree(mpEvalResDev);
		mpEvalResDev = nullptr;
	}
	if (mpEvalResCPU) {
		cudaFreeHost(mpEvalResCPU);
		mpEvalResCPU = nullptr;
	}
	if (mpEvalCoefsDev) {
		cudaFree(mpEvalCoefsDev);
		mpEvalCoefsDev = nullptr;
	}
	if (mpEvalCoefsCPU) {
		cudaFreeHost(mpEvalCoefsCPU);
		mpEvalCoefsCPU = nullptr;
	}
	if (mpRotChans) {
		for (int i = 0; i < mRotChansNum; ++i) {
			ROT_CHANNEL* pCh = &mpRotChans[i];
			if (pCh->pCoefs) {
				::free(pCh->pCoefs);
				pCh->pCoefs = nullptr;
			}
		}
		::free(mpRotChans);
		mpRotChans = nullptr;
		mRotChansNum = 0;
	}
	if (mpPosVecs) {
		::free(mpPosVecs);
		mpPosVecs = nullptr;
		mPosVecsNum = 0;
	}
	if (mpClip) {
		motClipUnload(mpClip);
		mpClip = nullptr;
	}
	mCut = 0;
	mCoefsNum = 0;
	mParamFactor = 0.0f;
}

void cMotion::eval_pos_vecs(float frame) {
	if (!mpClip) return;
	if (!mpPosVecs) return;
	int nfrm = mpClip->nfrm;
	int numNodes = mpClip->nnod;
	int idx = 0;
	for (int i = 0; i < numNodes; ++i) {
		int srcMask = mpClip->nodes[i].trk[POS].dataMask;
		int dataMask = mpClip->nodes[i].trk[POS].dataMask;
		if (srcMask || dataMask) {
			int fno = (int)frame;
			MOT_VEC v = motGetVec(mpClip, i, fno, POS);
			if (fno < nfrm - 1) {
				float t = frame - (float)fno;
				MOT_VEC v1 = motGetVec(mpClip, i, fno + 1, POS);
				for (int j = 0; j < 3; ++j) {
					v.v[j] += (v1.v[j] - v.v[j]) * t;
				}
			}
			mpPosVecs[idx] = v;
			++idx;
		}
	}
}

__host__ __device__
#if 0
void eval_sub(float* pRes, const float* pCoefs, float t, int n, int tid) {
	const float* pRe = &pCoefs[tid * (n + n - 1)];
	const float* pIm = &pRe[n];
	float res = pRe[0];
	for (int i = 1; i < n; ++i) {
		float r = t * (float)i;
		float re = pRe[i];
		float im = pIm[i - 1];
		res += re*cosf(r) + im*sinf(r);
	}
	pRes[tid] = res;
}
#else
// NR ed3: (5.4.6)
void eval_sub(float* pRes, const float* pCoefs, float t, int n, int tid) {
	const float* pRe = &pCoefs[tid * (n + n - 1)];
	const float* pIm = &pRe[n];
	float res = pRe[0];
	float r = t;
	float c = cosf(r);
	float s = sinf(r);
	float a = sinf(r*0.5f);
	a = 2.0f * a*a;
	float b = s;
	float re = pRe[1];
	float im = pIm[0];
	res += re*c + im*s;
	for (int i = 2; i < n; ++i) {
		float ci = c - (a*c + b*s);
		float si = s - (a*s - b*c);
		re = pRe[i];
		im = pIm[i - 1];
		res += re*ci + im*si;
		c = ci;
		s = si;
	}
	pRes[tid] = res;
}
#endif

void cMotion::eval_cpu(float frame) {
	float* pCoefs = mpEvalCoefsCPU;
	float* pRes = mpEvalResCPU;
	if (!pCoefs || !pRes) return;
	float t = frame_to_param(frame);
	int n = mRotChansNum;
//#pragma omp parallel for
	for (int i = 0; i < n; ++i) {
		eval_sub(pRes, pCoefs, t, mCut, i);
	}
	eval_pos_vecs(frame);
}

__global__ void eval_kernel(float* pRes, const float* pCoefs, float t, int n, int nres) {
	int tid = blockIdx.x*blockDim.x + threadIdx.x;
	if (tid >= nres) return;
	eval_sub(pRes, pCoefs, t, n, tid);
}

static int s_blkMin = 64;
static int s_blkMax = 128;

static int calc_thr_num(int nwk) {
	int n = (int)::log2(nwk) - 1;
	if (n < 0) n = 0;
	n = 1 << n;
	if (n < s_blkMin) n = s_blkMin;
	if (n > s_blkMax) n = s_blkMax;
	return n;
}

void cMotion::eval_dev(float frame) {
	float* pCoefs = mpEvalCoefsDev;
	float* pRes = mpEvalResDev;
	if (!pCoefs || !pRes) return;
	float t = frame_to_param(frame);
	int nch = mRotChansNum;
	int nthr = calc_thr_num(nch);
	int nblk = (nch + nthr - 1) / nthr;
	eval_kernel<<<nblk, nthr, 0, 0>>>(pRes, pCoefs, t, mCut, nch);
	cudaMemcpyAsync(mpEvalResCPU, pRes, mRotChansNum * sizeof(float), cudaMemcpyDeviceToHost, 0);
	cudaEventRecord(mEvt, 0);
	eval_pos_vecs(frame);
	while (cudaEventQuery(mEvt) == cudaErrorNotReady) {}
}

static cMotion s_mot;

void init() {
	cudaDeviceProp devProps;
	cudaGetDeviceProperties(&devProps, 0);
	s_blkMax = devProps.maxThreadsPerBlock / 8;
	::printf("device: %s, compute %d.%d\n", devProps.name, devProps.major, devProps.minor);
	::printf("SM count = %d\n", devProps.multiProcessorCount);
	::printf("max thr/SM = %d\n", devProps.maxThreadsPerMultiProcessor);
	::printf("max thr/blk = %d\n", devProps.maxThreadsPerBlock);
	::printf("concurrent exec = %s\n", devProps.concurrentKernels ? "yes" : "no");
	::printf("\n");

	const char* pPath = "test.mclp";
	s_mot.load(pPath);
	::printf("#rot chans = %d\n", s_mot.get_nrot());
	::printf("#pos vecs = %d\n", s_mot.get_npos());
}


double res_l2() {
	double res = 0;
	int n = s_mot.get_nrot();
	float* p = s_mot.get_res_ptr();
	if (p) {
		for (int i = 0; i < n; ++i) {
			res += p[i] * p[i];
		}
		res = sqrt(res);
	}
	return res;
}


int main() {
	init();

	const int N = 1000;
	double cpuT = 0.0f;
	double devT = 0.0f;
	double devRes = 0.0;
	double cpuRes = 0.0;
	::printf("-----\n");

	for (int i = 0; i < N; ++i) {
		float frm = s_mot.get_nfrm() * float(i) / float(N);
		int64_t devT0 = timestamp();
		s_mot.eval_dev(frm);
		int64_t devT1 = timestamp();
		double devDT = (double)(devT1 - devT0);
		devT += devDT;
		devRes += res_l2();
	}
	devT /= N;
	::printf("dev res = %.1f\n", devRes);
	::printf("dev t = %.1f\n", devT);

	s_mot.clear_res();

	for (int i = 0; i < N; ++i) {
		float frm = s_mot.get_nfrm() * float(i) / float(N);
		int64_t cpuT0 = timestamp();
		s_mot.eval_cpu(frm);
		int64_t cpuT1 = timestamp();
		double cpuDT = (double)(cpuT1 - cpuT0);
		cpuT += cpuDT;
		cpuRes += res_l2();
	}
	cpuT /= N;
	::printf("cpu res = %.1f\n", cpuRes);
	::printf("cpu t = %.1f\n", cpuT);

	::printf("%f\n", cpuT / devT);

	return 0;
}
