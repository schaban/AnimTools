/*
 * Motion Clip playback functions
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#include "motclip.h"

const char g_motClipFmt[4] = { 'M', 'C', 'L', 'P' };
const char g_motLibFmt[4] = { 'M', 'L', 'I', 'B' };

static float sq(float x) {
	return x*x;
}

static float sinc(float x) {
	if (fabsf(x) < 1.0e-4f) {
		return 1.0f;
	}
	return sinf(x) / x;
}

static float lerp(float a, float b, float t) {
	return a + (b - a)*t;
}

float motDegrees(float rad) {
	return rad * (45.0f / atanf(1.0f));
}

float motRadians(float deg) {
	return deg * (atanf(1.0f) / 45.0f);
}

MOT_VEC motVecLerp(const MOT_VEC v1, const MOT_VEC v2, float t) {
	MOT_VEC v;
	int i;
	for (i = 0; i < 3; ++i) {
		v.s[i] = lerp(v1.s[i], v2.s[i], t);
	}
	return v;
}

void motMtxMul(MOT_MTX* pRes, const MOT_MTX* pMtx1, const MOT_MTX* pMtx2) {
	MOT_MTX m;
	int i, j, k;
	float* pM0 = &m[0][0];
	const float* pM1 = &(*pMtx1)[0][0];
	const float* pM2 = &(*pMtx2)[0][0];
	for (i = 0; i < 4 * 4; ++i) {
		pM0[i] = 0.0f;
	}
	for (i = 0; i < 4; ++i) {
		int ri = i * 4;
		for (j = 0; j < 4; ++j) {
			int rj = j * 4;
			float v = pM1[ri + j];
			for (k = 0; k < 4; ++k) {
				pM0[ri + k] += v*pM2[rj + k];
			}
		}
	}
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 4; ++j) {
			(*pRes)[i][j] = m[i][j];
		}
	}
}

static void qmtx(float m[3][3], const MOT_QUAT q) {
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;
	m[0][0] = 1.0f - 2.0f*y*y - 2.0f*z*z;
	m[0][1] = 2.0f*x*y + 2.0f*w*z;
	m[0][2] = 2.0f*x*z - 2.0f*w*y;
	m[1][0] = 2.0f*x*y - 2.0f*w*z;
	m[1][1] = 1.0f - 2.0f*x*x - 2.0f*z*z;
	m[1][2] = 2.0f*y*z + 2.0f*w*x;
	m[2][0] = 2.0f*x*z + 2.0f*w*y;
	m[2][1] = 2.0f*y*z - 2.0f*w*x;
	m[2][2] = 1.0f - 2.0f*x*x - 2.0f*y*y;
}

void motMakeTransform(MOT_MTX* pMtx, const MOT_VEC tns, const MOT_QUAT rot, const MOT_VEC scl, E_MOT_XORD xord) {
	MOT_MTX ms[3];
	float qm[3][3];
	const uint8_t S = 0;
	const uint8_t R = 1;
	const uint8_t T = 2;
	int i, j, k;
	int i0, i1, i2;
	if (!pMtx) return;
	switch (xord) {
		default:
		case XORD_SRT: i0 = S; i1 = R; i2 = T; break;
		case XORD_STR: i0 = S; i1 = T; i2 = R; break;
		case XORD_RST: i0 = R; i1 = S; i2 = T; break;
		case XORD_RTS: i0 = R; i1 = T; i2 = S; break;
		case XORD_TSR: i0 = T; i1 = S; i2 = R; break;
		case XORD_TRS: i0 = T; i1 = R; i2 = S; break;
	};
	qmtx(qm, rot);
	for (k = 0; k < 3; ++k) {
		for (i = 0; i < 4; ++i) {
			for (j = 0; j < 4; ++j) {
				if (i == j) {
					float s = 1.0f;
					if (k == 0 && i < 3) {
						s = scl.s[i];
					}
					ms[k][i][j] = s;
				} else {
					float t = 0.0f;
					if (i == 3 && k == 2 && i < 3) {
						t = tns.s[i];
					}
					ms[k][i][j] = t;
				}
			}
		}
	}
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			ms[1][i][j] = qm[i][j];
		}
	}
	motMtxMul(pMtx, (const MOT_MTX*)&ms[i0], (const MOT_MTX*)&ms[i1]);
	motMtxMul(pMtx, (const MOT_MTX*)pMtx, (const MOT_MTX*)&ms[i2]);
}

void motMakeTransformTR(MOT_MTX* pMtx, const MOT_VEC tns, const MOT_QUAT rot, E_MOT_XORD xord) {
	float qm[3][3];
	int i, j;
	if (!pMtx) return;
	qmtx(qm, rot);
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			(*pMtx)[i][j] = qm[i][j];
		}
	}
	switch (xord) {
		default:
		case XORD_SRT:
		case XORD_RST:
		case XORD_RTS:
			for (i = 0; i < 3; ++i) {
				(*pMtx)[3][i] = tns.s[i];
			}
			break;
		case XORD_STR:
		case XORD_TSR:
		case XORD_TRS:
			for (i = 0; i < 3; ++i) {
				(*pMtx)[3][i] = 0.0f;
			}
			for (i = 0; i < 3; ++i) {
				for (j = 0; j < 3; ++j) {
					(*pMtx)[3][i] += tns.s[j] * qm[i][j];
				}
			}
			break;
	};
	for (i = 0; i < 4; ++i) {
		(*pMtx)[i][3] = 0.0f;
	}
	(*pMtx)[3][3] = 1.0f;
}

void motMakeTransformR(MOT_MTX* pMtx, const MOT_QUAT rot) {
	float qm[3][3];
	int i, j;
	if (!pMtx) return;
	qmtx(qm, rot);
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 3; ++j) {
			(*pMtx)[i][j] = qm[i][j];
		}
	}
	for (i = 0; i < 4; ++i) {
		(*pMtx)[i][3] = 0.0f;
	}
	for (j = 0; j < 3; ++j) {
		(*pMtx)[3][j] = 0.0f;
	}
	(*pMtx)[3][3] = 1.0f;
}

void motMakeTransformT(MOT_MTX* pMtx, const MOT_VEC tns) {
	int i, j;
	if (!pMtx) return;
	for (i = 0; i < 4; ++i) {
		for (j = 0; j < 4; ++j) {
			float v = 0.0f;
			if (i == 3 && j < 3) {
				v = tns.s[j];
			} else if (i == j) {
				v = 1.0f;
			}
			(*pMtx)[i][j] = v;
		}
	}
}

MOT_QUAT motQuatFromRadians(float rx, float ry, float rz, E_MOT_RORD rord) {
	MOT_QUAT q;
	MOT_QUAT qs[3];
	int i, j;
	int ix, iy, iz;
	float hx = rx * 0.5f;
	float hy = ry * 0.5f;
	float hz = rz * 0.5f;
	switch (rord) {
		default:
		case RORD_XYZ: ix = 0; iy = 1; iz = 2; break;
		case RORD_XZY: ix = 0; iy = 2; iz = 1; break;
		case RORD_YXZ: ix = 1; iy = 0; iz = 2; break;
		case RORD_YZX: ix = 2; iy = 0; iz = 1; break;
		case RORD_ZXY: ix = 1; iy = 2; iz = 0; break;
		case RORD_ZYX: ix = 2; iy = 1; iz = 0; break;
	}
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < 4; ++j) {
			qs[i].s[j] = 0.0f;
		}
	}
	qs[ix].x = sinf(hx);
	qs[ix].w = cosf(hx);
	qs[iy].y = sinf(hy);
	qs[iy].w = cosf(hy);
	qs[iz].z = sinf(hz);
	qs[iz].w = cosf(hz);
	q = motQuatMul(qs[2], qs[1]);
	q = motQuatMul(q, qs[0]);
	return q;
}

MOT_QUAT motQuatFromDegrees(float dx, float dy, float dz, E_MOT_RORD rord) {
	return motQuatFromRadians(motRadians(dx), motRadians(dy), motRadians(dz), rord);
}

MOT_QUAT motQuatMul(const MOT_QUAT q1, const MOT_QUAT q2) {
	MOT_QUAT q;
	q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
	q.y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z;
	q.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x;
	q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	return q;
}

MOT_QUAT motQuatNormalize(const MOT_QUAT q) {
	MOT_QUAT qn;
	int i;
	float s;
	float sqm = 0.0f;
	for (i = 0; i < 4; ++i) {
		sqm += sq(q.s[i]);
	}
	s = sqrtf(sqm);
	if (s > 0.0f) {
		s = 1.0f / s;
		for (i = 0; i < 4; ++i) {
			qn.s[i] = q.s[i] * s;
		}
	}
	return qn;
}

MOT_QUAT motQuatExp(const MOT_VEC v) {
	MOT_QUAT q;
	float ha = sqrtf(sq(v.x) + sq(v.y) + sq(v.z));
	float s = sinc(ha);
	q.x = v.x * s;
	q.y = v.y * s;
	q.z = v.z * s;
	q.w = cosf(ha);
	return motQuatNormalize(q);
}

MOT_QUAT motQuatSlerp(const MOT_QUAT q1, const MOT_QUAT q2, float t) {
	MOT_QUAT q;
	int i;
	float ang;
	float r, s;
	float u = 0.0f;
	float v = 0.0f;
	float d = 0.0f;
	for (i = 0; i < 4; ++i) {
		d += q1.s[i] * q2.s[i];
	}
	d = d < 0.0f ? -1.0f : 1.0f;
	for (i = 0; i < 4; ++i) {
		u += sq(q1.s[i] - q2.s[i]*d);
		v += sq(q1.s[i] + q2.s[i]*d);
	}
	ang = 2.0f * atan2f(sqrtf(u), sqrtf(v));
	s = 1.0f - t;
	r = 1.0f / sinc(ang);
	s = sinc(ang*s) * r * s;
	t = sinc(ang*t) * r * t;
	t *= d;
	for (i = 0; i < 4; ++i) {
		q.s[i] = q1.s[i]*s + q2.s[i]*t;
	}
	return motQuatNormalize(q);
}

static float rlimit(float rad) {
	const float pi = atanf(1.0f) * 4.0f;
	rad = fmodf(rad, pi*2);
	if (fabsf(rad) > pi) {
		if (rad < 0.0f) {
			rad = pi*2 + rad;
		} else {
			rad = rad - pi*2;
		}
	}
	return rad;
}

MOT_VEC motQuatToRadians(const MOT_QUAT q, E_MOT_RORD rord) {
	const float eps = 1.0e-6f;
	MOT_VEC r = { 0.0f, 0.0f, 0.0f };
	int singleAxis = 0;
	int axisMask = 0;
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w < -1.0f ? -1.0f : q.w > 1.0f ? 1.0f : q.w;
	if (fabsf(x) < eps) axisMask |= 1;
	if (fabsf(y) < eps) axisMask |= 2;
	if (fabsf(z) < eps) axisMask |= 4;
	if (fabsf(w) < eps) axisMask |= 8;
	switch (axisMask) {
		case 6: /* 0110 -> X */
			r.x = acosf(w) * 2.0f;
			if (x < 0.0f) r.x = -r.x;
			r.x = rlimit(r.x);
			singleAxis = 1;
			break;
		case 5: /* 0101 -> Y */
			r.y = acosf(w) * 2.0f;
			if (y < 0.0f) r.y = -r.y;
			r.y = rlimit(r.y);
			singleAxis = 1;
			break;
		case 3: /* 0011 -> Z */
			r.z = acosf(w) * 2.0f;
			if (z < 0.0f) r.z = -r.z;
			r.z = rlimit(r.z);
			singleAxis = 1;
			break;
		case 7: /* 0111 -> identity */
			singleAxis = 1;
			break;
	}
	if (!singleAxis) {
		int i, i0, i1, i2;
		float sgn;
		float m[3][3];
		float s, c;
		switch (rord) {
			default:
			case RORD_XYZ: i0 = 0; i1 = 1; i2 = 2; sgn =  1.0f; break;
			case RORD_XZY: i0 = 0; i1 = 2; i2 = 1; sgn = -1.0f; break;
			case RORD_YXZ: i0 = 1; i1 = 0; i2 = 2; sgn = -1.0f; break;
			case RORD_YZX: i0 = 1; i1 = 2; i2 = 0; sgn =  1.0f; break;
			case RORD_ZXY: i0 = 2; i1 = 0; i2 = 1; sgn =  1.0f; break;
			case RORD_ZYX: i0 = 2; i1 = 1; i2 = 0; sgn = -1.0f; break;
		}
		qmtx(m, q);
		r.s[i0] = atan2f(m[1][2], m[2][2]);
		r.s[i1] = atan2f(-m[0][2], sqrtf(m[0][0]*m[0][0] + m[0][1]*m[0][1]));
		s = sinf(r.s[i0]);
		c = cosf(r.s[i0]);
		r.s[i2] = atan2f(s*m[2][0] - c*m[1][0], c*m[1][1] - s*m[2][1]);
		for (i = 0; i < 3; ++i) {
			r.s[i] *= sgn;
		}
		for (i = 0; i < 3; ++i) {
			r.s[i] = rlimit(r.s[i]);
		}
	}
	return r;
}

MOT_VEC motQuatToDegrees(const MOT_QUAT q, E_MOT_RORD rord) {
	MOT_VEC r = motQuatToRadians(q, rord);
	int i;
	for (i = 0; i < 3; ++i) {
		r.s[i] = motDegrees(r.s[i]);
	}
	return r;
}

int motClipHeaderCk(const MOT_CLIP* pClip) {
	if (!pClip) return 0;
	int i;
	for (i = 0; i < 4; ++i) {
		if (pClip->fmt[i] != g_motClipFmt[i]) return 0;
	}
	return 1;
}

int motClipNodeIdxCk(const MOT_CLIP* pClip, int nodeIdx) {
	return !!(pClip && ((uint32_t)nodeIdx < pClip->nnod));
}

int motFrameNoCk(const MOT_CLIP* pClip, int fno) {
	return !!(pClip && ((uint32_t)fno < pClip->nfrm));
}

int motNodeTrackCk(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk) {
	int res = 0;
	if (pClip && motClipNodeIdxCk(pClip, nodeIdx)) {
		switch (trk) {
			case TRK_POS:
			case TRK_ROT:
			case TRK_SCL:
				res = pClip->nodes[nodeIdx].trk[trk].srcMask != 0;
				break;
			default:
				break;
		}
	}
	return res;
}

static uint32_t strhash(uint32_t* pLen, const char* pStr) {
	uint32_t c;
	uint32_t len = 0;
	uint32_t h = 2166136261U;
	while (c = *pStr++) {
		h *= 16777619U;
		h ^= c;
		++len;
	}
	*pLen = len;
	return h;
}

static int hfind(const uint32_t* pHashes, int n, uint32_t h) {
	const uint32_t* p = pHashes;
	uint32_t cnt = (uint32_t)n;
	while (cnt > 1) {
		uint32_t mid = cnt / 2;
		const uint32_t* pm = &p[mid];
		uint32_t ck = *pm;
		p = (h < ck) ? p : pm;
		cnt -= mid;
	}
	return (int)(p - pHashes) + ((p != pHashes) & (*p > h));
}

int motFindClipNode(const MOT_CLIP* pClip, const char* pName) {
	int idx = -1;
	const int binSearchThreshold = 20;
	if (pClip && pName) {
		int i;
		int n = pClip->nnod;
		if (n > binSearchThreshold && pClip->hash) {
			uint32_t len;
			uint32_t h = strhash(&len, pName);
			if ((size_t)len < sizeof(MOT_STRING) - 2) {
				uint32_t* pHashes = (uint32_t*)((uint8_t*)pClip + pClip->hash);
				int hidx = hfind(pHashes, n, h);
				if (h == pHashes[hidx]) {
					int nc = 1;
					for (i = hidx; --i >= 0;) {
						if (pHashes[i] != h) break;
						--idx;
						++nc;
					}
					for (int i = 0; i < nc; ++i) {
						int tidx = hidx + i;
						const MOT_NODE* pNode = &pClip->nodes[tidx];
						if (len == pNode->name.len) {
							if (memcmp(pNode->name.chr, pName, len) == 0) {
								idx = tidx;
								break;
							}
						}
					}
				}
			}
		} else {
			for (i = 0; i < n; ++i) {
				if (strcmp(pClip->nodes[i].name.chr, pName) == 0) {
					idx = i;
					break;
				}
			}
		}
	}
	return idx;
}

float* motGetTrackData(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk) {
	float* p = NULL;
	if (pClip && motClipNodeIdxCk(pClip, nodeIdx)) {
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
	float* p = NULL;
	if ((uint32_t)chIdx < 3) {
		float* pTrk = motGetTrackData(pClip, nodeIdx, trk);
		if (pTrk) {
			int i;
			int dataMask = pClip->nodes[nodeIdx].trk[(int)trk].dataMask;
			for (i = 0; i < 3; ++i) {
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

E_MOT_RORD motGetRotOrd(const MOT_CLIP* pClip, int nodeIdx) {
	return motClipNodeIdxCk(pClip, nodeIdx) ? (E_MOT_RORD)pClip->nodes[nodeIdx].rord : RORD_XYZ;
}

E_MOT_XORD motGetXformOrd(const MOT_CLIP* pClip, int nodeIdx) {
	return motClipNodeIdxCk(pClip, nodeIdx) ? (E_MOT_XORD)pClip->nodes[nodeIdx].xord : XORD_SRT;
}

MOT_VEC motGetVec(const MOT_CLIP* pClip, int nodeIdx, int fno, E_MOT_TRK trk) {
	MOT_VEC v = {0.0f, 0.0f, 0.0f};
	if (pClip && motClipNodeIdxCk(pClip, nodeIdx) && motFrameNoCk(pClip, fno)) {
		int i;
		float* p = motGetTrackData(pClip, nodeIdx, trk);
		if (p) {
			int itrk = (int)trk;
			if (itrk < 3) {
				float defVal = trk == TRK_SCL ? 1.0f : 0.0f;
				int dataMask = pClip->nodes[nodeIdx].trk[itrk].dataMask;
				int srcMask = pClip->nodes[nodeIdx].trk[itrk].srcMask;
				int vsize = 0;
				for (i = 0; i < 3; ++i) {
					if (dataMask & (1 << i)) ++vsize;
				}
				p += fno * vsize;
				for (i = 0; i < 3; ++i) {
					if (dataMask & (1 << i)) {
						v.s[i] = *p++;
					} else if (srcMask & (1 << i)) {
						v.s[i] = pClip->nodes[nodeIdx].trk[itrk].vmin.s[i];
					} else {
						v.s[i] = defVal;
					}
				}
			}
		}
	}
	return v;
}

MOT_VEC motGetPos(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	return motGetVec(pClip, nodeIdx, fno, TRK_POS);
}

MOT_VEC motGetScl(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	if (!motNodeTrackCk(pClip, nodeIdx, TRK_SCL)) {
		MOT_VEC s = { 1.0f, 1.0f, 1.0f };
		return s;
	}
	return motGetVec(pClip, nodeIdx, fno, TRK_SCL);
}

MOT_QUAT motGetQuat(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	return motQuatExp(motGetVec(pClip, nodeIdx, fno, TRK_ROT));
}

MOT_VEC motGetRadians(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	return motQuatToRadians(motGetQuat(pClip, nodeIdx, fno), motGetRotOrd(pClip, nodeIdx));
}

MOT_VEC motGetDegrees(const MOT_CLIP* pClip, int nodeIdx, int fno) {
	return motQuatToDegrees(motGetQuat(pClip, nodeIdx, fno), motGetRotOrd(pClip, nodeIdx));
}

typedef struct _MOT_FRAME_INFO {
	float f;
	float t;
	int fno;
	int next;
} MOT_FRAME_INFO;

static MOT_FRAME_INFO finfo(const MOT_CLIP* pClip, float frm) {
	MOT_FRAME_INFO fi;
	int nfrm = pClip->nfrm;
	frm = fabsf(frm);
	fi.f = fmodf(frm, (float)nfrm);
	fi.fno = (int)fi.f;
	fi.t = fi.f - (float)fi.fno;
	fi.next = fi.fno < nfrm - 1 ? fi.fno + 1 : 0;
	return fi;
}

MOT_QUAT motEvalQuat(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	MOT_FRAME_INFO fi;
	MOT_QUAT q = { 0.0f, 0.0f, 0.0f, 1.0f };
	MOT_VEC v;
	if (!pClip || !motClipNodeIdxCk(pClip, nodeIdx)) {
		return q;
	}
	fi = finfo(pClip, frm);
	v = motGetVec(pClip, nodeIdx, fi.fno, TRK_ROT);
	if (fi.t != 0.0f) {
		MOT_VEC vnext = motGetVec(pClip, nodeIdx, fi.next, TRK_ROT);
		v = motVecLerp(v, vnext, fi.t);
	}
	q = motQuatExp(v);
	return q;
}

MOT_QUAT motEvalQuatSlerp(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	MOT_FRAME_INFO fi;
	MOT_QUAT q = { 0.0f, 0.0f, 0.0f, 1.0f };
	if (!pClip || !motClipNodeIdxCk(pClip, nodeIdx)) {
		return q;
	}
	fi = finfo(pClip, frm);
	q = motGetQuat(pClip, nodeIdx, fi.fno);
	if (fi.t != 0.0f) {
		MOT_QUAT qnext = motGetQuat(pClip, nodeIdx, fi.next);
		q = motQuatSlerp(q, qnext, fi.t);
	}
	return q;
}

MOT_VEC motEvalRadians(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	return motQuatToRadians(motEvalQuat(pClip, nodeIdx, frm), motGetRotOrd(pClip, nodeIdx));
}

MOT_VEC motEvalDegrees(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	return motQuatToDegrees(motEvalQuat(pClip, nodeIdx, frm), motGetRotOrd(pClip, nodeIdx));
}

MOT_VEC motEvalPos(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	MOT_FRAME_INFO fi;
	MOT_VEC v = { 0.0f, 0.0f, 0.0f };
	if (!pClip || !motClipNodeIdxCk(pClip, nodeIdx)) {
		return v;
	}
	fi = finfo(pClip, frm);
	v = motGetPos(pClip, nodeIdx, fi.fno);
	if (fi.t != 0.0f) {
		MOT_VEC vnext = motGetPos(pClip, nodeIdx, fi.next);
		v = motVecLerp(v, vnext, fi.t);
	}
	return v;
}

MOT_VEC motEvalScl(const MOT_CLIP* pClip, int nodeIdx, float frm) {
	MOT_FRAME_INFO fi;
	MOT_VEC v = { 1.0f, 1.0f, 1.0f };
	if (!pClip || !motClipNodeIdxCk(pClip, nodeIdx)) {
		return v;
	}
	fi = finfo(pClip, frm);
	v = motGetScl(pClip, nodeIdx, fi.fno);
	if (fi.t != 0.0f) {
		MOT_VEC vnext = motGetScl(pClip, nodeIdx, fi.next);
		v = motVecLerp(v, vnext, fi.t);
	}
	return v;
}

void motEvalTransform(MOT_MTX* pMtx, const MOT_CLIP* pClip, int nodeIdx, float frm, const MOT_VEC* pDefTns) {
	MOT_QUAT q;
	MOT_VEC t;
	MOT_VEC s;
	E_MOT_XORD xord;
	int srt = 0;
	if (!pMtx || !pClip || !motClipNodeIdxCk(pClip, nodeIdx)) return;
	xord = motGetXformOrd(pClip, nodeIdx);
	if (motNodeTrackCk(pClip, nodeIdx, TRK_POS)) srt |= 1;
	if (motNodeTrackCk(pClip, nodeIdx, TRK_ROT)) srt |= 2;
	if (motNodeTrackCk(pClip, nodeIdx, TRK_SCL)) srt |= 4;
	if (srt & 1) {
		t = motEvalPos(pClip, nodeIdx, frm);
	} else {
		if (pDefTns) {
			t = *pDefTns;
			srt |= 1;
		}
	}
	switch (srt) {
		case 0:
			break;
		case 1:
			motMakeTransformT(pMtx, t);
			break;
		case 2:
			q = motEvalQuat(pClip, nodeIdx, frm);
			motMakeTransformR(pMtx, q);
			break;
		case (1 | 2):
			q = motEvalQuat(pClip, nodeIdx, frm);
			motMakeTransformTR(pMtx, t, q, xord);
			break;
		default:
			q = motEvalQuat(pClip, nodeIdx, frm);
			s = motEvalScl(pClip, nodeIdx, frm);
			motMakeTransform(pMtx, t, q, s, xord);
			break;
	}
}
