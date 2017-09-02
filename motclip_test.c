#ifdef _MSC_VER
#	define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef _WIN32
#	define WIN32_LEAN_AND_MEAN 1
#	define NOMINMAX
#	include <Windows.h>
#endif

#include <time.h>

#include "motclip.h"

double timestamp() {
	double ms = 0.0f;
#if defined(_WIN32)
	LARGE_INTEGER frq;
	if (QueryPerformanceFrequency(&frq)) {
		LARGE_INTEGER ctr;
		QueryPerformanceCounter(&ctr);
		ms = ((double)ctr.QuadPart / (double)frq.QuadPart) * 1.0e6;
	}
#else
	struct timespec t;
	if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
		clock_gettime(CLOCK_REALTIME, &t);
	}
	ms = (double)t.tv_nsec*1.0e-3 + (double)t.tv_sec*1.0e6;
#endif
	return ms;
}

MOT_CLIP* clipLoad(const char* pPath) {
	MOT_CLIP* pClip = NULL;
	FILE* f = fopen(pPath, "rb");
	if (f) {
		long len = 0;
		if (0 == fseek(f, 0, SEEK_END)) {
			len = ftell(f);
		}
		fseek(f, 0, SEEK_SET);
		if (len > sizeof(MOT_CLIP)) {
			pClip = (MOT_CLIP*)malloc(len);
			if (pClip) {
				fread(pClip, len, 1, f);
				if (!motClipHeaderCk(pClip)) {
					free(pClip);
					pClip = NULL;
				}
			}
		}
		fclose(f);
	}
	return pClip;
}

void clipUnload(MOT_CLIP* pClip) {
	if (pClip) { free(pClip); }
}

static MOT_CLIP* s_pClip = NULL;

static void verifyFindClipNode(MOT_CLIP* pClip) {
	int i, n, idx;
	if (!pClip) return;
	n = pClip->nnod;
	for (i = 0; i < n; ++i) {
		idx = motFindClipNode(pClip, pClip->nodes[i].name.chr);
		if (idx != i) {
			fprintf(stderr, "[ERR] FindClipNode: %d != %d\n", idx, i);
		}
		idx = motFindClipNode(pClip, "@#$%^");
		if (motClipNodeIdxCk(pClip, idx)) {
			fprintf(stderr, "[ERR] FindClipNode: expected to fail\n");
		}
	}
}

static int smpcmp(const void* pA, const void* pB) {
	double* pSmp1 = (double*)pA;
	double* pSmp2 = (double*)pB;
	double s1 = *pSmp1;
	double s2 = *pSmp2;
	if (s1 > s2) return 1;
	if (s1 < s2) return -1;
	return 0;
}

static double perfsmp(double* pSmps, int nsmp) {
	qsort(pSmps, nsmp, sizeof(double), smpcmp);
	return (nsmp & 1) ? pSmps[(nsmp - 1) / 2] : (pSmps[(nsmp / 2) - 1] + pSmps[nsmp / 2]) * 0.5;
}

#define N_PERF_SMP (100)

static double perfFindClipNodeSub(MOT_CLIP* pClip) {
	int i, ismp, n;
	double smps[N_PERF_SMP];
	double t0, t1;
	double dt = 0;
	const char* pInvalidNames[4];
	char** pValidNames;
	char longName[0x80];
	if (!pClip) return 0.0;
	n = pClip->nnod;
	memset(longName, 0, sizeof(longName));
	for (i = 0; i < (int)sizeof(longName) - 1; ++i) {
		longName[i] = "@#$%"[i & 3];
	}
	pInvalidNames[0] = longName;
	pInvalidNames[1] = "^ACDC";
	pInvalidNames[2] = "#ABBA";
	pInvalidNames[3] = "$123456789";

	pValidNames = (char**)malloc(n * sizeof(char*));
	for (i = 0; i < n; ++i) {
		MOT_NODE* pNode = &pClip->nodes[i];
		const char* pNodeName = pNode->name.chr;
		pValidNames[i] = (char*)malloc(strlen(pNodeName)+1);
		strcpy(pValidNames[i], pNodeName);
	}

	for (ismp = 0; ismp < N_PERF_SMP; ++ismp) {
		int iHit = -1;
		int iMiss = -1;
		t0 = timestamp();
		for (i = 0; i < n; ++i) {
			iHit = motFindClipNode(pClip, pValidNames[i]);
			if (iHit != i) {
				fprintf(stderr, "[ERR] FindClipNode: %d != %d\n", iHit, i);
			}
			iMiss = motFindClipNode(pClip, pInvalidNames[i & 3]);
			if (motClipNodeIdxCk(pClip, iMiss)) {
				fprintf(stderr, "[ERR] FindClipNode: expected to fail\n");
			}
		}
		t1 = timestamp();
		smps[ismp] = t1 - t0;
	}

	for (i = 0; i < n; ++i) {
		free(pValidNames[i]);
	}
	free(pValidNames);

	dt = perfsmp(smps, N_PERF_SMP);
	return dt;
}

static void perfFindClipNode(MOT_CLIP* pClip) {
	uint32_t hsave;
	double dtSeq, dtBin;
	if (!pClip) return;

	hsave = pClip->hash;
	pClip->hash = 0;
	dtSeq = perfFindClipNodeSub(pClip);

	pClip->hash = hsave;
	dtBin = perfFindClipNodeSub(pClip);

	printf("dtSeq: %f\n", dtSeq);
	printf("dtBin: %f\n", dtBin);
	printf("ratio: %f\n", dtSeq / dtBin);
}

void init() {
	const char* pClipName = "../data/walk.mclp";
	MOT_CLIP* pClip = clipLoad(pClipName);
	if (!pClip) return;
	s_pClip = pClip;
	verifyFindClipNode(pClip);
	perfFindClipNode(pClip);
}

void test() {
	MOT_CLIP* pClip = s_pClip;
	int i, j, k;
	int nfrm;
	int nnod;
	int npos;
	int nrot;
	int nscl;
	float frm;
	int nsub = 4;
	FILE* pOut = NULL;
	MOT_VEC* pRot = NULL;
	MOT_MTX* pMtx = NULL;
	int midx;
	if (!pClip) return;

	nfrm = pClip->nfrm;
	nnod = pClip->nnod;
	npos = 0;
	nrot = 0;
	nscl = 0;
	for (i = 0; i < nnod; ++i) {
		if (motNodeTrackCk(pClip, i, TRK_POS)) {
			++npos;
		}
		if (motNodeTrackCk(pClip, i, TRK_ROT)) {
			++nrot;
		}
		if (motNodeTrackCk(pClip, i, TRK_SCL)) {
			++nscl;
		}
	}
	printf("motion clip: %s\n", pClip->name.chr);
	printf("#nod: %d\n", nnod);
	printf("#frm: %d\n", nfrm);
	printf("#pos tracks: %d\n", npos);
	printf("#rot tracks: %d\n", nrot);
	printf("#scl tracks: %d\n", nscl);

	pOut = fopen("../dump.clip", "w");

	fprintf(pOut, "{\n");
	fprintf(pOut, "  rate = %.1f\n", pClip->rate);
	fprintf(pOut, "  start = -1\n");
	fprintf(pOut, "  tracklength = %d\n", nfrm * nsub);
	fprintf(pOut, "  tracks = %d\n", nrot * 3);

	pRot = (MOT_VEC*)malloc(nfrm * nsub * sizeof(MOT_VEC));
	pMtx = (MOT_MTX*)malloc(nnod * nfrm * nsub * sizeof(MOT_MTX));
	midx = 0;
	for (i = 0; i < nnod; ++i) {
		frm = 0.0f;
		for (k = 0; k < nfrm * nsub; ++k) {
			motEvalTransform(&pMtx[midx], pClip, i, frm, NULL);
			frm += 1.0f / (float)nsub;
			++midx;
		}
		if (motNodeTrackCk(pClip, i, TRK_ROT)) {
			frm = 0.0f;
			for (k = 0; k < nfrm * nsub; ++k) {
				pRot[k] = motEvalDegrees(pClip, i, frm);
				frm += 1.0f / (float)nsub;
			}
			for (k = 0; k < nfrm * nsub; ++k) {
				E_MOT_RORD rord = motGetRotOrd(pClip, i);
				MOT_QUAT q = motQuatFromDegrees(pRot[k].x, pRot[k].y, pRot[k].z, rord);
				pRot[k] = motQuatToDegrees(q, rord);
			}
			for (j = 0; j < 3; ++j) {
				if (pOut) {
					fprintf(pOut, "  {\n");
					fprintf(pOut, "    name = %s:r%c\n", pClip->nodes[i].name.chr, "xyz"[j]);
					fprintf(pOut, "    data =");
					for (k = 0; k < nfrm * nsub; ++k) {
						fprintf(pOut, " %f", pRot[k].s[j]);
					}
					fprintf(pOut, "\n");
					fprintf(pOut, "  }\n");
				}
			}
		}
	}
	fprintf(pOut, "}\n");
	if (pOut) {
		fclose(pOut);
		pOut = NULL;
	}
}

int main() {
	init();
	test();
	return 0;
}
