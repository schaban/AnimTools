#ifdef _MSC_VER
#	define _CRT_SECURE_NO_WARNINGS
#endif

#include "motclip.h"

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

void init() {
	MOT_CLIP* pClip = clipLoad("../data/walk.mclp");
	if (!pClip) return;
	s_pClip = pClip;
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
