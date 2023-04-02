/* SPDX-License-Identifier: MIT */
/* SPDX-FileCopyrightText: 2023 Sergey Chaban <sergey.chaban@gmail.com> */

/* Convert track data in Houdini bclip files into 32-bit floats. */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static const char* s_pSig = "bclp";

static uint32_t hbinU32(const uint8_t* pMem) {
	uint32_t b3 = pMem[0];
	uint32_t b2 = pMem[1];
	uint32_t b1 = pMem[2];
	uint32_t b0 = pMem[3];
	return (b0 | (b1 << 8) | (b2 << 16) | (b3 << 24));
}

static int32_t hbinI32(const uint8_t* pMem) {
	return (int32_t)hbinU32(pMem);
}

static uint16_t hbinU16(const uint8_t* pMem) {
	uint16_t b1 = pMem[0];
	uint16_t b0 = pMem[1];
	return (b0 | (b1 << 8));
}

static int16_t hbinI16(const uint8_t* pMem) {
	return (int16_t)hbinU16(pMem);
}

static float hbinF32(const uint8_t* pMem) {
	float f;
	uint32_t u = hbinU32(pMem);
	memcpy(&f, &u, sizeof(float));
	return f;
}

static float hbinF64(const uint8_t* pMem) {
	double d;
	int i;
	uint8_t b[8];
	/* assume LE */
	for (i = 0; i < 8; ++i) {
		b[7 - i] = pMem[i];
	}
	memcpy(&d, b, sizeof(double));
	return d;
}

static size_t file_size(FILE* pFile) {
	size_t size = 0;
	if (pFile) {
		long fpos = ftell(pFile);
		if (fpos >= 0) {
			if (fseek(pFile, 0, SEEK_END) == 0) {
				long flen = ftell(pFile);
				if (flen >= 0) {
					size = (size_t)flen;
				}
			}
			fseek(pFile, fpos, SEEK_SET);
		}
	}
	return size;
}

static size_t file_read(FILE* pFile, void* pDst, size_t size) {
	size_t nread = 0;
	if (pFile && pDst && size > 0) {
		nread = fread(pDst, 1, size, pFile);
	}
	return nread;
}

static void* bin_load(const char* pPath, size_t* pSize) {
	void* pData = NULL;
	size_t size = 0;
	if (pPath) {
		FILE* pFile = fopen(pPath, "rb");
		if (pFile) {
			size = file_size(pFile);
			if (size > 0) {
				pData = malloc(size);
				if (pData) {
					fseek(pFile, 0, SEEK_SET);
					size = file_read(pFile, pData, size);
				}
			}
			fclose(pFile);
		}
	}
	if (pSize) {
		*pSize = size;
	}
	return pData;
}

static int bclip_valid(const void* pBclip) {
	int res = 0;
	if (pBclip) {
		if (memcmp(pBclip, s_pSig, 4) == 0) {
			res = 1;
		}
	}
	return res;
}

static const uint8_t* bclip_find_pkt(const void* pBclip, const int32_t pktTag) {
	const uint8_t* pPkt = NULL;
	if (bclip_valid(pBclip)) {
		const uint8_t* pTop = (const uint8_t*)pBclip;
		const uint8_t* pMem = pTop + 4;
		while (1) {
			int32_t pktSize = hbinI32(pMem);
			if (pktSize > 0) {
				uint16_t tag = hbinU16(pMem + 4);
				if (tag != 0xF) { break; }
				tag = hbinU16(pMem + 6);
				if (tag == pktTag) { pPkt = pMem; break; }
				if (tag == 0) { /* END */ break; }
				pMem += pktSize;
			} else {
				break;
			}
		}
	}
	return pPkt;
}

static int32_t bclip_version(const void* pBclip) {
	int32_t ver = 0;
	const uint8_t* pVer = bclip_find_pkt(pBclip, 9);
	if (pVer) { ver = hbinI32(pVer + 8); }
	return ver;
}

static int bclip_is_data_f64(const void* pBclip) {
	int res = 0;
	const uint8_t* pTyp = bclip_find_pkt(pBclip, 8);
	if (pTyp) { res = !!pTyp[8]; }
	return res;
}

static int bclip_is_info_f64(const void* pBclip) {
	int res = 0;
	const uint8_t* pTyp = bclip_find_pkt(pBclip, 0xA);
	if (pTyp) { res = !!pTyp[8]; }
	return res;
}

static float bclip_sample_rate(const void* pBclip) {
	float rate = 0.0f;
	const uint8_t* pRate = bclip_find_pkt(pBclip, 1);
	if (pRate) {
		if (bclip_is_info_f64(pBclip)) {
			rate = (float)hbinF64(pRate + 8);
		} else {
			rate = hbinF32(pRate + 8);
		}
	}
	return rate;
}

static float bclip_start_index(const void* pBclip) {
	float start = 0.0f;
	const uint8_t* pStart = bclip_find_pkt(pBclip, 2);
	if (pStart) {
		if (bclip_is_info_f64(pBclip)) {
			start = (float)hbinF64(pStart + 8);
		} else {
			start = hbinF32(pStart + 8);
		}
	}
	return start;
}

static int32_t bclip_track_length(const void* pBclip) {
	int32_t len = 0;
	const uint8_t* pLen = bclip_find_pkt(pBclip, 3);
	if (pLen) { len = hbinI32(pLen + 8); }
	return len;
}

static int32_t bclip_num_tracks(const void* pBclip) {
	int32_t num = 0;
	const uint8_t* pTrk = bclip_find_pkt(pBclip, 5);
	if (pTrk) { num = hbinI32(pTrk + 8); }
	return num;
}

static uint32_t bclip_track_data_size(const void* pBclip) {
	uint32_t size = 0;
	const uint8_t* pTrk = bclip_find_pkt(pBclip, 5);
	if (pTrk) { size = hbinU32(pTrk); }
	return size;
}

static void patch_u32(uint8_t* pDst, const uint32_t x) {
	pDst[0] = (uint8_t)((x >> 24) & 0xFF);
	pDst[1] = (uint8_t)((x >> 16) & 0xFF);
	pDst[2] = (uint8_t)((x >> 8) & 0xFF);
	pDst[3] = (uint8_t)(x & 0xFF);
}

static uint8_t* cvt_trk_data(uint8_t* pBuf, const void* pBclip) {
	const uint8_t* pTrk = NULL;
	uint8_t* pDst = pBuf;
	int32_t i;
	int32_t j;
	int32_t ntrk = 0;
	int32_t nsmp = 0;
	if (!pBuf) return NULL;
	if (!bclip_valid(pBclip)) return NULL;
	if (!bclip_is_data_f64(pBclip)) return NULL;

	pTrk = bclip_find_pkt(pBclip, 5);
	if (!pTrk) return NULL;
	memcpy(pDst, pTrk, 0xC);
	pDst += 0xC;
	pTrk += 0xC;
	ntrk = bclip_num_tracks(pBclip);
	nsmp = bclip_track_length(pBclip);
	for (i = 0; i < ntrk; ++i) {
		while (1) {
			int32_t pktLen = hbinI32(pTrk);
			int32_t pktTag = hbinU16(pTrk + 4);
			if (pktTag != 0x10) return NULL;
			pktTag = hbinU16(pTrk + 6);
			if (pktTag == 1 || pktTag == 0) {
				/* copy NAME, END */
				memcpy(pDst, pTrk, pktLen);
				pDst += pktLen;
			} else if (pktTag == 2) {
				/* convert DATA */
				uint8_t* pData = pDst;
				memcpy(pDst, pTrk, 8);
				pDst += 8;
				const uint8_t* pSrc = pTrk + 8;
				for (j = 0; j < nsmp; ++j) {
					float smp = (float)hbinF64(pSrc);
					patch_u32(pDst, ((uint32_t*)&smp)[0]);
					pDst += 4;
					pSrc += 8;
				}
				patch_u32(pData, (uint32_t)(pDst - pData));
			}
			pTrk += pktLen;
			if (pktTag == 0) { /* END */
				break;
			}
		}
	}
	patch_u32(pBuf, (uint32_t)(pDst - pBuf));
	return pDst;
}

static uint8_t* buf_alloc(const size_t size) {
	return (uint8_t*)malloc(size);
}

static void buf_free(uint8_t* pBuf) {
	if (pBuf) {
		free(pBuf);
	}
}

static int exec_cvt(const char* pInPath, const char* pOutPath) {
	int res = 0;
	int32_t ver = 0;
	int infoF64 = 0;
	int dataF64 = 0;
	float fps = 0.0f;
	float start = 0.0f;
	int32_t tlen = 0;
	int32_t ntrk = 0;
	void* pBclip = NULL;
	size_t bclipSize = 0;
	if (pInPath) {
		pBclip = bin_load(pInPath, &bclipSize);
		if (pBclip) {
			if (bclipSize < 0x10 || !bclip_valid(pBclip)) {
				res = -2;
			} else {
				ver = bclip_version(pBclip);
				infoF64 = bclip_is_info_f64(pBclip);
				dataF64 = bclip_is_data_f64(pBclip);
				fps = bclip_sample_rate(pBclip);
				start = bclip_start_index(pBclip);
				tlen = bclip_track_length(pBclip);
				ntrk = bclip_num_tracks(pBclip);
				printf("bclip: %ld bytes\n", bclipSize);
				printf("version: %d, infoF64: %s, dataF64: %s\n",
				       ver, infoF64 ? "yes" : "no", dataF64 ? "yes" : "no");
				if (fps > 1.0f) {
					printf("FPS: %.2f\n", fps);
				} else {
					printf("FPS: %.10f\n", fps);
				}
				printf("start: %.2f\n", start);
				printf("length: %d (0x%x)\n", tlen, tlen);
				printf("num tracks: %d (0x%x)\n", ntrk, ntrk);
				if (dataF64 && pOutPath) {
					size_t i;
					static uint16_t infoPkts[] = {
						0x9, /* VERSION */
						0xA, /* FLOATFIELDDATATYPE */
						0x1, /* SAMPLERATE */
						0x2, /* START */
						0x8, /* TRACKDATATYPE */
						0x3  /* TRACKLENGTH */
					};
					size_t numInfoPkts = sizeof(infoPkts) / sizeof(infoPkts[0]);
					uint32_t orgTrkSize = bclip_track_data_size(pBclip);
					printf("trk data size: (0x%x) bytes\n", orgTrkSize);
					uint32_t tmpSize = 4;
					for (i = 0; i < numInfoPkts; ++i) {
						int32_t infoPktTag = infoPkts[i];
						const uint8_t* pInfoPkt = bclip_find_pkt(pBclip, infoPktTag);
						if (pInfoPkt) {
							tmpSize += hbinU32(pInfoPkt);
						} else {
							tmpSize = 0;
							break;
						}
					}
					if (tmpSize > 0) {
						FILE* pOut = NULL;
						uint8_t* pBuf = NULL;
						tmpSize += orgTrkSize + 8;
						pBuf = buf_alloc(tmpSize);
						if (pBuf) {
							uint8_t* pDst = pBuf;
							memcpy(pDst, s_pSig, 4);
							pDst += 4;
							for (i = 0; i < numInfoPkts; ++i) {
								int32_t infoPktTag = infoPkts[i];
								const uint8_t* pInfoPkt = bclip_find_pkt(pBclip, infoPktTag);
								if (pInfoPkt) {
									uint32_t infoPktLen = hbinU32(pInfoPkt);
									memcpy(pDst, pInfoPkt, infoPktLen);
									if (infoPktTag == 8) {
										/* TRACKDATATYPE */
										pDst[8] = 0;
									}
									pDst += infoPktLen;
								}
							}
							pDst = cvt_trk_data(pDst, pBclip);
							pOut = fopen(pOutPath, "wb");
							if (pOut) {
								static uint8_t end[] = {0, 0, 0, 8, 0, 0xF, 0, 0};
								memcpy(pDst, end, 8);
								pDst += 8;
								size_t outSize = (size_t)(pDst - pBuf);
								fwrite(pBuf, 1, outSize, pOut);
								fclose(pOut);
							}
							buf_free(pBuf);
							pBuf = NULL;
						}
					}
				} else {
					res = -3;
				}
			}
		} else {
			res = -1;
		}
	}
	if (pBclip) {
		free(pBclip);
		pBclip = NULL;
	}
	return res;
}


int main(int argc, char* argv[]) {
	const char* pInPath = NULL;
	const char* pOutPath = NULL;
	if (argc < 2) {
		printf("bclip_f32 <in_path> <out_path>\n");
		return -1;
	}
	pInPath = argv[1];
	if (argc > 2) {
		pOutPath = argv[2];
	}
	return exec_cvt(pInPath, pOutPath);
}
