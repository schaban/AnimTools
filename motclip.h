/*
 * Motion Clip playback functions
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>

#ifdef __cplusplus
#	define MOT_EXTERN_FUNC extern "C"
#	define MOT_EXTERN_DATA extern "C"
#else
#	define MOT_EXTERN_FUNC
#	define MOT_EXTERN_DATA extern
#endif

typedef enum _E_MOT_TRK { TRK_POS, TRK_ROT, TRK_SCL } E_MOT_TRK;
typedef enum _E_MOT_RORD { RORD_XYZ, RORD_XZY, RORD_YXZ, RORD_YZX, RORD_ZXY, RORD_ZYX } E_MOT_RORD;
typedef enum _E_MOT_XORD { XORD_SRT, XORD_STR, XORD_RST, XORD_RTS, XORD_TSR, XORD_TRS } E_MOT_XORD;

typedef struct _MOT_STRING {
	uint8_t len;
	char    chr[0x40-1];
} MOT_STRING;

typedef float MOT_MTX[4][4];

typedef union _MOT_VEC {
	struct { float x, y, z; };
	float s[3];
} MOT_VEC;

typedef union _MOT_QUAT {
	struct { float x, y, z, w; };
	float s[4];
} MOT_QUAT;

typedef struct _MOT_TRACK {
	MOT_VEC vmin;
	MOT_VEC vmax;
	uint8_t srcMask;
	uint8_t dataMask;
	uint8_t reserved[6];
} MOT_TRACK;

typedef struct _MOT_NODE {
	MOT_STRING name;
	uint32_t offs[3];
	uint8_t xord;
	uint8_t rord;
	uint8_t reserved[2];
	MOT_TRACK trk[3];
} MOT_NODE;

typedef struct _MOT_CLIP {
	char       fmt[4];
	uint32_t   size;
	float      rate;
	uint32_t   nfrm;
	uint32_t   nnod;
	uint32_t   hash;
	uint32_t   ext;
	uint32_t   pad;
	MOT_STRING name;
	MOT_NODE   nodes[1];
} MOT_CLIP;

MOT_EXTERN_DATA const char g_motClipFmt[4];
MOT_EXTERN_DATA const char g_motLibFmt[4];

MOT_EXTERN_FUNC float motDegrees(float rad);
MOT_EXTERN_FUNC float motRadians(float deg);

MOT_EXTERN_FUNC MOT_VEC motVecLerp(const MOT_VEC v1, const MOT_VEC v2, float t);

MOT_EXTERN_FUNC void motMtxMul(MOT_MTX* pRes, const MOT_MTX* pMtx1, const MOT_MTX* pMtx2);

MOT_EXTERN_FUNC void motMakeTransform(MOT_MTX* pMtx, const MOT_VEC tns, const MOT_QUAT rot, const MOT_VEC scl, E_MOT_XORD xord);
MOT_EXTERN_FUNC void motMakeTransformTR(MOT_MTX* pMtx, const MOT_VEC tns, const MOT_QUAT rot, E_MOT_XORD xord);
MOT_EXTERN_FUNC void motMakeTransformR(MOT_MTX* pMtx, const MOT_QUAT rot);
MOT_EXTERN_FUNC void motMakeTransformT(MOT_MTX* pMtx, const MOT_VEC tns);

MOT_EXTERN_FUNC MOT_QUAT motQuatFromRadians(float rx, float ry, float rz, E_MOT_RORD rord);
MOT_EXTERN_FUNC MOT_QUAT motQuatFromDegrees(float dx, float dy, float dz, E_MOT_RORD rord);
MOT_EXTERN_FUNC MOT_QUAT motQuatMul(const MOT_QUAT q1, const MOT_QUAT q2);
MOT_EXTERN_FUNC MOT_QUAT motQuatNormalize(const MOT_QUAT q);
MOT_EXTERN_FUNC MOT_QUAT motQuatExp(const MOT_VEC v);
MOT_EXTERN_FUNC MOT_QUAT motQuatSlerp(const MOT_QUAT q1, const MOT_QUAT q2, float t);
MOT_EXTERN_FUNC MOT_VEC motQuatToRadians(const MOT_QUAT q, E_MOT_RORD rord);
MOT_EXTERN_FUNC MOT_VEC motQuatToDegrees(const MOT_QUAT q, E_MOT_RORD rord);

MOT_EXTERN_FUNC int motClipHeaderCk(const MOT_CLIP* pClip);
MOT_EXTERN_FUNC int motNodeIdxCk(const MOT_CLIP* pClip, int nodeIdx);
MOT_EXTERN_FUNC int motFrameNoCk(const MOT_CLIP* pClip, int fno);
MOT_EXTERN_FUNC int motNodeTrackCk(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk);
MOT_EXTERN_FUNC int motFindNode(const MOT_CLIP* pClip, const char* pName);
MOT_EXTERN_FUNC float* motGetTrackData(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk);
MOT_EXTERN_FUNC void motGetChanData(const MOT_CLIP* pClip, int nodeIdx, E_MOT_TRK trk, int chIdx, float** ppData, int* pStride);
MOT_EXTERN_FUNC E_MOT_RORD motGetRotOrd(const MOT_CLIP* pClip, int nodeIdx);
MOT_EXTERN_FUNC E_MOT_XORD motGetXformOrd(const MOT_CLIP* pClip, int nodeIdx);
MOT_EXTERN_FUNC MOT_VEC motGetVec(const MOT_CLIP* pClip, int nodeIdx, int fno, E_MOT_TRK trk);
MOT_EXTERN_FUNC MOT_VEC motGetPos(const MOT_CLIP* pClip, int nodeIdx, int fno);
MOT_EXTERN_FUNC MOT_VEC motGetScl(const MOT_CLIP* pClip, int nodeIdx, int fno);
MOT_EXTERN_FUNC MOT_QUAT motGetQuat(const MOT_CLIP* pClip, int nodeIdx, int fno);
MOT_EXTERN_FUNC MOT_VEC motGetRadians(const MOT_CLIP* pClip, int nodeIdx, int fno);
MOT_EXTERN_FUNC MOT_VEC motGetDegrees(const MOT_CLIP* pClip, int nodeIdx, int fno);
MOT_EXTERN_FUNC MOT_QUAT motEvalQuat(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC MOT_QUAT motEvalQuatSlerp(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC MOT_VEC motEvalRadians(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC MOT_VEC motEvalDegrees(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC MOT_VEC motEvalPos(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC MOT_VEC motEvalScl(const MOT_CLIP* pClip, int nodeIdx, float frm);
MOT_EXTERN_FUNC void motEvalTransform(MOT_MTX* pMtx, const MOT_CLIP* pClip, int nodeIdx, float frm, const MOT_VEC* pDefTns);
