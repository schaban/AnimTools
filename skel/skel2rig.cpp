#include "crosscore.hpp"

static void dbgmsg(const char* pMsg) {
	::printf("%s", pMsg);
}

static void init_sys() {
	sxSysIfc sysIfc;
	nxCore::mem_zero(&sysIfc, sizeof(sysIfc));
	sysIfc.fn_dbgmsg = dbgmsg;
	nxSys::init(&sysIfc);
}

#define MAX_NODE_NAME_LEN 63

struct Node {
	char name[MAX_NODE_NAME_LEN + 1];
	char parentName[MAX_NODE_NAME_LEN + 1];
	xt_float3 pos;
	xt_float3 rot;
	xt_float3 rgb;

	bool is_skin() const {
		return nxCore::str_starts_with(name, "j_") ||
		       nxCore::str_starts_with(name, "s_") ||
		       nxCore::str_starts_with(name, "x_") ||
		       nxCore::str_starts_with(name, "f_");
	}
};

static Node s_nodes[256] = {};
static int s_numNodes = 0;

static float s_scale = 1.0f;

static void fmtU64(char* pBuf, size_t bsize, uint64_t val) {
	uint32_t* p = (uint32_t*)&val;
	uint32_t lo = p[0];
	uint32_t hi = p[1];
	XD_SPRINTF(XD_SPRINTF_BUF(pBuf, bsize), "%08X%08X", hi, lo);
}

class TokEcho : public cxLexer::TokenFunc {
public:
virtual bool operator()(const cxLexer::Token& tok) {
	char str64[32];
	if (tok.id == cxLexer::TokId::TOK_FLOAT) {
		::printf("float %.12f @ (%d, %d)\n", tok.val.f, tok.loc.line, tok.loc.column);
	} else if (tok.id == cxLexer::TokId::TOK_INT) {
		fmtU64(str64, sizeof(str64), tok.val.i);
		::printf("int 0x%s @ (%d, %d)\n", str64, tok.loc.line, tok.loc.column);
	} else if (tok.is_punctuation()) {
		::printf("punct %s @ (%d, %d)\n", tok.val.c, tok.loc.line, tok.loc.column);
	} else if (tok.id == cxLexer::TokId::TOK_QSTR) {
		::printf("\"%s\" @ (%d, %d)\n", (char*)tok.val.p, tok.loc.line, tok.loc.column);
	} else if (tok.id == cxLexer::TokId::TOK_SQSTR) {
		::printf("'%s' @ (%d, %d)\n", (char*)tok.val.p, tok.loc.line, tok.loc.column);
	} else if (tok.is_symbol()) {
		::printf("symbol %s @ (%d, %d)\n", (char*)tok.val.p, tok.loc.line, tok.loc.column);
	} else if (tok.is_keyword()) {
		::printf("keyword %s @ (%d, %d)\n", (char*)tok.val.p, tok.loc.line, tok.loc.column);
	} else {
		::printf("unknown @ (%d, %d)\n", tok.loc.line, tok.loc.column);
	}
	return true;
}
};

static void parse_skel_test(const char* pPath) {
	cxLexer lex;
	size_t textSize = 0;
	char* pText = (char*)nxCore::raw_bin_load(pPath, &textSize);
	if (pText) {
		nxCore::dbg_msg("lex <- %d bytes\n", textSize);
		lex.set_text(pText, textSize);
		TokEcho echo;
		lex.scan(echo);
		nxCore::bin_unload(pText);
	}
}

class TokSkel : public cxLexer::TokenFunc {
protected:
	int mState;
	int mStateCnt;
	int mNodeCnt;

public:
TokSkel() : mState(0), mStateCnt(0), mNodeCnt(0) {}

int get_num_nodes() const { return mNodeCnt; }

virtual bool operator()(const cxLexer::Token& tok) {
	if (mState < 0) return false;
	if (mNodeCnt >= XD_ARY_LEN(s_nodes)) {
		mState = -1;
		return false;
	}
	Node* pNode = &s_nodes[mNodeCnt];
	if (mState == 0) {
		// name
		if (tok.is_symbol()) {
			const char* pName = (const char*)tok.val.p;
			size_t nameLen = nxCore::str_len(pName);
			if (nameLen < MAX_NODE_NAME_LEN) {
				nxCore::mem_copy(pNode->name, pName, nameLen + 1);
				mState = 1;
			} else {
				mState = -1;
			}
		} else {
			mState = -1;
		}
	} else if (mState == 1) {
		// parent
		if (tok.id == cxLexer::TokId::TOK_DOT) {
			pNode->parentName[0] = '.';
			pNode->parentName[1] = 0;
			mState = 2;
			mStateCnt = 0;
		} else if (tok.is_symbol()) {
			const char* pName = (const char*)tok.val.p;
			size_t nameLen = nxCore::str_len(pName);
			if (nameLen < MAX_NODE_NAME_LEN) {
				nxCore::mem_copy(pNode->parentName, pName, nameLen + 1);
				mState = 2;
				mStateCnt = 0;
			} else {
				mState = -1;
			}
		} else {
			mState = -1;
		}
	} else {
		bool numFlg = false;
		float val = 0.0f;
		if (tok.id == cxLexer::TokId::TOK_FLOAT) {
			val = (float)tok.val.f;
			numFlg = true;
		} else if (tok.id == cxLexer::TokId::TOK_INT) {
			val = (float)tok.val.i;
			numFlg = true;
		}
		if (!numFlg) {
			mState = -1;
		} else {
			switch (mStateCnt) {
				case 0: pNode->pos.x = val; break;
				case 1: pNode->pos.y = val; break;
				case 2: pNode->pos.z = val; break;
				case 3: pNode->rot.x = val; break;
				case 4: pNode->rot.y = val; break;
				case 5: pNode->rot.z = val; break;
			}
			++mStateCnt;
			if (mStateCnt >= 6) {
				mState = 0;
				++mNodeCnt;
			}
		}
	}
	return true;
}
};

static void skel_info() {
	int n = s_numNodes;
	nxCore::dbg_msg("%d nodes\n", n);
	for (int i = 0; i < n; ++i) {
		nxCore::dbg_msg("%s %s %.3f %.3f %.3f %.3f %.3f %.3f\n",
			s_nodes[i].name, s_nodes[i].parentName,
			s_nodes[i].pos.x, s_nodes[i].pos.y, s_nodes[i].pos.z,
			s_nodes[i].rot.x, s_nodes[i].rot.y, s_nodes[i].rot.z
		);
	}
}

static float neg_val(float x) {
	if (x) return -x;
	return x;
}

static void mirror_jnts() {
	Node* pMirr = &s_nodes[s_numNodes];
	int n = s_numNodes;
	for (int i = 0; i < n; ++i) {
		Node* pNode = &s_nodes[i];
		if (nxCore::str_ends_with(pNode->name, "_L")) {
			size_t slen = nxCore::str_len(pNode->name);
			nxCore::mem_copy(pMirr->name, pNode->name, slen + 1);
			pMirr->name[slen - 1] = 'R';
			slen = nxCore::str_len(pNode->parentName);
			nxCore::mem_copy(pMirr->parentName, pNode->parentName, slen + 1);
			if (nxCore::str_ends_with(pNode->parentName, "_L")) {
				pMirr->parentName[slen - 1] = 'R';
			}
			pMirr->pos.x = neg_val(pNode->pos.x);
			pMirr->pos.y = pNode->pos.y;
			pMirr->pos.z = pNode->pos.z;
			pMirr->rot.x = pNode->rot.x;
			pMirr->rot.y = neg_val(pNode->rot.y);
			pMirr->rot.z = neg_val(pNode->rot.z);
			++pMirr;
			++s_numNodes;
		}
	}
}

static void colorize_jnts() {
	int n = s_numNodes;
	if (n <= 0) return;
	sxRNG rng;
	nxCore::rng_seed(&rng, 0xACDC);
	for (int i = 0; i < n; ++i) {
		Node* pNode = &s_nodes[i];
		pNode->rgb.fill(0.0f);
		if (pNode->is_skin()) {
			float* pRGB = (float*)&pNode->rgb;
			for (int j = 0; j < 3; ++j) {
				float base = nxCore::rng_f01(&rng);
				base = nxCalc::fit(base, 0.0f, 1.0f, 0.025f, 0.2f);
				float rel = nxCore::rng_f01(&rng);
				rel = nxCalc::fit(rel, 0.0f, 1.0f, 0.22f, 0.33f);
				pRGB[j] = base + rel;
			}
		}
	}
}

static void parse_skel(const char* pPath) {
	cxLexer lex;
	size_t textSize = 0;
	char* pText = (char*)nxCore::raw_bin_load(pPath, &textSize);
	if (pText) {
		lex.set_text(pText, textSize);
		TokSkel skelTok;
		lex.scan(skelTok);
		nxCore::bin_unload(pText);
		s_numNodes = skelTok.get_num_nodes();
		mirror_jnts();
		colorize_jnts();
		//skel_info();
	}
}

static void mk_rig_py(FILE* pOut = stdout) {
	int n = s_numNodes;
	if (n <= 0) return;
	if (!pOut) return;
	int nskin = 0;
	int nctl = 0;
	for (int i = 0; i < n; ++i) {
		Node* pNode = &s_nodes[i];
		bool skinFlg = pNode->is_skin();
		::fprintf(pOut, "nd_%s = hou.node(\"obj\").createNode(\"null\", \"%s\")\n", pNode->name, pNode->name);
		xt_float3 pos = pNode->pos;
		pos.scl(s_scale);
		::fprintf(pOut, "nd_%s.setParms({'tx':%.4f, 'ty':%.4f, 'tz':%.4f})\n", pNode->name, pos.x, pos.y, pos.z);
		xt_float3 rot = pNode->rot;
		::fprintf(pOut, "nd_%s.setParms({'rx':%.4f, 'ry':%.4f, 'rz':%.4f})\n", pNode->name, rot.x, rot.y, rot.z);
		float ctlSize = 0.01f;
		if (skinFlg) {
			::fprintf(pOut, "cr = nd_%s.createNode(\"cregion\", \"cregion\")\n", pNode->name);
			float crs = 0.0001f;
			::fprintf(pOut, "cr.setParms({'squashx':%.4f, 'squashy':%.4f, 'squashz':%.4f})\n", crs, crs, crs);
			::fprintf(pOut, "nd_%s.setUserData(\"nodeshape\", \"%s\")\n", pNode->name, "bone");
			++nskin;
		} else {
			::fprintf(pOut, "nd_%s.setParms({'controltype':%d, 'orientation':%d})\n",
				pNode->name, /*circles*/ 1, /*ZX*/ 2);
			::fprintf(pOut, "nd_%s.setUserData(\"nodeshape\", \"%s\")\n", pNode->name, "rect");
			ctlSize = 0.05f;
			++nctl;
		}
		xt_float3 rgb = pNode->rgb;
		::fprintf(pOut, "nd_%s.setParms({'dcolorr':%.2f,'dcolorg':%.2f,'dcolorb':%.2f,'geoscale':%.4f})\n",
			pNode->name, rgb.x, rgb.y, rgb.z, ctlSize);
	}
	for (int i = 0; i < n; ++i) {
		Node* pNode = &s_nodes[i];
		if (!nxCore::str_eq(pNode->parentName, ".")) {
			::fprintf(pOut, "nd_%s.setFirstInput(nd_%s)\n", pNode->name, pNode->parentName);
		}
	}
	::fprintf(pOut, "# %d (0x%X) skin, %d (0x%X) ctl\n", nskin, nskin, nctl, nctl);
}


int main(int argc, char* argv[]) {
	nxApp::init_params(argc, argv);
	init_sys();
	size_t narg = nxApp::get_args_count();
	s_scale = nxApp::get_float_opt("scl", 1.0f);
	const char* pSkelPath = "_skel_.txt";
	if (narg > 0) {
		pSkelPath = nxApp::get_arg(0);
	}
	parse_skel(pSkelPath);
	mk_rig_py();
	nxApp::reset();
	return 0;
}

