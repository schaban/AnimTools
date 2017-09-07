/*
 * Convert motion channels from Houdini .clip files into node-based binary format.
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

using System;
using System.IO;
using System.Threading;
using System.Globalization;
using System.Collections.Generic;


public enum TRK_KIND {
	POS = 0,
	ROT = 1,
	SCL = 2
}

public enum ROT_ORD {
	XYZ = 0,
	XZY = 1,
	YXZ = 2,
	YZX = 3,
	ZXY = 4,
	ZYX = 5
}

public enum XFORM_ORD {
	SRT = 0,
	STR = 1,
	RST = 2,
	RTS = 3,
	TSR = 4,
	TRS = 5
}

public enum DUMP_MODE {
	DEFAULT,
	LOGVECS,
	QUATS
}

public static class DEFS {
	public static int FIX_STR_SIZE = 0x40;
	public static int CLIP_HEADER_SIZE = 0x20 + DEFS.FIX_STR_SIZE;
	public static int NODE_INFO_SIZE = DEFS.FIX_STR_SIZE + 0x70;
	public static uint MOT_CLIP_ID = nUtl.FOURCC('M', 'C', 'L', 'P');
	public static uint MOT_LIB_ID = nUtl.FOURCC('M', 'L', 'I', 'B');
}

public struct QUAT {
	public float x, y, z, w;
		
	public void Identity() { Set(0.0f, 0.0f, 0.0f, 1.0f); }
		
	public void Set(float x, float y, float z, float w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}
		
	public void Cpy(ref QUAT other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
		this.w = other.w;
	}
	
	public float this[int i] {
		get {
			float val;
			switch (i) {
				case 0: val = x; break;
				case 1: val = y; break;
				case 2: val = z; break;
				case 3: val = w; break;
				default: val = Single.NaN; break;
			}
			return val;
		}
	}
		
	public void Mul(ref QUAT q) {
		float tx = w*q.x + x*q.w + y*q.z - z*q.y;
		float ty = w*q.y + y*q.w + z*q.x - x*q.z;
		float tz = w*q.z + z*q.w + x*q.y - y*q.x;
		float tw = w*q.w - x*q.x - y*q.y - z*q.z;
		Set(tx, ty, tz, tw);
	}
		
	public void SetRX(float radians) { float h = radians*0.5f; Set((float)Math.Sin(h), 0.0f, 0.0f, (float)Math.Cos(h)); }
	public void SetRY(float radians) { float h = radians*0.5f; Set(0.0f, (float)Math.Sin(h), 0.0f, (float)Math.Cos(h)); }
	public void SetRZ(float radians) { float h = radians*0.5f; Set(0.0f, 0.0f, (float)Math.Sin(h), (float)Math.Cos(h)); }
		
	public void SetDX(float degrees) { SetRX(nUtl.DegToRad(degrees)); }
	public void SetDY(float degrees) { SetRY(nUtl.DegToRad(degrees)); }
	public void SetDZ(float degrees) { SetRZ(nUtl.DegToRad(degrees)); }
		
	public void SetR(float rx, float ry, float rz, ROT_ORD ord) {
		var tbl = new byte[] {
			/* XYZ */ 0, 1, 2,
			/* XZY */ 0, 2, 1,
			/* YXZ */ 1, 0, 2,
			/* YZX */ 1, 2, 0,
			/* ZXY */ 2, 0, 1,
			/* ZYX */ 2, 1, 0
		};
		int idx = (int)ord;
		if (idx >= 6) {
			Identity();
		} else {
			int iq2 = tbl[idx];
			int iq1 = tbl[idx + 1];
			int iq0 = tbl[idx + 2];
			var rq = new QUAT[3];
			rq[0].SetRX(rx);
			rq[1].SetRY(ry);
			rq[2].SetRZ(rz);
			Cpy(ref rq[iq0]);
			Mul(ref rq[iq1]);
			Mul(ref rq[iq2]);
		}
	}
		
	public void SetD(float dx, float dy, float dz, ROT_ORD ord) {
		SetR(nUtl.DegToRad(dx), nUtl.DegToRad(dy), nUtl.DegToRad(dz), ord);
	}
	
	public float[] GetR(ROT_ORD ord) {
		var rXYZ = new float[] { 0, 0, 0 };
		int axisMask = 0;
		const float eps = 1.0e-6f;
		if (Math.Abs(x) < eps) axisMask |= 1;
		if (Math.Abs(y) < eps) axisMask |= 2;
		if (Math.Abs(z) < eps) axisMask |= 4;
		if (Math.Abs(w) < eps) axisMask |= 8;
		bool singleAxis = false;
		float qw = w;
		if (qw < -1.0f) qw = -1.0f;
		if (qw > 1.0f) qw = 1.0f;
		switch (axisMask) {
			case 6: /* 0110 -> X */
				rXYZ[0] = (float)Math.Acos(qw) * 2.0f;
				if (x < 0) rXYZ[0] = -rXYZ[0];
				rXYZ[0] = nUtl.LimitPI(rXYZ[0]);
				singleAxis = true;
				break;
			case 5: /* 0101 -> Y */
				rXYZ[1] = (float)Math.Acos(qw) * 2.0f;
				if (y < 0) rXYZ[1] = -rXYZ[1];
				rXYZ[1] = nUtl.LimitPI(rXYZ[1]);
				singleAxis = true;
				break;
			case 3: /* 0011 -> Z */
				rXYZ[2] = (float)Math.Acos(qw) * 2.0f;
				if (z < 0) rXYZ[2] = -rXYZ[2];
				rXYZ[2] = nUtl.LimitPI(rXYZ[2]);
				singleAxis = true;
				break;
			case 7: /* 0111 -> identity */
				singleAxis = true;
				break;
		}
		if (singleAxis) {
			return rXYZ;
		}

		var tbl = new byte[] {
			/* XYZ */ 0, 1, 2, 1,
			/* XZY */ 0, 2, 1, 0,
			/* YXZ */ 1, 0, 2, 0,
			/* YZX */ 1, 2, 0, 1,
			/* ZXY */ 2, 0, 1, 1,
			/* ZYX */ 2, 1, 0, 0
		};
		int tblIdx = ((int)ord) * 4;
		int i0 = tbl[tblIdx + 0];
		int i1 = tbl[tblIdx + 1];
		int i2 = tbl[tblIdx + 2];
		float sgn = tbl[tblIdx + 3] != 0 ? 1.0f : -1.0f;
		var m = new float[3, 3];
		m[0, 0] = 1.0f - 2.0f*y*y - 2.0f*z*z;
		m[0, 1] = 2.0f*x*y + 2.0f*w*z;
		m[0, 2] = 2.0f*x*z - 2.0f*w*y;
		m[1, 0] = 2.0f*x*y - 2.0f*w*z;
		m[1, 1] = 1.0f - 2.0f*x*x - 2.0f*z*z;
		m[1, 2] = 2.0f*y*z + 2.0f*w*x;
		m[2, 0] = 2.0f*x*z + 2.0f*w*y;
		m[2, 1] = 2.0f*y*z - 2.0f*w*x;
		m[2, 2] = 1.0f - 2.0f*x*x - 2.0f*y*y;

		var rm0 = new float[] { m[i0, i0], m[i0, i1], m[i0, i2] };
		var rm1 = new float[] { m[i1, i0], m[i1, i1], m[i1, i2] };
		var rm2 = new float[] { m[i2, i0], m[i2, i1], m[i2, i2] };

		rXYZ[i0] = (float)Math.Atan2(rm1[2], rm2[2]);
		rXYZ[i1] = (float)Math.Atan2(-rm0[2], Math.Sqrt(rm0[0]*rm0[0] + rm0[1]*rm0[1]));
		float s = (float)Math.Sin(rXYZ[i0]);
		float c = (float)Math.Cos(rXYZ[i0]);
		rXYZ[i2] = (float)Math.Atan2(s*rm2[0] - c*rm1[0], c*rm1[1] - s*rm2[1]);
		for (int i = 0; i < 3; ++i) {
			rXYZ[i] *= sgn;
		}
		for (int i = 0; i < 3; ++i) {
			rXYZ[i] = nUtl.LimitPI(rXYZ[i]);
		}

		return rXYZ;
	}
	
	public float[] GetD(ROT_ORD ord) {
		float[] r = GetR(ord);
		float s = nUtl.RadToDeg(1.0f);
		for (int i = 0; i < 3; ++i) {
			r[i] *= s;
		}
		return r;
	}
		
	public void GetLogVec(ref VEC v) {
		float cosh = Math.Min(Math.Max(w, -1.0f), 1.0f);
		float hang = (float)Math.Acos(cosh);
		float norm = (float)Math.Sqrt(x*x + y*y + z*z);
		float s = 0.0f;
		if (norm > 0.0f) {
			s = hang / norm;
		}
		v.x = x * s;
		v.y = y * s;
		v.z = z * s;
	}
	
	public void FromLogVec(VEC v) {
		float hang = v.Mag();
		float s = nUtl.Sinc(hang);
		x = v.x * s;
		y = v.y * s;
		z = v.z * s;
		w = (float)Math.Cos(hang);
		Nrm();
	}
	
	public void Nrm() {
		float s = (float)Math.Sqrt(Dot(ref this, ref this));
		if (s > 0.0f) {
			s = 1.0f / s;
		}
		x *= s;
		y *= s;
		z *= s;
		w *= s;
	}
	
	public void Neg() { x = -x; y = -y; z = -z; w = -w; }
	public void Conj() { x = -x; y = -y; z = -z; }
		
	public static float Dot(ref QUAT q0, ref QUAT q1) {
		return q0.x*q1.x + q0.y*q1.y + q0.z*q1.z + q0.w*q1.w;
	}
}


public struct VEC {
	public float x, y, z;
		
	public void Set(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public void Fill(float val) {
		x = val;
		y = val;
		z = val;
	}
		
	public void Cpy(ref VEC other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
	}
	
	public float this[int i] {
		get {
			float val;
			switch (i) {
				case 0: val = x; break;
				case 1: val = y; break;
				case 2: val = z; break;
				default: val = Single.NaN; break;
			}
			return val;
		}
	}
		
	public void Min(ref VEC v) {
		x = Math.Min(x, v.x);
		y = Math.Min(y, v.y);
		z = Math.Min(z, v.z);
	}
		
	public void Max(ref VEC v) {
		x = Math.Max(x, v.x);
		y = Math.Max(y, v.y);
		z = Math.Max(z, v.z);
	}
	
	public float Mag2() { return x*x + y*y + z*z; }
	public float Mag() { return (float)Math.Sqrt(Mag2()); }
	
	public void Read(BinaryReader br) {
		x = br.ReadSingle();
		y = br.ReadSingle();
		z = br.ReadSingle();
	}
		
	public void Write(BinaryWriter bw) {
		bw.Write(x);
		bw.Write(y);
		bw.Write(z);
	}
}

public class cStrList {
	public List<string> mStr;
	public List<int> mOrd;
	public List<uint> mHash;

	public cStrList() {
		Reset();
	}

	public void Reset() {
		mStr = new List<string>();
		mOrd = new List<int>();
		mHash = new List<uint>();
	}

	public int Count {
		get {
			return mStr.Count;
		}
	}

	protected int FindSlot(uint h, int cnt) {
		int org = 0;
		int end = cnt;
		do {
			int mid = (org + end) / 2;
			if (h < mHash[mid]) {
				end = mid;
			} else {
				org = mid;
			}
		} while (end - org >= 2);
		return org;
	}

	protected int FindSlot(uint h) {
		return FindSlot(h, Count);
	}

	protected void Swap(int i0, int i1) {
		string s = mStr[i0];
		int i = mOrd[i0];
		uint h = mHash[i0];
		mStr[i0] = mStr[i1];
		mOrd[i0] = mOrd[i1];
		mHash[i0] = mHash[i1];
		mStr[i1] = s;
		mOrd[i1] = i;
		mHash[i1] = h;
	}

	protected void Move(int dst, int src) {
		mStr[dst] = mStr[src];
		mOrd[dst] = mOrd[src];
		mHash[dst] = mHash[src];
	}

	protected void Append(string s, int i, uint h) {
		mStr.Add(s);
		mOrd.Add(i);
		mHash.Add(h);
	}

	protected void Store(int at, string s, int i, uint h) {
		mStr[at] = s;
		mOrd[at] = i;
		mHash[at] = h;
	}

	protected uint Hash(string s) {
		uint h = 2166136261U;
		if (s != null) unchecked {
				for (int i = 0; i < s.Length; ++i) {
					h *= 16777619U;
					h ^= (byte)s[i];
				}
			}
		return h;
	}

	public void Add(string str) {
		if (str == null) return;
		uint h = Hash(str);
		int ord = Count;
		Append(str, ord, h);
		if (ord > 0) {
			int dst = FindSlot(h, ord);
			if (dst == ord - 1) {
				if (h < mHash[dst]) {
					Swap(ord, ord - 1);
				}
			} else {
				if (h < mHash[dst]) {
					--dst;
				}
				for (int i = ord; --i >= dst + 1;) {
					Move(i + 1, i);
				}
				Store(dst + 1, str, ord, h);
			}
		}
	}

	public int FindIdx(string s) {
		int res = -1;
		if (s != null && Count > 0) {
			uint h = Hash(s);
			int idx = FindSlot(h);
			if (h == mHash[idx]) {
				int nc = 1;
				for (int i = idx; --i >= 0;) {
					if (h != mHash[i]) break;
					--idx;
					++nc;
				}
				for (int i = 0; i < nc; ++i) {
					if (mStr[idx + i] == s) return idx + i;
				}
			}
		}
		return res;
	}

	public int this[string s] => FindIdx(s);

	public int FindOrd(string s) {
		int idx = FindIdx(s);
		if (idx >= 0) {
			return mOrd[idx];
		}
		return -1;
	}
}

public class cEvalInfo {
	public struct MAP {
		public int node;
		public TRK_KIND kind;
		public int chan;

		public void Write(BinaryWriter bw) {
			bw.Write((ushort)node);
			bw.Write((byte)kind);
			bw.Write((byte)chan);
		}

		public override string ToString() => $"node{node:D2} {kind}[{chan}]";
	}

	public int[] ntrk;
	public int[] nchn;
	public int[] ncrv;
	public MAP[] map; // [ curves<pos,rot,scl>[ncrv], consts<pos,rot,scl>[nchn - ncrv] ]

	public cEvalInfo() {
		ntrk = new int[3];
		nchn = new int[3];
		ncrv = new int[3];
	}

	public int NumPosTracks => ntrk[(int)TRK_KIND.POS];
	public int NumRotTracks => ntrk[(int)TRK_KIND.ROT];
	public int NumSclTracks => ntrk[(int)TRK_KIND.SCL];

	public int TracksTotal {
		get {
			int cnt = 0;
			foreach (int n in ntrk) { cnt += n; }
			return cnt;
		}
	}

	public void AllocMap() {
		int n = 0;
		foreach (int nc in nchn) n += nc;
		map = new MAP[n];
	}

	public void Write(BinaryWriter bw) {
		foreach (int n in ntrk) bw.Write(n);
		foreach (int n in nchn) bw.Write(n);
		foreach (int n in ncrv) bw.Write(n);
		foreach (MAP m in map) m.Write(bw);
	}
}


public static class nUtl {

	public static float RadToDeg(float rad) {
		return ((rad * 180.0f) / (float)Math.PI);
	}

	public static float DegToRad(float deg) {
		return ((deg * (float)Math.PI) / 180.0f);
	}

	public static float LimitPI(float rad) {
		const float pi = (float)Math.PI;
		rad %= pi * 2;
		if (Math.Abs(rad) > pi) {
			if (rad < 0.0f) {
				rad = pi * 2 + rad;
			} else {
				rad = rad - pi * 2;
			}
		}
		return rad;
	}

	public static float Sinc(float x) {
		if (Math.Abs(x) < 1.0e-4f) return 1.0f;
		return (float)Math.Sin(x) / x;
	}

	public static uint FOURCC(char c1, char c2, char c3, char c4) {
		return (uint)((((byte)c4) << 24) | (((byte)c3) << 16) | (((byte)c2) << 8) | ((byte)c1));
	}

	public static void WriteFixStr(BinaryWriter bw, string s) {
		int slen = Math.Min(DEFS.FIX_STR_SIZE - 2, s.Length);
		int pad = DEFS.FIX_STR_SIZE - 1 - slen;
		bw.Write((byte)s.Length);
		for (int i = 0; i < slen; ++i) {
			bw.Write((byte)s[i]);
		}
		for (int i = 0; i < pad; ++i) {
			bw.Write((byte)0);
		}
	}

	public static void Patch32(BinaryWriter bw, long offs, int val) {
		long oldPos = bw.BaseStream.Position;
		bw.BaseStream.Seek(offs, SeekOrigin.Begin);
		bw.Write(val);
		bw.BaseStream.Seek(oldPos, SeekOrigin.Begin);
	}

	public static void PatchWithCurrPos32(BinaryWriter bw, long offs) {
		Patch32(bw, offs, (int)bw.BaseStream.Position);
	}

	public static int ParseInt(string s) {
		try {
			NumberStyles nstyles = NumberStyles.AllowExponent | NumberStyles.AllowLeadingSign;
			long i = Int64.Parse(s, nstyles);
			return (int)i;
		} catch {
			return -1;
		}
	}

	public static double ParseF64(string s) {
		double d;
		try {
			NumberStyles nstyles = NumberStyles.AllowExponent | NumberStyles.AllowDecimalPoint | NumberStyles.AllowLeadingSign;
			d = Double.Parse(s, nstyles, NumberFormatInfo.InvariantInfo);
		} catch {
			d = Double.NaN;
		}
		return d;
	}

	public static float ParseF32(string s) {
		return (float)ParseF64(s);
	}

	public static string ReadStr(BinaryReader br) {
		string s = "";
		while (true) {
			char c = (char)br.ReadByte();
			if (c == 0) break;
			s += c;
		}
		return s;
	}

	public static string ReadFixStr(BinaryReader br) {
		int len = br.ReadByte();
		string s = "";
		for (int i = 0; i < len; ++i) {
			s += (char)br.ReadByte();
		}
		return s;
	}
}

public class cArgs {
	protected List<string> mArgs;
	protected Dictionary<string, string> mOpts;

	public cArgs() {
		mArgs = new List<string>();
		mOpts = new Dictionary<string, string>();
	}

	protected static bool IsOption(string arg) {
		return arg.StartsWith("-") || arg.StartsWith("/");
	}

	public void Parse(string[] args) {
		if (args == null || args.Length == 0) return;
		foreach (string arg in args) {
			if (IsOption(arg)) {
				int sep = arg.IndexOf(":");
				if (sep == -1) {
					mOpts.Add(arg.Substring(1), "");
				} else {
					mOpts.Add(arg.Substring(1, sep - 1), arg.Substring(sep + 1));
				}
			} else {
				mArgs.Add(arg);
			}
		}
	}

	public int ArgNum { get { return mArgs.Count; } 	}

	public string GetArg(int i, string def) {
		if (i < mArgs.Count) {
			return mArgs[i];
		}
		return def;
	}

	public string GetArg(int i) {
		return GetArg(i, null);
	}

	public bool HasOption(string name) {
		return mOpts.ContainsKey(name);
	}

	public string GetOpt(string name, string def) {
		if (HasOption(name)) {
			return mOpts[name];
		}
		return def;
	}

	public string GetOpt(string name) {
		return GetOpt(name, "");
	}
}


public class cHouClip {
	
	public struct QCHANS {
		public int ix, iy, iz;
		
		public int Read(string[] toks, int idx) {
			ix = nUtl.ParseInt(toks[idx++]);
			iy = nUtl.ParseInt(toks[idx++]);
			iz = nUtl.ParseInt(toks[idx++]);
			return idx;
		}
	}
	
	public class cTrack {
		public cHouClip mClip;
		public string mName;
		public float[] mData;
		public float mMinVal;
		public float mMaxVal;
		
		public cTrack(cHouClip clp) {
			mClip = clp;
		}
		
		public bool IsConst {
			get {
				return mMinVal == mMaxVal;
			}
		}
		
		public string ShortName {
			get {
				string name = null;
				if (mName != null) {
					name = mName;
					if (name.Contains(":") || name.Contains("/")) {
						int i = name.LastIndexOf('/');
						if (i >= 0) {
							name = name.Substring(i+1);
						}
						i = name.LastIndexOf(':');
						if (i >= 0) {
							name = name.Substring(0, i);
						}
					}
				}
				return name;
			}
		}
		
		public string ChannelName {
			get {
				string name = "<none>";
				int i = mName.LastIndexOf(':');
				if (i >= 0) {
					name = mName.Substring(i + 1);
				}
				return name;
			}
		}
		
		public float GetVal(int frm) {
			float val = 0.0f;
			if (mData != null && frm >= 0 && frm < mData.Length) {
				val = mData[frm];
			}
			return val;
		}
		
		protected int ReadData(string[] toks, int idx) {
			int n = mClip.mFramesNum;
			mData = new float[n];
			for (int i = 0; i < n; ++i) {
				float val = nUtl.ParseF32(toks[idx++]);
				mData[i] = val;
				if (i == 0) {
					mMinVal = val;
					mMaxVal = val;
				} else {
					mMinVal = Math.Min(mMinVal, val);
					mMaxVal = Math.Max(mMaxVal, val);
				}
			}
			return idx;
		}
		
		public int Read(string[] toks, int idx) {
			int i = idx;
			if (toks[i++] != @"{") {
				throw new Exception(@"! trk {");
			}
			while (true) {
				string t = toks[i++];
				if (t == @"}") {
					break;
				}
				if (toks[i++] != "=") {
					throw new Exception("! =");
				}
				switch (t) {
					case "name":
						mName = toks[i++];
						break;
					case "data":
						i = ReadData(toks, i);
						break;
					case "lefttype":
						++i;
						break;
					case "righttype":
						++i;
						break;
					case "default":
						++i;
						break;
					default:
						break;
				}
			}
			return i;
		}
		
		public override string ToString() {
			return String.Format("{0}: {1}..{2}", mName, mMinVal, mMaxVal);
		}

	}

	public 	string mPath = "<unknown>";
	public 	string mName = "<unknown>";
	public float mRate;
	public int mStart;
	public int mFramesNum;
	public int mTracksNum;
	public cTrack[] mTracks;
	public string mQuatOrder = "xyz";
	public int mQuatNum;
	public QCHANS[] mQChans;
	
	public Dictionary<string, int> mChanDict;
	
	public int FindTrackIdx(string nodeName, string chName) {
		int idx = -1;
		if (mChanDict != null) {
			string cpath = nodeName + ":" + chName;
			if (mChanDict.ContainsKey(cpath)) {
				idx = mChanDict[cpath];
			}
		}
		return idx;
	}
	
	public cTrack FindTrack(string nodeName, string chName) {
		cTrack trk = null;
		int idx = FindTrackIdx(nodeName, chName);
		if (idx >= 0) {
			trk = mTracks[idx];
		}
		return trk;
	}
	
	protected int ReadTracks(string[] toks, int idx) {
		int n = mTracksNum;
		mTracks = new cTrack[n];
		for (int i = 0; i < n; ++i) {
			mTracks[i] = new cTrack(this);
			idx = mTracks[i].Read(toks, idx);
		}
		mChanDict = new Dictionary<string, int>();
		for (int i = 0; i < n; ++i) {
			cTrack trk = mTracks[i];
			string cpath = trk.ShortName + ":" + trk.ChannelName;
			mChanDict[cpath] = i;
		}
		return idx;
	}
	
	protected int ReadQChans(string[] toks, int idx) {
		int n = mQuatNum;
		mQChans = new QCHANS[n];
		for (int i = 0; i < n; ++i) {
			idx = mQChans[i].Read(toks, idx);
		}
		return idx;
	}
	
	public void Read(StreamReader sr) {
		string text = sr.ReadToEnd();
		string[] toks = text.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
		int n = toks.Length;
		int i = 0;
		if (toks[i++] != @"{") {
			throw new Exception(@"! leading {");
		}
		while (i < n) {
			string t = toks[i++];
			if (t == @"}") {
				break;
			}
			if (toks[i++] != "=") {
				throw new Exception("! =");
			}
			string s = toks[i++];
			switch (t) {
				case "rate":
					mRate = nUtl.ParseF32(s);
					break;
				case "start":
					mStart = nUtl.ParseInt(s) + 1;
					break;
				case "tracklength":
					mFramesNum = nUtl.ParseInt(s);
					break;
				case "tracks":
					mTracksNum = nUtl.ParseInt(s);
					i = ReadTracks(toks, i);
					break;
				case "quaternions":
					mQuatOrder = s;
					s = toks[i++];
					mQuatNum = nUtl.ParseInt(s);
					i = ReadQChans(toks, i);
					break;
				default:
					break;
			}
		}
	}
	
	public void Load(string fpath) {
		mPath = fpath;
		mName = Path.GetFileNameWithoutExtension(fpath);
		StreamReader sr = File.OpenText(fpath);
		Read(sr);
		sr.Close();
	}
}


public class cMotClipWriter {

	public class cTrack {
		public cNode mNode;
		public TRK_KIND mKind;
		public cHouClip.cTrack mSrcX;
		public cHouClip.cTrack mSrcY;
		public cHouClip.cTrack mSrcZ;
		public VEC[] mData;
		public VEC mMin;
		public VEC mMax;
		public byte mSrcMask;
		public byte mDataMask;
		public long mInfoTop;
		public long mDataTop;

		public cTrack(cNode node, TRK_KIND kind) {
			mNode = node;
			mKind = kind;
			string ks = "" + "trs"[(int)mKind];
			cHouClip hclip = mNode.mClip.mSrc;
			mSrcX = hclip.FindTrack(mNode.mName, ks + "x");
			mSrcY = hclip.FindTrack(mNode.mName, ks + "y");
			mSrcZ = hclip.FindTrack(mNode.mName, ks + "z");
			mSrcMask = 0;
			mDataMask = 0;
			if (mSrcX != null || mSrcY != null || mSrcZ != null) {
				int nfrm = hclip.mFramesNum;
				mData = new VEC[nfrm];
				for (int i = 0; i < nfrm; ++i) {
					mData[i].x = GetRawX(i);
					mData[i].y = GetRawY(i);
					mData[i].z = GetRawZ(i);
				}
				if (mKind == TRK_KIND.ROT) {
					var tq = new QUAT[nfrm];
					for (int i = 0; i < nfrm; ++i) {
						tq[i].SetD(mData[i].x, mData[i].y, mData[i].z, mNode.mRotOrd);
					}
					var flip = new bool[nfrm];
					bool flg = false;
					for (int i = 0; i < nfrm; ++i) {
						if (i > 0) {
							flg ^= QUAT.Dot(ref tq[i], ref tq[i-1]) < 0.0f;
						}
						flip[i] = flg;
					}
					for (int i = 0; i < nfrm; ++i) {
						if (flip[i]) {
							tq[i].Neg();
						}
					}
					for (int i = 0; i < nfrm; ++i) {
						tq[i].GetLogVec(ref mData[i]);
					}
				}
				for (int i = 0; i < nfrm; ++i) {
					if (i == 0) {
						mMin.Cpy(ref mData[i]);
						mMax.Cpy(ref mData[i]);
					} else {
						mMin.Min(ref mData[i]);
						mMax.Max(ref mData[i]);
					}
				}
				float xs = mMax.x - mMin.x;
				float ys = mMax.y - mMin.y;
				float zs = mMax.z - mMin.z;
				if (xs != 0.0f) mDataMask |= 1;
				if (ys != 0.0f) mDataMask |= 2;
				if (zs != 0.0f) mDataMask |= 4;
				if (mSrcX != null) mSrcMask |= 1;
				if (mSrcY != null) mSrcMask |= 2;
				if (mSrcZ != null) mSrcMask |= 4;
			}
		}
		
		public float GetRawX(int frm) { return (mSrcX != null) ? mSrcX.GetVal(frm) : 0.0f; }
		public float GetRawY(int frm) { return (mSrcY != null) ? mSrcY.GetVal(frm) : 0.0f; }
		public float GetRawZ(int frm) { return (mSrcZ != null) ? mSrcZ.GetVal(frm) : 0.0f; }

		public int Stride {
			get {
				int s = 0;
				for (int i = 0; i < 3; ++i) {
					if ((mDataMask & (1 << i)) != 0) {
						++s;
					}
				}
				return s;
			}
		}

		public int NumChannels {
			get {
				int n = 0;
				for (int i = 0; i < 3; ++i) {
					if ((mSrcMask & (1 << i)) != 0) {
						++n;
					}
				}
				return n;
			}
		}

		public int NumCurves {
			get {
				int n = 0;
				for (int i = 0; i < 3; ++i) {
					if ((mDataMask & (1<<i)) != 0) {
						++n;
					}
				}
				return n;
			}
		}

		public int NumConts => NumChannels - NumCurves;


		public void WriteInfo(BinaryWriter bw) {
			mInfoTop = bw.BaseStream.Position - mNode.mClip.mFileTop;
			mMin.Write(bw); // +00
			mMax.Write(bw); // +0C
			bw.Write(mSrcMask); // +18
			bw.Write(mDataMask); // +19
			bw.Write((byte)Stride); // +1A
			bw.Write((byte)0); // +1B
			bw.Write((int)0); // +1C
		}
		
		public void WriteData(BinaryWriter bw) {
			if (mData != null && mDataMask != 0) {
				mDataTop = bw.BaseStream.Position - mNode.mClip.mFileTop;
				int n = mData.Length;
				for (int i = 0; i < n; ++i) {
					if ((mDataMask & 1) != 0) {
						bw.Write(mData[i].x);
					}
					if ((mDataMask & 2) != 0) {
						bw.Write(mData[i].y);
					}
					if ((mDataMask & 4) != 0) {
						bw.Write(mData[i].z);
					}
				}
			}
		}
	}

	public class cNode {
		public cMotClipWriter mClip;
		public string mName;
		public cTrack mPosTrk;
		public cTrack mRotTrk;
		public cTrack mSclTrk;
		public XFORM_ORD mXformOrd;
		public ROT_ORD mRotOrd;

		public cNode(cMotClipWriter clip, string name) {
			mClip = clip;
			mName = name;
			mXformOrd = XFORM_ORD.SRT;
			cHouClip.cTrack xOrdTrk = mClip.mSrc.FindTrack(mName, "xOrd");
			if (xOrdTrk != null) {
				if (!xOrdTrk.IsConst) {
					Console.Error.WriteLine("Node {0}: animated xOrd.");
				}
				mXformOrd = (XFORM_ORD)xOrdTrk.GetVal(0);
			}
			mRotOrd = ROT_ORD.XYZ;
			cHouClip.cTrack rOrdTrk = mClip.mSrc.FindTrack(mName, "rOrd");
			if (rOrdTrk != null) {
				if (!rOrdTrk.IsConst) {
					Console.Error.WriteLine("Node {0}: animated rOrd.");
				}
				mRotOrd = (ROT_ORD)rOrdTrk.GetVal(0);
			}
			mPosTrk = new cTrack(this, TRK_KIND.POS);
			mRotTrk = new cTrack(this, TRK_KIND.ROT);
			mSclTrk = new cTrack(this, TRK_KIND.SCL);
		}

		
		public void WriteInfo(BinaryWriter bw) {
			nUtl.WriteFixStr(bw, mName);
			bw.Write((uint)0); // +40 -> pos
			bw.Write((uint)0); // +44 -> rot
			bw.Write((uint)0); // +48 -> scl
			bw.Write((byte)mXformOrd); // +4C
			bw.Write((byte)mRotOrd); // +4D
			bw.Write((byte)0); // +4E
			bw.Write((byte)0); // +4F
			mPosTrk.WriteInfo(bw);
			mRotTrk.WriteInfo(bw);
			mSclTrk.WriteInfo(bw);
		}
		
		public void WriteData(BinaryWriter bw, long patchTop) {
			if (mPosTrk.mDataMask != 0) {
				mClip.PatchCur(patchTop);
				mPosTrk.WriteData(bw);
			}
			if (mRotTrk.mDataMask != 0) {
				mClip.PatchCur(patchTop + 4);
				mRotTrk.WriteData(bw);
			}
			if (mSclTrk.mDataMask != 0) {
				mClip.PatchCur(patchTop + 8);
				mSclTrk.WriteData(bw);
			}
		}

		public cTrack GetTrack(TRK_KIND kind) {
			cTrack trk = null;
			switch (kind) {
				case TRK_KIND.POS: trk = mPosTrk; break;
				case TRK_KIND.ROT: trk = mRotTrk; break;
				case TRK_KIND.SCL: trk = mSclTrk; break;
			}
			return trk;
		}
		
		public override string ToString() {
			return String.Format("{0}", mName);
		}
	}

	public struct SEQ {
		public int node;
		public int chan;
		public int offs;
		public int stride;

		public void Write(BinaryWriter bw) {
			bw.Write(offs);
			bw.Write((ushort)node);
			bw.Write((byte)chan);
			bw.Write((byte)stride);
		}
	}

	public cHouClip mSrc;
	public cStrList mNodeNamesLst;
	public List<string> mNodeNames;
	public cNode[] mNodes;
	public cEvalInfo mEvalInfo;
	public SEQ[] mSeq;
	public long mFileTop;

	protected BinaryWriter mBW;

	public cMotClipWriter(cHouClip src) {
		mSrc = src;
		GetNodeNames();
		int n = mNodeNames.Count;
		mNodes = new cNode[n];
		for (int i = 0; i < n; ++i) {
			mNodes[i] = new cNode(this, mNodeNames[i]);
		}
		mEvalInfo = new cEvalInfo();
		for (int i = 0; i < n; ++i) {
			if (mNodes[i].mPosTrk.mSrcMask != 0) {
				++mEvalInfo.ntrk[(int)TRK_KIND.POS];
			}
			if (mNodes[i].mRotTrk.mSrcMask != 0) {
				++mEvalInfo.ntrk[(int)TRK_KIND.ROT];
			}
			if (mNodes[i].mSclTrk.mSrcMask != 0) {
				++mEvalInfo.ntrk[(int)TRK_KIND.SCL];
			}

			mEvalInfo.nchn[(int)TRK_KIND.POS] += mNodes[i].mPosTrk.NumChannels;
			mEvalInfo.nchn[(int)TRK_KIND.ROT] += mNodes[i].mRotTrk.NumChannels;
			mEvalInfo.nchn[(int)TRK_KIND.SCL] += mNodes[i].mSclTrk.NumChannels;

			mEvalInfo.ncrv[(int)TRK_KIND.POS] += mNodes[i].mPosTrk.NumCurves;
			mEvalInfo.ncrv[(int)TRK_KIND.ROT] += mNodes[i].mRotTrk.NumCurves;
			mEvalInfo.ncrv[(int)TRK_KIND.SCL] += mNodes[i].mSclTrk.NumCurves;
		}
		mEvalInfo.AllocMap();
		int mapIdx = 0;
		var tkinds = new TRK_KIND[] { TRK_KIND.POS, TRK_KIND.ROT, TRK_KIND.SCL };
		for (int bit = 0; bit < 2; ++bit) {
			foreach (var kind in tkinds) {
				for (int inod = 0; inod < n; ++inod) {
					cTrack trk = mNodes[inod].GetTrack(kind);
					for (int ichn = 0; ichn < 3; ++ichn) {
						bool dataFlg = ((trk.mDataMask >> ichn) & 1) != bit;
						bool srcFlg = (trk.mSrcMask & (1 << ichn)) != 0;
						if (srcFlg && dataFlg) {
							mEvalInfo.map[mapIdx].node = inod;
							mEvalInfo.map[mapIdx].chan = ichn;
							mEvalInfo.map[mapIdx].kind = kind;
							++mapIdx;
						}
					}
				}
			}
		}
		mSeq = new SEQ[mEvalInfo.TracksTotal * 3];
		int iseq = 0;
		foreach (var kind in tkinds) {
			for (int inod = 0; inod < n; ++inod) {
				cTrack trk = mNodes[inod].GetTrack(kind);
				if (trk.mSrcMask != 0) {
					for (int ichn = 0; ichn < 3; ++ichn) {
						mSeq[iseq].node = inod;
						mSeq[iseq].chan = ichn;
						bool srcFlg = (trk.mSrcMask & (1 << ichn)) != 0;
						bool dataFlg = ((trk.mDataMask >> ichn) & 1) != 0;
						if (srcFlg) {
							mSeq[iseq].offs = 1;
							if (dataFlg) {
								mSeq[iseq].stride = trk.Stride;
							} else {
								mSeq[iseq].stride = 0;
							}
						} else {
							mSeq[iseq].offs = 0;
							mSeq[iseq].stride = 0;
						}
						++iseq;
					}
				}
			}
		}
	}
	
	public int FrameCount {
		get {
			int n = 0;
			if (mSrc != null) {
				n = mSrc.mFramesNum;
			}
			return n;
		}
	}
	
	public int NumNodes {
		get {
			int n = 0;
			if (mNodes != null) {
				n = mNodes.Length;
			}
			return n;
		}
	}
	
	public float FPS {
		get {
			float rate = 30.0f;
			if (mSrc != null) {
				rate = mSrc.mRate;
			}
			return rate;
		}
	}

	protected void GetNodeNames() {
		var names = new List<string>();
		int ntrk = mSrc.mTracksNum;
		for (int i = 0; i < ntrk; ++i) {
			cHouClip.cTrack strk = mSrc.mTracks[i];
			string name = strk.ShortName;
			if (!names.Contains(name)) {
				names.Add(name);
			}
		}
		mNodeNamesLst = new cStrList();
		foreach (string name in names) {
			mNodeNamesLst.Add(name);
		}
		mNodeNames = mNodeNamesLst.mStr;
	}

	public void PatchCur(long offs) {
		if (mBW != null) {
			int curOffs = (int)(mBW.BaseStream.Position - mFileTop);
			nUtl.Patch32(mBW, offs, curOffs);
		}
	}

	protected void WriteSeq() {
		BinaryWriter bw = mBW;
		if (bw == null) return;
		int n = NumNodes;
		var tkinds = new TRK_KIND[] { TRK_KIND.POS, TRK_KIND.ROT, TRK_KIND.SCL };
		int iseq = 0;
		foreach (var kind in tkinds) {
			for (int inod = 0; inod < n; ++inod) {
				cTrack trk = mNodes[inod].GetTrack(kind);
				if (trk.mSrcMask != 0) {
					for (int ichn = 0; ichn < 3; ++ichn) {
						if (mSeq[iseq].offs != 0) {
							int offs;
							if (mSeq[iseq].stride != 0) {
								offs = (int)trk.mDataTop;
								for (int k = 0; k < ichn; ++k) {
									if ((trk.mDataMask & (1<<k)) != 0) {
										offs += 4;
									}
								}
							} else {
								offs = (int)trk.mInfoTop + (ichn*4); /* -> vmin[ichn] */
							}
							mSeq[iseq].offs = offs;
						}
						++iseq;
					}
				}
			}
		}
		int nseq = mSeq.Length;
		if (iseq != nseq) {
			throw new Exception("WriteSeq");
		}
		for (int i = 0; i < nseq; ++i) {
			mSeq[i].Write(bw);
		}
	}

	public void Write(BinaryWriter bw) {
		mBW = bw;
		mFileTop = bw.BaseStream.Position;
		int n = NumNodes;
		/* +00 */ bw.Write(DEFS.MOT_CLIP_ID);
		/* +04 */ bw.Write((uint)0); // size
		/* +08 */ bw.Write(FPS);
		/* +0C */ bw.Write(FrameCount);
		/* +10 */ bw.Write(n);
		/* +14 */ bw.Write((uint)0); // -> hash
		/* +18 */ bw.Write((uint)0); // -> eval
		/* +1C */ bw.Write((uint)0); // -> seq
		nUtl.WriteFixStr(bw, mSrc.mName);
		long[] nodeTop = new long[n];
		for (int i = 0; i < n; ++i) {
			cNode node = mNodes[i];
			nodeTop[i] = bw.BaseStream.Position;
			node.WriteInfo(bw);
		}
		PatchCur(0x14); // <- hash
		for (int i = 0; i < n; ++i) {
			bw.Write(mNodeNamesLst.mHash[i]);
		}
		for (int i = 0; i < n; ++i) {
			cNode node = mNodes[i];
			node.WriteData(bw, nodeTop[i] + 0x40);
		}
		PatchCur(0x18); // <- eval
		mEvalInfo.Write(bw);
		PatchCur(0x1C); // <- seq
		WriteSeq();
		PatchCur(0x4); // size
		mBW = null;
	}

	public void Save(string fpath) {
		var ofs = new FileStream(fpath, FileMode.Create);
		var bw = new BinaryWriter(ofs);
		Write(bw);
		bw.Close();
	}
}


public class cMotClipReader {

	public class cTrack {
		public cNode mNode;
		public VEC mMin;
		public VEC mMax;
		public byte mSrcMask;
		public byte mDataMask;
		public VEC[] mData;
		
		public cTrack(cNode node) {
			mNode = node;
		}
		
		public int NumSrcChannels {
			get {
				int n = 0;
				if ((mSrcMask & 1) != 0) ++n;
				if ((mSrcMask & 2) != 0) ++n;
				if ((mSrcMask & 4) != 0) ++n;
				return n;
			}
		}
		
		public void ReadInfo(BinaryReader br) {
			mMin.Read(br);
			mMax.Read(br);
			mSrcMask = br.ReadByte();
			mDataMask = br.ReadByte();
			br.ReadByte();
			br.ReadByte();
			br.ReadInt32();
		}
		
		public void ReadData(BinaryReader br) {
			int n = mNode.mClip.mFramesNum;
			mData = new VEC[n];
			for (int i = 0; i < n; ++i) {
				mData[i].Cpy(ref mMin);
				if ((mDataMask & 1) != 0) {
					mData[i].x = br.ReadSingle();
				}
				if ((mDataMask & 2) != 0) {
					mData[i].y = br.ReadSingle();
				}
				if ((mDataMask & 4) != 0) {
					mData[i].z = br.ReadSingle();
				}
			}
		}
	}

	public class cNode {
		public cMotClipReader mClip;
		public long mFileTop;
		public string mName;
		public uint mOffsPos;
		public uint mOffsRot;
		public uint mOffsScl;
		public XFORM_ORD mXformOrd;
		public ROT_ORD mRotOrd;
		public cTrack mPosTrk;
		public cTrack mRotTrk;
		public cTrack mSclTrk;

		public cNode(cMotClipReader clip) {
			mClip = clip;
		}
		
		public void ReadInfo(BinaryReader br) {
			mFileTop = br.BaseStream.Position;
			mName = nUtl.ReadFixStr(br);
			br.BaseStream.Seek(mFileTop + DEFS.FIX_STR_SIZE, SeekOrigin.Begin);
			mOffsPos = br.ReadUInt32();
			mOffsRot = br.ReadUInt32();
			mOffsScl = br.ReadUInt32();
			mXformOrd = (XFORM_ORD)br.ReadByte();
			mRotOrd = (ROT_ORD)br.ReadByte();
			br.ReadByte();
			br.ReadByte();
			mPosTrk = new cTrack(this);
			mPosTrk.ReadInfo(br);
			mRotTrk = new cTrack(this);
			mRotTrk.ReadInfo(br);
			mSclTrk = new cTrack(this);
			mSclTrk.ReadInfo(br);
		}
		
		public void ReadData(BinaryReader br) {
			long top = mClip.mFileTop;
			if (mOffsPos != 0) {
				br.BaseStream.Seek(top + mOffsPos, SeekOrigin.Begin);
				mPosTrk.ReadData(br);
			}
			if (mOffsRot != 0) {
				br.BaseStream.Seek(top + mOffsRot, SeekOrigin.Begin);
				mRotTrk.ReadData(br);
			}
			if (mOffsScl != 0) {
				br.BaseStream.Seek(top + mOffsScl, SeekOrigin.Begin);
				mSclTrk.ReadData(br);
			}
		}
		
		public VEC GetLogVec(int fno) {
			VEC v = new VEC();
			v.Fill(0.0f);
			if (mClip != null && mRotTrk != null) {
				if (mRotTrk.mData != null && mClip.CkFrameNo(fno)) {
					v = mRotTrk.mData[fno];
				} else {
					v.Cpy(ref mRotTrk.mMin);
				}
			}
			return v;
		}
		
		public QUAT GetQuat(int fno) {
			VEC v = GetLogVec(fno);
			QUAT q = new QUAT();
			q.FromLogVec(v);
			return q;
		}
		
		public float[] GetRadians(int fno) {
			QUAT q = GetQuat(fno);
			return q.GetR(mRotOrd);
		}
		
		public float[] GetDegrees(int fno) {
			QUAT q = GetQuat(fno);
			return q.GetD(mRotOrd);
		}
		
		public VEC GetPos(int fno) {
			VEC v = new VEC();
			v.Fill(0.0f);
			if (mClip != null && mPosTrk != null) {
				if (mPosTrk.mData != null && mClip.CkFrameNo(fno)) {
					v = mPosTrk.mData[fno];
				} else {
					v.Cpy(ref mPosTrk.mMin);
				}
			}
			return v;
		}
		
		public VEC GetScl(int fno) {
			VEC v = new VEC();
			v.Fill(1.0f);
			if (mClip != null && mPosTrk != null) {
				if (mPosTrk.mData != null && mClip.CkFrameNo(fno)) {
					v = mPosTrk.mData[fno];
				} else {
					v.Cpy(ref mPosTrk.mMin);
				}
			}
			return v;
		}
		
		public override string ToString() {
			return String.Format("{0} @ ({1:X}, {2:X}, {3:X}), {4} {5}", mName, mOffsPos, mOffsRot, mOffsScl, mXformOrd, mRotOrd);
		}
	}

	public long mFileTop;
	public uint mDataSize;
	public float mRate;
	public int mFramesNum;
	public int mNodesNum;
	public int mHashesOffs;
	public string mName;
	public cNode[] mNodes;
	public Dictionary<string, int> mNodeMap;

	public cMotClipReader() {
	}
	
	public bool CkFrameNo(int fno) { return fno >= 0 && fno < mFramesNum; }
	
	public int FindNodeIdx(string name) {
		int i = -1;
		if (mNodeMap != null && mNodeMap.ContainsKey(name)) {
			i = mNodeMap[name];
		}
		return i;
	}
	
	public cNode FindNode(string name) {
		cNode node = null;
		int i = FindNodeIdx(name);
		if (i >= 0) {
			node = mNodes[i];
		}
		return node;
	}

	public void Read(BinaryReader br) {
		mFileTop = br.BaseStream.Position;
		uint fmt = br.ReadUInt32();
		if (fmt != DEFS.MOT_CLIP_ID) {
			throw new Exception("!MotClip");
		}
		mDataSize = br.ReadUInt32();
		mRate = br.ReadSingle();
		mFramesNum = br.ReadInt32();
		mNodesNum = br.ReadInt32();
		mHashesOffs = br.ReadInt32();
		br.ReadInt32(); // ext
		br.ReadInt32(); // pad
		mName = nUtl.ReadFixStr(br);
		mNodes = new cNode[mNodesNum];
		mNodeMap = new Dictionary<string, int>();
		for (int i = 0; i < mNodesNum; ++i) {
			br.BaseStream.Seek(DEFS.CLIP_HEADER_SIZE + (i*DEFS.NODE_INFO_SIZE), SeekOrigin.Begin);
			mNodes[i] = new cNode(this);
			mNodes[i].ReadInfo(br);
			string nodeName = mNodes[i].mName;
			if (!mNodeMap.ContainsKey(nodeName)) {
				mNodeMap[nodeName] = i;
			} else {
				throw new Exception("node dup");
			}
		}
		for (int i = 0; i < mNodesNum; ++i) {
			mNodes[i].ReadData(br);
		}
	}
	
	public void Load(string fpath) {
		FileInfo fi = new FileInfo(fpath);
		if (!fi.Exists) {
			throw new Exception(String.Format("File not found: {0}", fpath));
		}
		FileStream fs = File.OpenRead(fpath);
		BinaryReader br = new BinaryReader(fs);
		Read(br);
		br.Close();
	}
	
	protected void DumpLogVecs(TextWriter tw, cNode node) {
		if (node.mRotTrk == null || node.mRotTrk.mSrcMask == 0) return;
		int nfrm = mFramesNum;
		for (int c = 0; c < 3; ++c) {
			tw.WriteLine(@"   {");
			tw.WriteLine("      name = {0}:lv{1}", node.mName, "xyz"[c]);
			tw.Write("      data =");
			for (int i = 0; i < nfrm; ++i) {
				VEC lv = node.GetLogVec(i);
				tw.Write(" {0}", lv[c]);
			}
			tw.WriteLine();
			tw.WriteLine(@"   }");
		}
	}
	
	protected void DumpQuats(TextWriter tw, cNode node) {
		if (node.mRotTrk == null || node.mRotTrk.mSrcMask == 0) return;
		int nfrm = mFramesNum;
		for (int c = 0; c < 4; ++c) {
			tw.WriteLine(@"   {");
			tw.WriteLine("      name = {0}:q{1}", node.mName, "xyzw"[c]);
			tw.Write("      data =");
			for (int i = 0; i < nfrm; ++i) {
				QUAT q = node.GetQuat(i);
				tw.Write(" {0}", q[c]);
			}
			tw.WriteLine();
			tw.WriteLine(@"   }");
		}
	}
	
	protected void DumpRotChans(TextWriter tw, cNode node) {
		if (node.mRotTrk == null || node.mRotTrk.mSrcMask == 0) return;
		int nfrm = mFramesNum;
		for (int c = 0; c < 3; ++c) {
			if ((node.mRotTrk.mSrcMask & (1 << c)) != 0) {
				tw.WriteLine(@"   {");
				tw.WriteLine("      name = {0}:r{1}", node.mName, "xyz"[c]);
				tw.Write("      data =");
				for (int i = 0; i < nfrm; ++i) {
					float[] r = node.GetDegrees(i);
					tw.Write(" {0}", r[c]);
				}
				tw.WriteLine();
				tw.WriteLine(@"   }");
			}
		}
	}
	
	protected void DumpPosChans(TextWriter tw, cNode node) {
		if (node.mPosTrk == null || node.mPosTrk.mSrcMask == 0) return;
		int nfrm = mFramesNum;
		for (int c = 0; c < 3; ++c) {
			if ((node.mPosTrk.mSrcMask & (1 << c)) != 0) {
				tw.WriteLine(@"   {");
				tw.WriteLine("      name = {0}:t{1}", node.mName, "xyz"[c]);
				tw.Write("      data =");
				for (int i = 0; i < nfrm; ++i) {
					VEC pos = node.GetPos(i);
					tw.Write(" {0}", pos[c]);
				}
				tw.WriteLine();
				tw.WriteLine(@"   }");
			}
		}
	}
	
	public void DumpClip(TextWriter tw, DUMP_MODE mode) {
		if (mNodes == null) return;
		int ntrk = 0;
		foreach (cNode node in mNodes) {
			if (node.mRotTrk != null) {
				switch (mode) {
					case DUMP_MODE.QUATS:
						if (node.mRotTrk.mSrcMask != 0) {
							ntrk += 4;
						}
						break;
					case DUMP_MODE.LOGVECS:
						if (node.mRotTrk.mSrcMask != 0) {
							ntrk += 3;
						}
						break;
					default:
						ntrk += node.mRotTrk.NumSrcChannels;
						break;
				}
			}
			if (node.mPosTrk != null) {
				ntrk += node.mPosTrk.NumSrcChannels;
			}
		}
		tw.WriteLine(@"{");
		tw.WriteLine("   rate = {0}", mRate);
		tw.WriteLine("   start = {0}", -1);
		tw.WriteLine("   tracklength = {0}", mFramesNum);
		tw.WriteLine("   tracks = {0}", ntrk);
		foreach (cNode node in mNodes) {
			switch (mode) {
				case DUMP_MODE.LOGVECS:
					DumpLogVecs(tw, node);
					break;
				case DUMP_MODE.QUATS:
					DumpQuats(tw, node);
					break;
				default:
					DumpRotChans(tw, node);
					break;
			}
			DumpPosChans(tw, node);
		}
		tw.WriteLine(@"}");
	}
	
	public void DumpClip(TextWriter tw) {
		DumpClip(tw, DUMP_MODE.DEFAULT);
	}
}


public class cMotLibWriter {

	public class cEntry {
		public cMotLibWriter mLib;
		public string mSrcPath;
		public cHouClip mHouClip;
		public cMotClipWriter mMotClip;
		
		public cEntry(cMotLibWriter lib, string fpath) {
			mSrcPath = fpath;
			mHouClip = new cHouClip();
			mHouClip.Load(fpath);
			mMotClip = new cMotClipWriter(mHouClip);
		}
	}

	public List<cEntry> mEntries;
	public Dictionary<string, int> mPathToIdx;
	public List<string> mNodeNames;
	public Dictionary<string, int> mNodeMap;
	
	public cMotLibWriter() {
		mEntries = new List<cEntry>();
		mPathToIdx = new Dictionary<string, int>();
		mNodeNames = new List<string>();
		mNodeMap = new Dictionary<string, int>();
	}

	public void AddHouClip(string fpath) {
		if (mPathToIdx.ContainsKey(fpath)) {
			throw new Exception("motlib: clip dup.");
		}
		var entry = new cEntry(this, fpath);
		int clipIdx = mEntries.Count;
		mPathToIdx[fpath] = clipIdx;
		mEntries.Add(entry);
		foreach (string nodeName in entry.mMotClip.mNodeNames) {
			if (!mNodeMap.ContainsKey(nodeName)) {
				int nodeNameIdx = mNodeNames.Count;
				mNodeMap[nodeName] = nodeNameIdx;
				mNodeNames.Add(nodeName);
			}
		}
	}
	
	public void Write(BinaryWriter bw) {
		int nclips = mEntries.Count;
		int nnodes = mNodeNames.Count;
		bw.Write(DEFS.MOT_LIB_ID);
		bw.Write(nclips);
		bw.Write(nnodes);
	}
	
	public void Save(string fpath) {
		var ofs = new FileStream(fpath, FileMode.Create);
		var bw = new BinaryWriter(ofs);
		Write(bw);
		bw.Close();
	}
}


public class HClipTool {
	
	private static void PrintClipInfo(cHouClip clip, TextWriter tw) {
		tw.WriteLine("FPS: {0}", clip.mRate);
		int nfrm = clip.mFramesNum;
		int ntrk = clip.mTracksNum;
		tw.WriteLine("#frames = {0}, #tracks = {1}", nfrm, ntrk);
		for (int i = 0; i < ntrk; ++i) {
			cHouClip.cTrack trk = clip.mTracks[i];
			tw.WriteLine("[{0:D2}]: {1}, {2}, range=({3}, {4})", i, trk.ShortName, trk.ChannelName, trk.mMinVal, trk.mMaxVal);
		}
	}

	public static int Main(string[] argStrs) {
		Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

		cArgs args = new cArgs();
		args.Parse(argStrs);
		
		if (args.ArgNum < 1) {
			Console.Error.WriteLine("hclip -[logvecs|quads] <motion.clip>");
			return -1;
		}
		
		string clpPath = args.GetArg(0);
		var clip = new cHouClip();
		clip.Load(clpPath);
		//PrintClipInfo(clip, Console.Out);

		var mcw = new cMotClipWriter(clip);
		string motPath = clpPath.Replace(".clip", ".mclp");
		mcw.Save(motPath);

	
		var mcr = new cMotClipReader();
		mcr.Load(motPath);
		
		DUMP_MODE dumpMode = DUMP_MODE.DEFAULT;
		if (args.HasOption("logvecs")) {
			dumpMode = DUMP_MODE.LOGVECS;
		} else if (args.HasOption("quats")) {
			dumpMode = DUMP_MODE.QUATS;
		}
		mcr.DumpClip(Console.Out, dumpMode);
		
		return 0;
	}

}
