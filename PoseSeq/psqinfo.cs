// Pose Sequence file info utility
// Author: Sergey Chaban <sergey.chaban@gmail.com>

using System;
using System.IO;
using System.Text;
using System.Numerics;
using System.Globalization;
using System.Collections.Generic;

sealed class Util {
	public static uint FOURCC(char c1, char c2, char c3, char c4) {
		return (uint)((((byte)c4) << 24) | (((byte)c3) << 16) | (((byte)c2) << 8) | ((byte)c1));
	}

	public static int ParseInt(string s) {
		try {
			NumberStyles nstyles = NumberStyles.AllowExponent | NumberStyles.AllowDecimalPoint | NumberStyles.AllowLeadingSign;
			long i = Int64.Parse(s, nstyles);
			return (int)i;
		} catch {
			return -1;
		}
	}

	public static string ReadString(BinaryReader br) {
		string s = "";
		while (true) {
			char c = (char)br.ReadByte();
			if (c == 0) break;
			s += c;
		}
		return s;
	}

	// http://www.isthe.com/chongo/tech/comp/fnv/index.html
	public static uint StrHash32(string s) {
		uint h = 2166136261;
		foreach (char c in s) {
			h ^= (byte)c;
			h *= 16777619;
		}
		return h;
	}

	public static ushort StrHash16(string s) {
		uint h = StrHash32(s);
		return (ushort)((h >> 16) ^ (h & 0xFFFF));
	}

	public static uint ByteDiv(uint x) {
		uint res = x >> 3;
		if ((x & 7) != 0) ++res;
		return res;
	}

	public static TextWriter MakeTextFile(string fname) {
		FileStream ofs = new FileStream(fname, FileMode.Create);
		TextWriter tw = new StreamWriter(ofs);
		return tw;
	}
}

class CmdLine {
	public List<string> mArgs;
	public Dictionary<string, string> mOpts;

	public CmdLine(string[] args) {
		mArgs = new List<string>();
		mOpts = new Dictionary<string, string>();
		Parse(args);
	}

	protected static bool IsOption(string arg) {
		return arg.StartsWith("-");
	}

	protected void Parse(string[] args) {
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

	public int ArgNum {
		get {
			return mArgs.Count;
		}
	}

	public int OptNum {
		get {
			return mOpts.Count;
		}
	}

	public bool HasOpt(string opt) {
		return mOpts.ContainsKey(opt);
	}

	public bool GetBoolOpt(string name, bool defVal) {
		if (HasOpt(name)) {
			string optVal = mOpts[name];
			if (optVal == "1" || optVal == "true") {
				return true;
			}
		}
		return defVal;
	}

	public string GetStringOpt(string name, string defVal) {
		string res = defVal;
		if (HasOpt(name)) {
			res = mOpts[name];
		}
		return res;
	}

}

enum eXformOrder {
	SRT = 0,
	STR = 1,
	RST = 2,
	RTS = 3,
	TSR = 4,
	TRS = 5
}

enum eRotOrder {
	XYZ = 0,
	XZY = 1,
	YXZ = 2,
	YZX = 3,
	ZXY = 4,
	ZYX = 5
}

[Flags]
enum eTrkMask {
	ROT = 1,
	POS = 2,
	SCL = 4
}

struct VEC {
	public float x, y, z, w;

	public void Read(BinaryReader br) {
		x = br.ReadSingle();
		y = br.ReadSingle();
		z = br.ReadSingle();
		w = br.ReadSingle();
	}

	public override string ToString() {
		return String.Format("<{0}, {1}, {2}, {3}>", x, y, z, w);
	}
}

struct PSQHEAD {
	public uint dataSize;
	public int nodeNum;
	public int poseNum;
	public int vecNum;
	public uint nodeOffs;
	public uint sizeOffs;
	public uint vecOffs;
	public uint nbits;

	public void Read(BinaryReader br) {
		uint magic = br.ReadUInt32(); // +00
		if (magic != Util.FOURCC('P', 'S', 'Q', '\0')) {
			throw new Exception("invalid data (PSQ)");
		}
		dataSize = br.ReadUInt32();
		nodeNum = br.ReadUInt16();
		poseNum = br.ReadUInt16();
		vecNum = br.ReadUInt16();
		br.ReadUInt16(); // reserved
		nodeOffs = br.ReadUInt32();
		sizeOffs = br.ReadUInt32();
		vecOffs = br.ReadUInt32();
		nbits = br.ReadUInt32();
	}

	public void WriteInfo(TextWriter tw) {
		tw.WriteLine("data size = 0x{0:X}", dataSize);
		tw.WriteLine("#nodes = 0x{0:X}", nodeNum);
		tw.WriteLine("nodes @ 0x{0:X}", nodeOffs);
		tw.WriteLine("#poses = 0x{0:X}", poseNum);
		tw.WriteLine("pose size list @ 0x{0:X}", sizeOffs);
		tw.WriteLine("#vecs = 0x{0:X}", vecNum);
		tw.WriteLine("vecs @ 0x{0:X}", vecOffs);
		tw.WriteLine("#pose bits = 0x{0:X}", nbits);
	}
}

class NodeName {
	public ushort mHash;
	public string mStr;

	public void Read(BinaryReader br) {
		mHash = br.ReadUInt16();
		mStr = Util.ReadString(br);
	}

	public bool Verify() {
		if (mStr != null) {
			return mHash == Util.StrHash16(mStr);
		}
		return true;
	}

	public override int GetHashCode() {
		return mHash;
	}

	public override bool Equals(object obj) {
		if (Object.ReferenceEquals(this, obj)) return true;
		if (obj is NodeName) {
			if ((obj as NodeName).mHash == mHash && (obj as NodeName).mStr == mStr) {
				return true;
			}
		}
		if (obj is string) {
			ushort h = Util.StrHash16(obj as string);
			if (h == mHash) {
				return (obj as string) == mStr;
			}
		}
		return false;
	}
}

class PoseNode {
	public PoseSeq mPsq;
	public NodeName mName;
	public eXformOrder mXformOrd;
	public eRotOrder mRotOrd;
	public ushort mRotBitInfo;
	public ushort mPosBitInfo;
	public ushort mSclBitInfo;
	public short mRotBaseId;
	public short mRotSizeId;
	public short mPosBaseId;
	public short mPosSizeId;
	public short mSclBaseId;
	public short mSclSizeId;
	public byte mRotAxisMask;
	public byte mPosAxisMask;
	public byte mSclAxisMask;
	public eTrkMask mTrkMask;

	public PoseNode(PoseSeq psq) {
		mPsq = psq;
	}

	public string Name {
		get {
			if (mName != null && mName.mStr != null) {
				if (!mName.Verify()) {
					return "<bad>:" + mName.mStr;
				}
				return mName.mStr;
			}
			return "<???>";
		}
	}

	public void Read(BinaryReader br) {
		uint nameOffs = br.ReadUInt32();
		mXformOrd = (eXformOrder)br.ReadByte();
		mRotOrd = (eRotOrder)br.ReadByte();
		mRotBitInfo = br.ReadUInt16();
		mPosBitInfo = br.ReadUInt16();
		mSclBitInfo = br.ReadUInt16();
		mRotBaseId = br.ReadInt16();
		mRotSizeId = br.ReadInt16();
		mPosBaseId = br.ReadInt16();
		mPosSizeId = br.ReadInt16();
		mSclBaseId = br.ReadInt16();
		mSclSizeId = br.ReadInt16();
		mRotAxisMask = br.ReadByte();
		mPosAxisMask = br.ReadByte();
		mSclAxisMask = br.ReadByte();
		mTrkMask = (eTrkMask)br.ReadByte();

		br.BaseStream.Seek(nameOffs, SeekOrigin.Begin);
		mName = new NodeName();
		mName.Read(br);
	}

	public bool HasRotTrack { get { return (mTrkMask & eTrkMask.ROT) != 0; } }
	public bool HasPosTrack { get { return (mTrkMask & eTrkMask.POS) != 0; } }
	public bool HasSclTrack { get { return (mTrkMask & eTrkMask.SCL) != 0; } }

	public void WriteInfo(TextWriter tw) {
		tw.WriteLine("\"{0}\"", Name);
		tw.WriteLine("order: xform={0}, rot={1}", mXformOrd, mRotOrd);
		tw.WriteLine("tracks: rot={0}, pos={1}, scl={2}", HasRotTrack ? "yes" : "no", HasPosTrack ? "yes" : "no", HasSclTrack ? "yes" : "no");
		tw.WriteLine("rot quant base @ [{0}] = {1}", mRotBaseId, mPsq.GetVecStr(mRotBaseId));
		tw.WriteLine("rot quant size @ [{0}] = {1}", mRotSizeId, mPsq.GetVecStr(mRotSizeId));
		tw.WriteLine("rot bit info = 0x{0:X}", mRotBitInfo);
		tw.WriteLine("pos quant base @ [{0}] = {1}", mPosBaseId, mPsq.GetVecStr(mPosBaseId));
		tw.WriteLine("pos quant size @ [{0}] = {1}", mPosSizeId, mPsq.GetVecStr(mPosSizeId));
		tw.WriteLine("pos bit info = 0x{0:X}", mPosBitInfo);
		tw.WriteLine("scl quant base @ [{0}] = {1}", mSclBaseId, mPsq.GetVecStr(mSclBaseId));
		tw.WriteLine("scl quant size @ [{0}] = {1}", mSclSizeId, mPsq.GetVecStr(mSclSizeId));
		tw.WriteLine("scl bit info = 0x{0:X}", mSclBitInfo);
	}

	public override string ToString() {
		return Name;
	}
}

class PoseSeq {
	public string mFileName;
	public long mFileSize;
	public PSQHEAD mHead;
	public VEC[] mVec;
	public PoseNode[] mNode;
	public uint[] mPoseSize;
	public long mPoseTop;

	public void Read(BinaryReader br) {
		mHead.Read(br);
		int nvec = mHead.vecNum;
		if (nvec > 0) {
			mVec = new VEC[nvec];
			br.BaseStream.Seek(mHead.vecOffs, SeekOrigin.Begin);
			for (int i = 0; i < nvec; ++i) {
				mVec[i].Read(br);
			}
		}
		int nnode = mHead.nodeNum;
		mNode = new PoseNode[nnode];
		for (int i = 0; i < nnode; ++i) {
			br.BaseStream.Seek(mHead.nodeOffs + (i * 0x1C), SeekOrigin.Begin);
			mNode[i] = new PoseNode(this);
			mNode[i].Read(br);
		}
		int npose = mHead.poseNum;
		mPoseSize = new uint[npose];
		br.BaseStream.Seek(mHead.sizeOffs, SeekOrigin.Begin);
		for (int i = 0; i < npose; ++i) {
			mPoseSize[i] = br.ReadUInt32();
		}
		mPoseTop = br.BaseStream.Position;
	}

	public string GetVecStr(int idx) {
		if (mVec == null || (uint)idx >= mHead.vecNum) {
			return "<none>";
		}
		return mVec[idx].ToString();
	}

	public void Load(string fname) {
		FileInfo fi = new FileInfo(fname);
		if (!fi.Exists) {
			throw new Exception(String.Format("File \"{0}\" not found.", fname));
		}
		mFileName = fname;
		mFileSize = fi.Length;
		FileStream fs = File.OpenRead(fname);
		BinaryReader br = new BinaryReader(fs);
		Read(br);
		br.Close();
	}

	public void WriteInfo(TextWriter tw) {
		mHead.WriteInfo(tw);
		tw.WriteLine();
		if (mNode != null) {
			Dictionary<int, string> nameDict = new Dictionary<int, string>();
			int hcol = 0;
			for (int i = 0; i < mHead.nodeNum; ++i) {
				tw.WriteLine("-- node[{0}] ------", i);
				mNode[i].WriteInfo(tw);
				if (nameDict.ContainsKey(mNode[i].mName.mHash)) {
					++hcol;
				} else {
					nameDict[mNode[i].mName.mHash] = mNode[i].mName.mStr;
				}
			}
			tw.WriteLine();
			tw.WriteLine("name hash collisions = {0}", hcol);
		}
		tw.WriteLine();
		if (mPoseSize != null) {
			uint psum = 0;
			uint org = 0;
			for (int i = 0; i < mHead.poseNum; ++i) {
				uint psize = mPoseSize[i];
				psum += psize;
				tw.WriteLine("pose[{0}]: @ 0x{1:X}:{2}, size = 0x{3:X} bits (0x{4:X} bytes)", i, mPoseTop + (org/8), org&7, psize, Util.ByteDiv(psize));
				org += psize;
			}
			tw.WriteLine("(total = 0x{0:X})", psum);
		}
	}
}


class psqinfo {

	private static void PrintUsage() {
		Console.WriteLine("PSQ file info");
		Console.WriteLine("usage: psqinfo <options> <filename.psq>");
	}

	public static int Main(string[] args) {
		CmdLine cmd = new CmdLine(args);
		if (cmd.ArgNum < 1) {
			PrintUsage();
			return -1;
		}
		PoseSeq psq = new PoseSeq();
		psq.Load(cmd.mArgs[0]);


		TextWriter tw = Console.Out;
		string outName = cmd.GetStringOpt("out", null);
		if (outName != null) {
			tw = Util.MakeTextFile(outName);
		}

		psq.WriteInfo(tw);

		if (outName != null) {
			tw.Close();
		}

		return 0;
	}

}
