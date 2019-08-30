// Author: Sergey Chaban <sergey.chaban@gmail.com>

using System;
using System.IO;
using System.Threading;
using System.Globalization;
using System.Collections.Generic;

public static class nUtl {
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

		protected int ReadDataRLE(string[] toks, int idx) {
			int n = mClip.mFramesNum;
			mData = new float[n];
			int cnt = 0;
			while (true) {
				if (cnt >= n) break;
				string t = toks[idx++];
				if (t.StartsWith("@")) {
					int nrle = nUtl.ParseInt(t.Substring(1));
					float val = nUtl.ParseF32(toks[idx++]);
					if (cnt == 0) {
						mMinVal = val;
						mMaxVal = val;
					} else {
						mMinVal = Math.Min(mMinVal, val);
						mMaxVal = Math.Max(mMaxVal, val);
					}
					for (int i = 0; i < nrle; ++i) {
						mData[cnt++] = val;
					}
				} else {
					float val = nUtl.ParseF32(t);
					if (cnt == 0) {
						mMinVal = val;
						mMaxVal = val;
					} else {
						mMinVal = Math.Min(mMinVal, val);
						mMaxVal = Math.Max(mMaxVal, val);
					}
					mData[cnt++] = val;
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
					case "data_rle":
						i = ReadDataRLE(toks, i);
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

	public string mPath = "<unknown>";
	public string mName = "<unknown>";
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

	public void WriteLooped(TextWriter tw) {
		int nfrm = mFramesNum;
		int ntrk = mTracksNum;
		tw.WriteLine(@"{");
		tw.WriteLine("   rate = {0}", mRate);
		tw.WriteLine("   start = {0}", mStart - 1);
		tw.WriteLine("   tracklength = {0}", nfrm + 1);
		tw.WriteLine("   tracks = {0}", ntrk);
		for (int i = 0; i < mTracksNum; ++i) {
			cHouClip.cTrack trk = mTracks[i];
			tw.WriteLine(@"   {");
			tw.WriteLine("      name = {0}", trk.mName);
			tw.Write("      data =");
			for (int j = 0; j < nfrm; ++j) {
				tw.Write(" {0}", trk.mData[j]);
			}
			if (trk.ShortName == "n_Move") {
				if (trk.ChannelName == "tz" && nfrm > 1) {
					tw.Write(" {0}", trk.mData[nfrm - 1] + (trk.mData[1] - trk.mData[0]));
				} else {
					tw.Write(" {0}", trk.mData[0]);
				}
			} else {
				tw.Write(" {0}", trk.mData[0]);
			}
			tw.WriteLine();
			tw.WriteLine(@"   }");
		}
		tw.WriteLine(@"}");
	}
}

public sealed class clip_loop {

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
			Console.Error.WriteLine("clip_loop <motion.clip>");
			return -1;
		}
		
		string clpPath = args.GetArg(0);
		var clip = new cHouClip();
		clip.Load(clpPath);
		//PrintClipInfo(clip, Console.Out);
		clip.WriteLooped(Console.Out);

		return 0;
	}


}
