# Author: Sergey Chaban <sergey.chaban@gmail.com>

import sys
import numpy as nu
import numpy.linalg as la

def dist_mtx(pts):
	n = len(pts)
	m = nu.empty((n, n))
	for i in range(n):
		for j in range(n):
			m[i][j] = pts[i] - pts[j]
	return nu.abs(m)

def func(x): return nu.power(x, 3)

class Anim:
	def __init__(self):
		self.names = {}
		self.chans = []

	def load(self, fpath):
		chans = nu.genfromtxt(fpath, names = True, deletechars = "")
		nchn = len(chans[0])
		nfrm = len(chans)
		for idx, name in enumerate(chans.dtype.names):
			self.names[name] = idx
			self.chans.append(nu.array( [chans[i][idx] for i in range(nfrm)] ))

	def num_chans(self): return len(self.chans)
	def num_frames(self): return len(self.chans[0])

	def get_chan(self, name):
		if name not in self.names: return None
		return self.chans[self.names[name]]

	def fit_curves(self, gap):
		nfrm = self.num_frames()
		pts = []
		pt = 0
		frmax = nfrm - 1
		step = gap + 1
		while True:
			pts.append(pt)
			pt += step
			if pt >= frmax:
				pts.append(frmax)
				break
		nsmp = len(pts)
		self.pts = pts
		print("# samples: " + str(len(pts)))
		D = func(dist_mtx(pts))
		nchn = self.num_chans()
		v = nu.zeros((nsmp, nchn))
		for ch in range(nchn):
			for pt in range(nsmp):
				v[pt][ch] = self.chans[ch][pts[pt]]
		w = la.solve(D, v)
		self.wgts = []
		for ch in range(nchn):
			self.wgts.append([w[i][ch] for i in range(nsmp)])

	def plot_curve(self, idx):
		nfrm = self.num_frames()
		w = self.wgts[idx]
		n = len(w)
		v = []
		for frm in range(nfrm):
			e = nu.zeros((n))
			for i in range(n):
				e[i] = frm - self.pts[i]
			e = func(nu.abs(e))
			v.append(nu.dot(e, w))
		return v

	def write_clip(self, out, fps = 30):
		nfrm = self.num_frames()
		nchn = self.num_chans()
		out.write("{\n");
		out.write("   rate = " + str(fps) + "\n")
		out.write("   start = -1\n")
		out.write("   tracklength = " + str(nfrm) + "\n")
		out.write("   tracks = " + str(nchn) + "\n")
		for i in range(nchn):
			name = list(self.names.keys())[i]
			data = self.plot_curve(i)
			out.write("   {\n");
			out.write("      name = " + name)
			out.write("      data =")
			for val in data: out.write(" " + str(val))
			out.write("\n")
			out.write("   }\n")
		out.write("}\n");

	def save_clip(self, fpath, fps = 30):
		out = open(fpath, "w")
		if not out: return
		self.write_clip(out, fps)
		out.close()

if __name__=="__main__":
	narg = len(sys.argv)
	if narg < 2:
		print("anim_approx <fname> <gap>")
		sys.exit(-1)
	fpath = sys.argv[1]
	print("in: " + fpath)
	gap = 3
	if narg > 2:
		gap = int(sys.argv[2])
	print("gap: " + str(gap))

	anim = Anim()
	anim.load(fpath)
	print("# chans: " + str(anim.num_chans()))
	print("# frames: " + str(anim.num_frames()))
	anim.fit_curves(gap)
	outPath = fpath + ".clip"
	print("out: " + outPath)
	anim.save_clip(outPath)
