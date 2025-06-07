import sys
import hou
import os
import imp
import re
import struct
import inspect
from math import *
from array import array

def getChannelsInGroup(grpName):
	lst = hou.hscript("chgls -l " + grpName)[0].split()
	if len(lst): del lst[0]
	return lst

def getParamKeysInRange(prm, start, end):
	kf = None
	trk = prm.overrideTrack()
	if trk: kf = prm.getReferencedParm().keyframesInRange(start, end)
	else: kf = prm.keyframesInRange(start, end)
	return kf

def getRotOrd(node):
	rOrd = "xyz"
	if node:
		rOrdParm = node.parm("rOrd")
		if rOrdParm: rOrd = rOrdParm.evalAsString()
	return rOrd

def getXformOrd(node):
	xOrd = "srt"
	if node:
		xOrdParm = node.parm("xOrd")
		if xOrdParm: xOrd = xOrdParm.evalAsString()
	return xOrd

def getChIdx(map, name):
	if name in map:
		return '%(#)03d' % {"#":map[name]+1}
	return "000"

class Anim:
	def __init__(self, start, end, chLst):
		self.start = start
		self.end = end
		self.nodeLst = []
		self.prmLst = []
		for chPath in chLst:
			prm = hou.parm(chPath)
			if prm:
				hasAnim = len(getParamKeysInRange(prm, self.start, self.end)) > 0
				hasCHOP = prm.overrideTrack() is not None
				if hasAnim or hasCHOP:
					self.prmLst.append(prm)
		nch = len(self.prmLst)
		if nch < 1: return
		for prm in self.prmLst:
			node = prm.node()
			if not node in self.nodeLst:
				#print node.path()
				self.nodeLst.append(node)
		print "Anim", nch, "channels, from", self.start, "to", self.end
		self.anmMtx = []
		for prm in self.prmLst:
			trk = []
			for fno in xrange(self.start, self.end+1):
				val = prm.evalAtFrame(fno);
				trk.append(val)
			self.anmMtx.append(trk)

	def save(self, fpath):
		nch = len(self.prmLst)
		if nch < 1:
			print "There is no data to export."
			return

		f = open(fpath, "wb")
		if not f: return
		print "Exporting animation to '"+fpath+"'."
		f.write(struct.pack("11s", "Octave-1-L"))

		ncols = nch # column-major, transpose after loading
		nrows = 32 # fixed-length names
		f.write(struct.pack(
			"<i5siBi6siii",
			4, "name", 0, 0xFF, # 0xFF -> new binary
			6, "string", -2, # 2-dimensional mtx (-ndims for new binary)
			nrows, ncols
		))
		chMap = {}
		for i, prm in enumerate(self.prmLst):
			name = prm.node().name() + ":" + prm.name()
			f.write(struct.pack(str(nrows)+"s", name))
			chMap[name] = i

		mtx = self.anmMtx
		ncols = len(mtx) # column-major
		nrows = len(mtx[0])
		f.write(struct.pack(
			"<i5siBi6siiiB",
			4, "data", 0, 0xFF, # 0xFF -> new binary
			6, "matrix", -2, # 2-imensional mtx (-ndims for new binary)
			nrows, ncols,
			6 # LS_FLOAT
		))
		for v in mtx:
			f.write(struct.pack("<"+str(nrows)+"f", *v))

		ncols = len(self.nodeLst)
		nrows = 64
		f.write(struct.pack(
			"<i5siBi6siii",
			4, "info", 0, 0xFF, # 0xFF -> new binary
			6, "string", -2, # 2-imensional mtx (-ndims for new binary)
			nrows, ncols
		))
		for node in self.nodeLst:
			name = node.name()
			info = getXformOrd(node)
			info += getRotOrd(node)
			for xform in "trs":
				for axis in "xyz":
					info += getChIdx(chMap, name + ":" + xform + axis)
			info += name
			f.write(struct.pack(str(nrows)+"s", info))

		f.close()

def animExport(outPath, chLst):
	frange = hou.playbar.playbackRange()
	minFrame = int(frange[0])
	maxFrame = int(frange[1])
	anm = Anim(minFrame, maxFrame, chLst)
	anm.save(outPath)

if __name__=="__main__":
	exePath = os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))
	outPath = exePath + "/anim.mat"
	chLst = getChannelsInGroup("MOT")
	animExport(outPath, chLst)

