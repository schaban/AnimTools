# Pose Sequence library
# Author: Sergey Chaban <sergey.chaban@gmail.com>

import sys
import os
from math import *
from array import array

def align(x, y): return ((x + (y - 1)) & (~(y - 1)))

def sq(x): return x*x

def rcp0(x):
	if x: return 1.0 / x
	return 0.0

def div0(x, y):
	if y: return x / y
	return 0.0

def ceilDiv(x, div):
	res = x / div
	if x % div: res += 1
	return res

def clz32(x):
	if x == 0: return 32
	n = 0
	s = 16
	while s:
		m = ((1 << s) - 1) << (32 - s)
		if (x & m) == 0:
			n += s
			x <<= s
		s >>= 1
	return n

def ctz32(x):
	if x == 0: return 32
	n = 0
	s = 16
	while s:
		m = (1 << s) - 1
		if (x & m) == 0:
			n += s
			x >>= s
		s >>= 1
	return n

def bitLen32(x): return 32 - clz32(x)

def popCnt(x):
	n = 0
	while x:
		x &= x-1
		n += 1
	return n

def getBitsF32(x):
	a = array('f', [x]).tostring()
	return ord(a[0]) | (ord(a[1]) << 8) | (ord(a[2]) << 16) | (ord(a[3]) << 24)

def setBitsF32(x):
	a = array('f')
	s = chr(x&0xFF) + chr((x>>8)&0xFF) + chr((x>>16)&0xFF) + chr((x>>24)&0xFF)
	a.fromstring(s)
	return a[0]

def getFractionBitsF32(x):
	b = getBitsF32(x)
	if (b & 0x7FFFFFFF) == 0: return 0
	return b & ((1<<23)-1)

def getExponentBitsF32(x):
	b = getBitsF32(x)
	return (b>>23)&0xFF

def getFractionF32(x):
	if x == 0.0: return 0
	m = getFractionBitsF32(x)
	return 1.0 + float(m) / float(1<<23)

def getExponentF32(x):
	return getExponentBitsF32(x) - 127

def buildF32(e, m, s=0):
	return setBitsF32((((int(e)+127)&0xFF) << 23) | int(m) | ((int(s)&1)<<31))

def intEncodeFloat(x, bits):
	return long(x * ((1<<(bits-1)) - 1)) & ((1<<bits) - 1)

def intEncodeAng(ang, bits):
	return long(ang * (float(1<<bits) / (pi*2.0))) & ((1<<bits) - 1)

# http://lgdv.cs.fau.de/publications/publication/Pub.2010.tech.IMMD.IMMD9.onfloa/
# http://jcgt.org/published/0003/02/01/

def signNZ(x):
	if x >= 0.0: return 1.0
	return -1.0

def octaEncodeAxis(axis):
	x = axis[0]
	y = axis[1]
	z = axis[2]
	ax = abs(x)
	ay = abs(y)
	az = abs(z)
	d = 1.0 / (ax + ay + az)
	ox = x * d
	oy = y * d
	if z < 0.0:
		return [(1.0 - abs(oy)) * signNZ(ox), (1.0 - abs(ox)) * signNZ(oy)]
	return [limF32(ox), limF32(oy)]

def octaEncodeQuat(q, axisBits, angleBits):
	cosh = q[3]
	ang = acos(cosh) * 2.0
	iang = intEncodeAng(ang, angleBits)
	bits = 0
	if iang:
		sinh = sqrt(1.0 - sq(cosh))
		axis = [q[0], q[1], q[2]]
		if sinh:
			d = 1.0 / sinh;
			axis[0] *= d
			axis[1] *= d
			axis[2] *= d
		alen = sqrt(sq(axis[0]) + sq(axis[1]) + sq(axis[2]))
		if alen:
			d = 1.0 / alen
			axis[0] *= d
			axis[1] *= d
			axis[2] *= d
		if alen:
			oct = octaEncodeAxis(axis)
		else:
			oct = [0.0, 0.0]
		bits = intEncodeFloat(oct[0], axisBits)
		bits |= intEncodeFloat(oct[1], axisBits) << axisBits
		bits |= iang << (axisBits*2)
	else:
		if cosh < 0:
			bits = (1 << (axisBits*2)) - 1
	return bits

def limF32(x):
	a = array('f', [x])
	s = a.tostring()
	b = array('f')
	b.fromstring(s)
	return b[0]

def vecLimF32(v):
	return [limF32(v[i]) for i in xrange(len(v))]
	
def encodeQuantizedExpF32(x):
	e = getExponentF32(x)
	if e > 0: print "!!! Error: exponent > 0 !!!"
	return (abs(e) + 1) & 0x7F

def encodeQuantizedVec(v, mcut = 0):
	maxExp = 0
	for x in v: maxExp = max(maxExp, encodeQuantizedExpF32(x))
	n = bitLen32(maxExp)
	bits = 1 << n
	cnt = n + 1
	for x in v:
		e = encodeQuantizedExpF32(x)
		bits |= e << cnt
		cnt += n
		m = getFractionBitsF32(x)
		m >>= mcut
		bits |= m << cnt
		cnt += 23-mcut
	return (bits, cnt)

def decodeQuantizedExpF32(e):
	e -= 1
	if e:
		if e < 0: e = -127
		else: e = -e
	return e

def decodeQuantizedVec(bits, nelem, mcut = 0):
	n = ctz32(bits & 0xFFFFFFFF)
	cnt = n + 1
	v = []
	for i in xrange(nelem):
		ee = (bits >> cnt) & ((1 << n) - 1)
		e = decodeQuantizedExpF32(ee)
		cnt += n
		m = (bits >> cnt)
		m &= ((1<<(23-mcut)) - 1)
		m <<= mcut
		cnt += 23-mcut
		v.append(buildF32(e, m))
	return v

# FMV-1a
# http://www.isthe.com/chongo/tech/comp/fnv/index.html
def strHash32(s):
	h = 2166136261
	for c in s:
		h ^= ord(c)
		h *= 16777619
		h &= 0xFFFFFFFF
	return h

def strHash16(s):
	h = strHash32(s)
	h = (h >> 16) ^ (h & 0xFFFF)
	return h & 0xFFFF

class BinWriter:
	def __init__(self): pass
	
	def open(self, name):
		self.name = name
		self.file = open(name, "wb")

	def close(self): self.file.close()
	def seek(self, offs): self.file.seek(offs)
	def getPos(self): return self.file.tell()
	def writeI8(self, val): array('b', [val]).tofile(self.file)
	def writeU8(self, val): array('B', [val]).tofile(self.file)
	def writeI32(self, val): array('i', [val]).tofile(self.file)
	def writeU32(self, val): array('I', [val]).tofile(self.file)
	def writeI16(self, val): array('h', [val]).tofile(self.file)
	def writeU16(self, val): array('H', [val]).tofile(self.file)
	def writeF32(self, val): array('f', [val]).tofile(self.file)
	def writeFV(self, v):
		for x in v: self.writeF32(x)
	def writeFOURCC(self, str):
		n = len(str)
		if n > 4: n = 4
		for i in xrange(n): self.writeU8(ord(str[i]))
		for i in xrange(4-n): self.writeU8(0)
	def writeStr(self, str):
		for c in str: self.writeU8(ord(c))
		self.writeU8(0)
	def writeBytes(self, data, nbytes):
		t = data
		for i in xrange(nbytes):
			self.writeU8(t & 0xFF)
			t >>= 8

	def patch(self, offs, val):
		nowPos = self.getPos()
		self.seek(offs)
		self.writeU32(val)
		self.seek(nowPos)

	def patchCur(self, offs): self.patch(offs, self.getPos())

	def align(self, x):
		pos0 = self.getPos()
		pos1 = align(pos0, x)
		for i in xrange(pos1-pos0): self.writeU8(0xFF)

class VecList:
	def __init__(self):
		self.lst = []
		self.map = {}
		self.idx = 0

	def add(self, vec):
		if len(vec) == 3: v = tuple(vec) + (1,)
		else: v = tuple(vec)
		if v not in self.map:
			self.map[v] = self.idx
			self.lst.append(v)
			self.idx += 1
		return self.map[v]

	def write(self, out):
		for v in self.lst: out.writeFV(v)

	def getCount(self): return self.idx

class XORDER:
	def __init__(self): raise Error, "enum"
XORDER.SRT = 0
XORDER.STR = 1
XORDER.RST = 2
XORDER.RTS = 3
XORDER.TSR = 4
XORDER.TRS = 5

def xformOrdFromStr(str):
	try: return getattr(XORDER, str.upper())
	except: return XORDER.SRT

class RORDER:
	def __init__(self): raise Error, "enum"
RORDER.XYZ = 0
RORDER.XZY = 1
RORDER.YXZ = 2
RORDER.YZX = 3
RORDER.ZXY = 4
RORDER.ZYX = 5

def rotOrdFromStr(str):
	try: return getattr(RORDER, str.upper())
	except: return RORDER.XYZ

def rOrdFromStr(s):
	if s == 'xyz': return RORDER.XYZ
	elif s == 'xzy': return RORDER.XZY
	elif s == 'yxz': return RORDER.YXZ
	elif s == 'yzx': return RORDER.YZX
	elif s == 'zxy': return RORDER.ZXY
	elif s == 'zyx': return RORDER.ZYX
	return RORDER.XYZ

class BaseTrack:
	def __init__(self, data):
		self.bitInfo = 0
		self.baseId = -1
		self.sizeId = -1
		self.data = data
		for i, val in enumerate(data):
			self.growBBox(val, i)
		r = xrange(len(self.bbMin))
		self.bbSize = vecLimF32([self.bbMax[i] - self.bbMin[i] for i in r])
		self.bbScl = vecLimF32([rcp0(self.bbSize[i]) for i in r])
		if not self.isConstant():
			self.encode()

	def getRawFramesNum(self):
		if self.data: return len(self.data)
		return 0

	def growBBox(self, val, i):
		if i == 0:
			self.bbMin = val
			self.bbMax = val
		else:
			r = xrange(len(val))
			self.bbMin = [min(self.bbMin[j], val[j]) for j in r]
			self.bbMax = [max(self.bbMax[j], val[j]) for j in r]

	def isConstant(self):
		for x in self.bbSize:
			if x: return False
		return True
	
	def getAxisMask(self):
		msk = 0
		for i, x in enumerate(self.bbSize):
			if x: msk |= 1 << i
		return msk

	def getAxisCount(self):
		cnt = 0
		for i, x in enumerate(self.bbSize):
			if x: cnt += 1
		return cnt
	
	def getElemCount(self):
		return len(self.bbSize)

	def getRawRecordBitCount(self):
		return self.getAxisCount() * 4 * 8

	def isSparse(self):
		return self.getAxisCount() != self.getElemCount()

	def quantize(self, v):
		return [limF32((v[i] - self.bbMin[i]) * self.bbScl[i]) for i in xrange(len(v))]

	def encodeVec(self, v, mcut):
		qv = self.quantize(v)
		if self.isSparse():
			msk = self.getAxisMask()
			sqv = []
			for i in xrange(self.getElemCount()):
				if msk & (1 << i): sqv.append(qv[i])
			qv = sqv
		return encodeQuantizedVec(qv, mcut)

	def encode(self):
		self.nbits = 0
		self.bits = 0
		self.bitOrgFrm = []
		self.nbitsFrm = []
		mcut = self.getFractionCut()
		for v in self.data:
			self.bitOrgFrm.append(self.nbits)
			(bits, cnt) = self.encodeVec(v, mcut)
			self.bits |= bits << self.nbits
			self.nbits += cnt
			self.nbitsFrm.append(cnt)

	def getFrmBits(self, fno):
		org = self.bitOrgFrm[fno]
		cnt = self.nbitsFrm[fno]
		bits = (self.bits >> org) & ((1 << cnt) - 1)
		return (bits, cnt)

	def getFractionCut(self): return 0

	def initEncodingInfo(self, poseList):
		self.baseId = poseList.vlst.add(self.bbMin)
		if not self.isConstant():
			self.sizeId = poseList.vlst.add(self.bbSize)
			self.bitInfo = self.getFractionCut()

def getOctaBitsAligned(lvl):
	tbl = [
		(21, 22), # 21.21.22 64-bit
		(18, 20), # 18.18.20 56-bit
		(16, 16), # 16.16.16 48-bit
		(13, 14), # 13.13.14 40-bit
		(10, 12)  # 10.10.12 32-bit
	]
	if lvl < 0: lvl = 0
	if lvl >= len(tbl): lvl = len(tbl)-1
	return tbl[lvl]

def getOctaBits(lvl):
	if lvl < 0: lvl = 0
	if lvl >= 4: lvl = 3
	return (18-lvl, 19-lvl)

class QuatTrack(BaseTrack):
	def __init__(self, data, lvl = 0, useOcta = False):
		self.lvl = lvl
		self.useOcta = useOcta
		(self.axisBits, self.angleBits) = getOctaBits(lvl)
		BaseTrack.__init__(self, data)

	def isOcta(self):
		if self.isConstant(): return False
		return self.useOcta and not self.isSparse()

	def encodeVec(self, v, mcut):
		if self.isOcta():
			return (octaEncodeQuat(v, self.axisBits, self.angleBits), self.axisBits*2 + self.angleBits)
		return BaseTrack.encodeVec(self, v, mcut)

	def getFractionCut(self):
		#return 0 # lossless
		tbl = [0, 1, 2, 2, 4, 4, 4, 4]
		lvl = self.lvl
		if lvl < 0: lvl = 0
		if lvl >= len(tbl): lvl = len(tbl)-1
		return tbl[lvl]

	def initEncodingInfo(self, poseList):
		if self.isOcta():
			self.bitInfo = self.axisBits | (self.angleBits << 8)
		else:
			BaseTrack.initEncodingInfo(self, poseList)

class VecTrack(BaseTrack):
	def __init__(self, data, lvl = 0):
		self.lvl = lvl
		BaseTrack.__init__(self, data)

class PoseNode:
	def __init__(self, name):
		self.name = name
		self.rotTrk = None
		self.posTrk = None
		self.sclTrk = None
		self.xformOrd = XORDER.SRT
		self.rotOrd = RORDER.XYZ

	def setXformOrdStr(self, s):
		self.xformOrd = xformOrdFromStr(s)

	def setRotOrdStr(self, s):
		self.rotOrd = rotOrdFromStr(s)

	def addRot(self, data, lvl = 0, useOcta = False): self.rotTrk = QuatTrack(data, lvl, useOcta)
	def addPos(self, data, lvl = 0): self.posTrk = VecTrack(data, lvl)
	def addScl(self, data, lvl = 0): self.sclTrk = VecTrack(data, lvl)

	def hasRot(self): return self.rotTrk != None
	def hasPos(self): return self.posTrk != None
	def hasScl(self): return self.sclTrk != None

	def getFrameBits(self, fno):
		bits = 0
		n = 0
		if self.hasRot() and not self.rotTrk.isConstant():
			(frBits, frCnt) = self.rotTrk.getFrmBits(fno)
			bits |= frBits << n
			n += frCnt
		if self.hasPos() and not self.posTrk.isConstant():
			(frBits, frCnt) = self.posTrk.getFrmBits(fno)
			bits |= frBits << n
			n += frCnt
		if self.hasScl() and not self.sclTrk.isConstant():
			(frBits, frCnt) = self.sclTrk.getFrmBits(fno)
			bits |= frBits << n
			n += frCnt
		return (bits, n)

	def getRawFrameBitCount(self, fno):
		n = 0
		if self.hasRot() and not self.rotTrk.isConstant():
			n += self.rotTrk.getRawRecordBitCount()
		if self.hasPos() and not self.posTrk.isConstant():
			n += self.posTrk.getRawRecordBitCount()
		if self.hasScl() and not self.sclTrk.isConstant():
			n += self.sclTrk.getRawRecordBitCount()
		return n

	def getNumPoses(self):
		nrot = 0
		if self.hasRot(): nrot = self.rotTrk.getRawFramesNum()
		npos = 0
		if self.hasPos(): npos = self.posTrk.getRawFramesNum()
		nscl = 0
		if self.hasScl(): nscl = self.sclTrk.getRawFramesNum()
		return max(nscl, max(npos, nrot))

	def getTrackMask(self):
		m = 0
		if self.hasRot(): m |= 1
		if self.hasPos(): m |= 2
		if self.hasScl(): m |= 4
		return m

	def getRotAxisMask(self):
		if self.hasRot(): return self.rotTrk.getAxisMask()
		return 0
	def getRotBitInfo(self):
		if self.hasRot(): return self.rotTrk.bitInfo
		return 0
	def getRotBaseId(self):
		if self.hasRot(): return self.rotTrk.baseId
		return -1
	def getRotSizeId(self):
		if self.hasRot(): return self.rotTrk.sizeId
		return -1

	def getPosAxisMask(self):
		if self.hasPos(): return self.posTrk.getAxisMask()
		return 0
	def getPosBitInfo(self):
		if self.hasPos(): return self.posTrk.bitInfo
		return 0
	def getPosBaseId(self):
		if self.hasPos(): return self.posTrk.baseId
		return -1
	def getPosSizeId(self):
		if self.hasPos(): return self.posTrk.sizeId
		return -1

	def getSclAxisMask(self):
		if self.hasScl(): return self.sclTrk.getAxisMask()
		return 0
	def getSclBitInfo(self):
		if self.hasScl(): return self.sclTrk.bitInfo
		return 0
	def getSclBaseId(self):
		if self.hasScl(): return self.sclTrk.baseId
		return -1
	def getSclSizeId(self):
		if self.hasScl(): return self.sclTrk.sizeId
		return -1

	def initEncodingInfo(self, poseList):
		if self.hasRot(): self.rotTrk.initEncodingInfo(poseList)
		if self.hasPos(): self.posTrk.initEncodingInfo(poseList)
		if self.hasScl(): self.sclTrk.initEncodingInfo(poseList)

	def writeInfo(self, out):
		out.writeU32(0) # +00 -> name
		out.writeU8(self.xformOrd) # +04
		out.writeU8(self.rotOrd) # +05
		out.writeU16(self.getRotBitInfo()) # +06
		out.writeU16(self.getPosBitInfo()) # +08
		out.writeU16(self.getSclBitInfo()) # +0A
		out.writeI16(self.getRotBaseId()) # +0C
		out.writeI16(self.getRotSizeId()) # +0E
		out.writeI16(self.getPosBaseId()) # +10
		out.writeI16(self.getPosSizeId()) # +12
		out.writeI16(self.getSclBaseId()) # +14
		out.writeI16(self.getSclSizeId()) # +16
		out.writeU8(self.getRotAxisMask()) # +18
		out.writeU8(self.getPosAxisMask()) # +19
		out.writeU8(self.getSclAxisMask()) # +1A
		out.writeU8(self.getTrackMask()) # +1B

class PoseList:
	def __init__(self):
		self.nodes = []
		self.nbits = 0
		self.vlst = VecList()

	def addNode(self, node):
		node.initEncodingInfo(self)
		self.nodes.append(node)

	def getNumNodes(self):
		if self.nodes: return len(self.nodes)
		return 0

	def getNumPoses(self):
		if self.getNumNodes() > 0:
			return self.nodes[0].getNumPoses()
		return 0

	def encode(self):
		self.bits = 0
		self.nbits = 0
		nposes = self.getNumPoses()
		self.poseBitOrg = []
		self.poseBitLen = []
		self.rawBitCnt = 0
		for fno in xrange(nposes):
			poseOrg = self.nbits
			self.poseBitOrg.append(poseOrg)
			for node in self.nodes:
				(bits, n) = node.getFrameBits(fno)
				self.bits |= bits << self.nbits
				self.nbits += n
				self.rawBitCnt += node.getRawFrameBitCount(fno)
			self.poseBitLen.append(self.nbits - poseOrg)
		if True:
			print "raw bytes:", ceilDiv(self.rawBitCnt, 8)
			print "enc bytes:", ceilDiv(self.nbits, 8)
			pct = div0(float(self.nbits), (self.rawBitCnt / 100.0))
			print str(pct)+"%", 100-pct, "saving"

	def write(self, out):
		nnodes = self.getNumNodes()
		nposes = self.getNumPoses()
		nvec = self.vlst.getCount()
		# header
		out.writeFOURCC("PSQ") # +00
		dataSizePos = out.getPos()
		out.writeU32(0) # +04 data size
		out.writeU16(nnodes) # +08
		out.writeU16(nposes) # +0A
		out.writeU16(nvec) # +0C
		out.writeU16(0xFFFF) # +0E reserved
		nodeLstPos = out.getPos()
		out.writeU32(0) # +10 -> nodes
		poseSizePos = out.getPos()
		out.writeU32(0) # +14 -> pose size list
		vecLstPos = out.getPos()
		out.writeU32(0) # +18 -> vec list
		out.writeU32(self.nbits) # +1C

		if nvec > 0:
			out.align(0x10)
			out.patchCur(vecLstPos)
			self.vlst.write(out)

		out.patchCur(nodeLstPos)
		nodeInfoTop = []
		for node in self.nodes:
			nodeInfoTop.append(out.getPos())
			node.writeInfo(out)

		if (self.nbits):
			out.patchCur(poseSizePos)
			for i in xrange(1, nposes):
				out.writeU32(self.poseBitOrg[i] - self.poseBitOrg[i-1])
			out.writeU32(self.nbits - self.poseBitOrg[nposes-1])
			nbytes = ceilDiv(self.nbits, 8)
			out.writeBytes(self.bits, nbytes)

		for i, node in enumerate(self.nodes):
			out.patchCur(nodeInfoTop[i])
			out.writeU16(strHash16(node.name))
			out.writeStr(node.name)
		out.patchCur(dataSizePos)

	def save(self, path):
		out = BinWriter()
		out.open(path)
		self.write(out)
		out.close()
