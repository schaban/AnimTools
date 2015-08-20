# Houdini animation utils for Pose Sequence library
# Author: Sergey Chaban <sergey.chaban@gmail.com>

import sys
import hou

def qget(r):
	return (hou.Quaternion(r[0], (1, 0, 0)), hou.Quaternion(r[1], (0, 1, 0)), hou.Quaternion(r[2], (0, 0, 1)))

def qord(rord):
	id = 0
	for i in xrange(3): id += (ord(rord[i]) - ord('x')) << (i*2)
	return (id / 3) - 2

def qrot(rot, rord):
	tbl = [
		lambda qx, qy, qz: qx * qy * qz,
		lambda qx, qy, qz: qx * qz * qy,
		None,
		None,
		lambda qx, qy, qz: qy * qx * qz,
		None,
		lambda qx, qy, qz: qy * qz * qx,
		None,
		None,
		lambda qx, qy, qz: qz * qx * qy,
		lambda qx, qy, qz: qz * qy * qx
	]
	(qx, qy, qz) = qget(rot)
	return tbl[qord(rord)](qx, qy, qz)

def getMinFrame():
	return int(hou.hscript("frange")[0].split()[2])

def getMaxFrame():
	return int(hou.hscript("frange")[0].split()[4])

def getRotOrd(node):
	rOrd = "xyz"
	rOrdParm = node.parm("rOrd")
	if rOrdParm: rOrd = rOrdParm.evalAsString()
	return rOrd

def evalFrmRot(node, fno):
	rOrd = getRotOrd(node)
	parms = hou.parmTuple(node.path() + "/r")
	return qrot(parms.evalAtFrame(fno), rOrd)

def evalFrmPos(node, fno):
	parms = hou.parmTuple(node.path() + "/t")
	return hou.Vector3(parms.evalAtFrame(fno))

def evalFrmScl(node, fno):
	parms = hou.parmTuple(node.path() + "/s")
	return hou.Vector3(parms.evalAtFrame(fno))

def getKeys(parm):
	keyframes = None
	trk = parm.overrideTrack()
	if trk:
		keyframes = parm.getReferencedParm().keyframes()
	else:
		keyframes = parm.keyframes()
	return keyframes

def mkNode(basePath, nodeName):
	node = hou.node(basePath + "/" + nodeName)
	if not node: node = hou.node(basePath).createNode("null", nodeName)
	return node

def ckParmsAnim(parms):
	for parm in parms:
		keys = getKeys(parm)
		if keys: return True
	return False

def hasRotAnim(node):
	parms = hou.parmTuple(node.path() + "/r")
	return ckParmsAnim(parms)

def hasPosAnim(node):
	parms = hou.parmTuple(node.path() + "/t")
	return ckParmsAnim(parms)

def hasSclAnim(node):
	parms = hou.parmTuple(node.path() + "/s")
	return ckParmsAnim(parms)

class BaseTrack:
	def __init__(self, node, minFrame, maxFrame, trkName):
		self.node = node
		self.minFrame = minFrame
		self.maxFrame = maxFrame
		self.trkName = trkName
		self.getData()

	def getData(self): pass

class RotTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "r")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			q = evalFrmRot(self.node, fno)
			self.data.append(q)

class PosTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "t")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			v = evalFrmPos(self.node, fno)
			self.data.append(v)

class SclTrack(BaseTrack):
	def __init__(self, node, minFrame, maxFrame):
		BaseTrack.__init__(self, node, minFrame, maxFrame, "s")

	def getData(self):
		self.data = []
		for fno in xrange(self.minFrame, self.maxFrame+1):
			v = evalFrmScl(self.node, fno)
			self.data.append(v)

class AnimNode:
	def __init__(self, hrc, hnode, lvl):
		self.hrc = hrc
		self.hnode = hnode
		self.lvl = lvl
		self.rotTrk = None
		self.posTrk = None
		self.sclTrk = None
		self.rOrd = "xyz"
		rOrdParm = hnode.parm("rOrd")
		if rOrdParm: self.rOrd = rOrdParm.evalAsString()
		self.xOrd = "srt"
		xOrdParm = hnode.parm("xOrd")
		if xOrdParm: self.xOrd = xOrdParm.evalAsString()
		if hasRotAnim(self.hnode):
			self.rotTrk = RotTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)
		if hasPosAnim(self.hnode):
			self.posTrk = PosTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)
		if hasSclAnim(self.hnode):
			self.sclTrk = SclTrack(self.hnode, self.hrc.minFrame, self.hrc.maxFrame)

	def getName(self):
		if self.hnode: return self.hnode.name()
		return "<???>"

	def hasRot(self): return self.rotTrk != None
	def hasPos(self): return self.posTrk != None
	def hasScl(self): return self.sclTrk != None
	def isAnimated(self):
		return self.hasRot() or self.hasPos() or self.hasScl()

	def getRotData(self):
		if self.hasRot(): return self.rotTrk.data
		return None

	def getPosData(self):
		if self.hasPos(): return self.posTrk.data
		return None

	def getSclData(self):
		if self.hasScl(): return self.sclTrk.data
		return None

class AnimHrc:
	def __init__(self, rootPath, minFrame, maxFrame):
		self.FPS = hou.fps()
		self.minFrame = minFrame
		self.maxFrame = maxFrame
		self.nodes = []
		self.build(hou.node(rootPath), 0)
		self.maxLvl = 0
		for node in self.nodes:
			self.maxLvl = max(self.maxLvl, node.lvl)

	def build(self, hnode, lvl):
		self.nodes.append(AnimNode(self, hnode, lvl))
		for link in hnode.outputConnectors()[0]:
			self.build(link.outputNode(), lvl+1)
