import sys
import hou
import os
import imp
import re
import inspect
from math import *
from array import array

def getRotLvl(hrcLvl):
	tbl = [0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7]
	if hrcLvl < 0: hrcLvl = 0
	if hrcLvl >= len(tbl): hrcLvl = len(tbl)-1
	return tbl[hrcLvl]

path = os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))

libName = "psq"
libFile, libFname, libDescr = imp.find_module(libName, [path])
#print libFname
psqMod = imp.load_module(libName, libFile, libFname, libDescr)
import psq

libName = "hanim"
libFile, libFname, libDescr = imp.find_module(libName, [path])
#print libFname
hanimMod = imp.load_module(libName, libFile, libFname, libDescr)
import hanim


minFrame = hanim.getMinFrame()
maxFrame = hanim.getMaxFrame()
print "Frame range:", minFrame, maxFrame

rootPath = "/obj/ANIM/root"
if not hou.node(rootPath): rootPath = "/obj/root"
print rootPath

hrc = hanim.AnimHrc(rootPath, minFrame, maxFrame)
encBits = 0
rawBits = 0
poses = psq.PoseList()
octaFlg = not True
for node in hrc.nodes:
	if node.isAnimated():
		poseNode = psq.PoseNode(node.getName())
		poseNode.setXformOrdStr(node.xOrd)
		poseNode.setRotOrdStr(node.rOrd)
		rlvl = getRotLvl(node.lvl)
		if node.hasRot(): poseNode.addRot(node.getRotData(), rlvl, octaFlg)
		if node.hasPos(): poseNode.addPos(node.getPosData())
		if node.hasScl(): poseNode.addScl(node.getSclData())
		poses.addNode(poseNode)

outName = path + "/" + "__test.psq"
print outName
poses.encode()
poses.save(outName)


