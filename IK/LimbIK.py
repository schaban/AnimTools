# Author: Sergey Chaban <sergey.chaban@gmail.com>

import sys
import hou
from math import *
from array import array
from hou import Vector3, Vector4, Matrix4

def setAttr(geo, name, val):
	attr = geo.findGlobalAttrib(name)
	if attr:
		geo.setGlobalAttribValue(attr, val)
	else:
		geo.addAttrib(hou.attribType.Global, name, val)

def calcVec(v, m):
	return Vector3(Vector4(tuple(v) + (0,)) * m)

def calcPnt(v, m):
	return Vector3(Vector4(tuple(v) + (1,)) * m)

def mkBasis(vx, vy, vz):
	return Matrix4( ( (tuple(vx) + (0,)),  (tuple(vy) + (0,)),  (tuple(vz) + (0,)), (0, 0, 0, 1) ) )

def setTranslate(m, v):
	m.setAt(3, 0, v[0])
	m.setAt(3, 1, v[1])
	m.setAt(3, 2, v[2])

def getTranslate(m):
	return Vector3(m.at(3, 0), m.at(3, 1), m.at(3, 2))

def getNodeRotMtx(node):
	rot = node.evalParmTuple("r")
	ord = node.parm("rOrd").evalAsString()
	return hou.hmath.buildRotate(rot, ord)

def orientZX(vz, vx):
	vx = vx.normalized()
	vz = vz.normalized()
	vy = vz.cross(vx).normalized()
	vx = vy.cross(vz).normalized()
	return mkBasis(vx, vy, vz)

def orientZY(vz, vy):
	vz = vz.normalized()
	vy = vy.normalized()
	vx = vy.cross(vz).normalized()
	vy = vz.cross(vx).normalized()
	return mkBasis(vx, vy, vz)

def limitAng(ang):
	ang = ang % 360.0
	if abs(ang) > 180.0:
		if ang < 0.0:
			ang += 360.0
		else:
			ang -= 360.0
	return ang

def solve(geo, rootNode, parentNode, topNode, effNode, rotNode, endNode, axisKind, upKind, extCompensate = False):
	rootMtx = rootNode.worldTransform()
	parentMtx = parentNode.worldTransform()
	topMtx = topNode.worldTransform()
	topPos = getTranslate(topMtx)

	extNode = None
	effInp = effNode.inputConnectors()[0]
	if len(effInp) > 0:
		effParentNode = effInp[0].inputNode()
		if effParentNode != rootNode:
			extNode = effParentNode
			extMtx = extNode.parmTransform()
			extOffs = calcVec(effNode.evalParmTuple("t"), extMtx)
			extMtx = effNode.parmTransform() * rootMtx
			extPos = extNode.origin()
			setTranslate(extMtx, extPos)

	if extNode:
		effPos = calcPnt(extOffs, extMtx)
	else:
		effMtx = effNode.worldTransform()
		effPos = getTranslate(effMtx)
	dist = topPos.distanceTo(effPos)
	len0 = hou.Vector3(rotNode.evalParmTuple("t")).length()
	len1 = hou.Vector3(endNode.evalParmTuple("t")).length()
	rot0 = 0
	rot1 = 0
	if (dist < len0+len1):
		c0 = ( (len0*len0) - (len1*len1) + (dist*dist) ) / (2*len0*dist)
		c1 = ( (len0*len0) + (len1*len1) - (dist*dist) ) / (2*len0*len1)
		c0 = hou.hmath.clamp(c0, -1, 1)
		c1 = hou.hmath.clamp(c1, -1, 1)
		rot0 = -acos(c0)
		rot1 = pi - acos(c1)

	axisLst = [
		Vector3(1, 0, 0),
		Vector3(-1, 0, 0),
		Vector3(0, 1, 0),
		Vector3(0, -1, 0),
		Vector3(0, 0, 1),
		Vector3(0, 0, -1),
	]
	ikAxis = axisLst[axisKind]
	ikUp = axisLst[upKind]
	zyMtx = orientZY(ikAxis, ikUp)
	ikSide = calcVec(Vector3(1, 0, 0), zyMtx)

	axisX = calcVec(ikSide, topMtx)
	axisZ = effPos - topPos
	zxMtx = orientZX(axisZ, axisX)

	ikMtx = zyMtx.transposed() * zxMtx
	rotMtx = hou.hmath.buildRotateAboutAxis(ikSide, hou.hmath.radToDeg(rot0))

	topMtx = rotMtx * ikMtx
	setTranslate(topMtx, topPos)

	rotPos = hou.Vector3(rotNode.evalParmTuple("t")) * topMtx
	rotMtx = hou.hmath.buildRotateAboutAxis(ikSide, hou.hmath.radToDeg(rot1))
	rotMtx = rotMtx * topMtx
	setTranslate(rotMtx, rotPos)

	topLocal = topMtx * parentMtx.inverted()
	topAng = topLocal.extractRotates()
	for i in xrange(3): topAng[i] = limitAng(topAng[i])

	rotLocal = rotMtx * topMtx.inverted()
	rotAng = rotLocal.extractRotates()
	for i in xrange(3): rotAng[i] = limitAng(rotAng[i])

	setAttr(geo, "top", topAng)
	setAttr(geo, "rot", rotAng)

	if extNode:
		endMtx = extNode.parmTransform() * getNodeRotMtx(effNode) * rootMtx
	else:
		endMtx = effNode.parmTransform() * rootMtx
	setTranslate(endMtx, effPos)
	endLocal = endMtx * rotMtx.inverted()
	endAng = endLocal.extractRotates()
	for i in xrange(3): endAng[i] = limitAng(endAng[i])
	setAttr(geo, "end", endAng)

	if extNode:
		if extCompensate:
			extLocal = extMtx * endMtx.inverted()
		else:
			extLocal = effParentNode.parmTransform() * rootMtx * endMtx.inverted()
		extAng = extLocal.extractRotates()
		for i in xrange(3): extAng[i] = limitAng(extAng[i])
		setAttr(geo, "ext", extAng)
		#setAttr(geo, "offs", extOffs)

