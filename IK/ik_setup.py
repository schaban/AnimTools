def ckSolver(name = "LimbIK"):
	lst = hou.hda.loadedFiles()
	flg = False
	hdaName = name + ".hda"
	for path in lst:
		if path.endswith(hdaName):
			flg = True
			break
	if not flg:
		print "Installing IK solver: " + name
		hou.hda.installFile("$HIP/"+hdaName)

def setParamExpr(prm, expr):
	if not prm: return
	prm.deleteAllKeyframes()
	key = hou.Keyframe()
	key.setExpression(expr, hou.exprLanguage.Hscript)
	key.setFrame(0)
	prm.setKeyframe(key)

def addChainTop(dstPath, srcPath, name):
	jntNode = hou.node(srcPath + "/j_" + name)
	if not jntNode: return
	if len(jntNode.inputConnectors()[0]) < 1: return
	parentNode = jntNode.inputs()[0]
	dstParentNode = hou.node(dstPath + "/" + parentNode.name())
	if not dstParentNode: return
	locPos = jntNode.parmTuple("t").evalAtTime(0)
	ctlNode = hou.node(dstPath).createNode("null", "c_" + name)
	if not ctlNode: return
	ctlNode.setParms({"tx":locPos[0], "ty":locPos[1], "tz":locPos[2]})
	ctlNode.setFirstInput(dstParentNode)
	ctlNode.setParms({"geoscale":0.3, "controltype":1, "orientation":2, "dcolorr":0.9, "dcolorg":0.02, "dcolorb":0.02})
	ctlNode.setColor(hou.Color([1.0, 0.66, 0.66]))
	if hou.applicationVersion()[0] >= 16: ctlNode.setUserData("nodeshape", "rect")
	return ctlNode

def addChainEnd(dstPath, srcPath, name, rootName = "n_Move"):
	jntNode = hou.node(srcPath + "/j_" + name)
	if not jntNode: return
	if len(jntNode.inputConnectors()[0]) < 1: return
	rootNode = hou.node(srcPath + "/" + rootName)
	if not rootNode: return
	dstRootNode = hou.node(dstPath + "/" + rootNode.name())
	if not dstRootNode: return
	relPos = rootNode.getTransformToNode(jntNode).extractTranslates()
	ctlNode = hou.node(dstPath).createNode("null", "c_" + name)
	if not ctlNode: return
	ctlNode.setParms({"tx":relPos[0], "ty":relPos[1], "tz":relPos[2]})
	ctlNode.setFirstInput(dstRootNode)
	ctlNode.setParms({"geoscale":0.15, "controltype":2, "dcolorr":0.9, "dcolorg":0.02, "dcolorb":0.02})
	ctlNode.setColor(hou.Color([1.0, 0.66, 0.66]))
	if hou.applicationVersion()[0] >= 16: ctlNode.setUserData("nodeshape", "rect")
	return ctlNode

def addIKRotExprCh(node, chIdx, solverPath, attrName):
	chName = "r" + "xyz"[chIdx]
	prm = node.parm(chName)
	if not prm: return
	expr = 'detail("' + solverPath + '", "' + attrName + '", ' + str(chIdx) + ')'
	setParamExpr(prm, expr)

def addIKRotExpr(node, solverPath, attrName):
	if not node: return
	for i in xrange(3): addIKRotExprCh(node, i, solverPath, attrName)

def addSolver(name, side, dstPath, parentName, topName, topDstName, effName, rotName, endName, axisId, upvecId, extName = None, rootName = "n_Move", solverType = "limbik"):
	solverName = name + "_" + side
	sopNode = hou.node(dstPath).createNode("geo", solverName)
	if not sopNode: return
	lst = sopNode.children()
	for sub in lst: sub.destroy()
	ik = sopNode.createNode(solverType)
	if not ik: return
	parentPath = dstPath + "/" + parentName
	topPath = dstPath + "/" + topName + "_" + side
	effPath = dstPath + "/" + effName + "_" + side
	rotPath = dstPath + "/" + rotName + "_" + side
	endPath = dstPath + "/" + endName + "_" + side
	ik.setParms({
		"root": dstPath + "/" + rootName,
		"axis": axisId,
		"upvec": upvecId,
		"parent": parentPath,
		"top": topPath,
		"eff": effPath,
		"rot": rotPath,
		"end": endPath
	})
	addIKRotExpr(hou.node(dstPath + "/" + topDstName + "_" + side), sopNode.path(), "top")
	addIKRotExpr(hou.node(rotPath), sopNode.path(), "rot")
	addIKRotExpr(hou.node(endPath), sopNode.path(), "end")
	if extName:
		extPath = dstPath + "/" + extName + "_" + side
		addIKRotExpr(hou.node(extPath), sopNode.path(), "ext")

	sopNode.setColor(hou.Color([0.75, 0.75, 1.0]))
	netPos = hou.node(endPath).position()
	offsX = 1
	if side == "R": offsX = -offsX
	offsY = -2
	netPos[0] += offsX
	netPos[1] += offsY
	sopNode.setPosition(netPos)
	return sopNode

def addArm(side, dstPath, srcPath):
	topNode = addChainTop(dstPath, srcPath, "Shoulder_" + side)
	endNode = addChainEnd(dstPath, srcPath, "Wrist_" + side)
	refPos = topNode.inputs()[0].position()
	offs = hou.Vector2([3, -1])
	if side == "R": offs[0] = -offs[0]
	topNode.setPosition(refPos + offs)
	offs[1] -= 2
	endNode.setPosition(refPos + offs)
	axisId = 0 # +X
	upvecId = 5 # -Z
	if side == "R":
		axisId = 1 # -X
	ikNode = addSolver("IK_Arm", side, dstPath, "j_Clavicle" + "_" + side, "c_Shoulder", "j_Shoulder", "c_Wrist", "j_Elbow", "j_Wrist", axisId, upvecId)
	return ikNode

def addLeg(side, dstPath, srcPath):
	topNode = addChainTop(dstPath, srcPath, "Hip_" + side)
	endNode = addChainEnd(dstPath, srcPath, "Ankle_" + side)
	endNode.setParms({"geoscale":0.2})
	refPos = topNode.inputs()[0].position()
	offs = hou.Vector2([3, -1])
	if side == "R": offs[0] = -offs[0]
	topNode.setPosition(refPos + offs)
	offs[1] -= 3
	endNode.setPosition(refPos + offs)
	axisId = 3 # -Y
	upvecId = 4 # +Z
	ikNode = addSolver("IK_Leg", side, dstPath, "j_Pelvis", "c_Hip", "j_Hip", "c_Ankle", "j_Knee", "j_Ankle", axisId, upvecId)
	return ikNode

def addFootCtl(dstPath, name, parentNode, locPos, clr):
	ctlNode = hou.node(dstPath).createNode("null", name)
	if not ctlNode: return
	ctlNode.setParms({"tx":locPos[0], "ty":locPos[1], "tz":locPos[2]})
	ctlNode.setFirstInput(parentNode)
	ctlNode.setParms({"geoscale":0.1, "controltype":1, "dcolorr":0.9, "dcolorg":0.02, "dcolorb":0.02})
	ctlNode.setColor(clr)
	if hou.applicationVersion()[0] >= 16: ctlNode.setUserData("nodeshape", "rect")
	return ctlNode

def addLegExt(side, dstPath, srcPath, rootName = "n_Move"):
	rootNode = hou.node(dstPath + "/" + rootName)
	if not rootNode: return
	toeSrcNode = hou.node(srcPath + "/j_Toe_" + side)
	if not toeSrcNode: return
	ankleSrcNode = hou.node(srcPath + "/j_Ankle_" + side)
	if not ankleSrcNode: return
	ankleOffs = toeSrcNode.getTransformToNode(ankleSrcNode).extractTranslates()
	toeNode = hou.node(dstPath + "/j_Toe_" + side)
	if not toeNode: return
	ankleNode = hou.node(dstPath + "/j_Ankle_" + side)
	if not ankleNode: return
	nullClr = hou.Color([1.0, 0.6, 0.6])
	toePos = rootNode.getTransformToNode(toeNode).extractTranslates()
	toeNull = addFootCtl(dstPath, "n_Toe_" + side, rootNode, toePos, nullClr)
	if not toeNull: return
	ankleCtl = addFootCtl(dstPath, "c_Ankle_" + side, toeNull, ankleOffs, nullClr)
	if not ankleCtl: return
	ctlClr = hou.Color([1.0, 0.4, 0.4])
	anklePos = rootNode.getTransformToNode(ankleNode).extractTranslates()
	footCtl = addFootCtl(dstPath, "c_Foot_" + side, rootNode, anklePos, ctlClr)
	if not footCtl: return
	revFootCtl = addFootCtl(dstPath, "c_RevFoot_" + side, rootNode, toePos, ctlClr)
	if not revFootCtl: return

	sideScl = 1
	if side == "R": sideScl = -sideScl
	toeNull.setPosition(toeNode.position() + hou.Vector2([2*sideScl, 0]))
	toeNull.setDisplayFlag(False)
	ankleCtl.setPosition(toeNode.position() + hou.Vector2([2*sideScl, -1]))
	ankleCtl.setDisplayFlag(False)
	footCtl.setPosition(ankleNode.position() + hou.Vector2([3.5*sideScl, 0]))
	footCtl.setParms({"geoscale":0.2, "controltype":1, "orientation":1})
	revFootCtl.setPosition(toeNode.position() + hou.Vector2([4.5*sideScl, 0]))
	revFootCtl.setParms({"geoscale":0.1, "controltype":2})

	for i in xrange(3):
		chName = "t" + "xyz"[i]
		prm = toeNull.parm(chName)
		expr = 'origin("../' + rootName + '", "../' + revFootCtl.name() + '", "T' + "XYZ"[i] +'")'
		setParamExpr(prm, expr)
		chName = "r" + "xyz"[i]
		prm = toeNull.parm(chName)
		expr = 'ch("../' + revFootCtl.name() + '/' + chName + '")'
		setParamExpr(prm, expr)

	for i in xrange(3):
		chName = "r" + "xyz"[i]
		prm = ankleCtl.parm(chName)
		expr = 'ch("../' + footCtl.name() + '/' + chName + '")'
		setParamExpr(prm, expr)

	for i in xrange(3):
		chName = "t" + "xyz"[i]
		prm = footCtl.parm(chName)
		expr = 'origin("../' + rootName + '", "../' + ankleNode.name() + '", "T' + "XYZ"[i] +'")'
		setParamExpr(prm, expr)

	topNode = addChainTop(dstPath, srcPath, "Hip_" + side)
	refPos = topNode.inputs()[0].position()
	topNode.setPosition(refPos + hou.Vector2([3*sideScl, -1]))

	axisId = 3 # -Y
	upvecId = 4 # +Z
	ikNode = addSolver("IK_Leg", side, dstPath, "j_Pelvis", "c_Hip", "j_Hip", "c_Ankle", "j_Knee", "j_Ankle", axisId, upvecId, "j_Toe")
	return ikNode

def addIK(dstPath = "/obj/ANIM", srcPath = "/obj/REST", revFoot = True):
	addArm("L", dstPath, srcPath)
	addArm("R", dstPath, srcPath)

	if revFoot:
		addLegExt("L", dstPath, srcPath)
		addLegExt("R", dstPath, srcPath)
	else:
		addLeg("L", dstPath, srcPath)
		addLeg("R", dstPath, srcPath)


if __name__=="__main__":
	ckSolver()
	addIK()
