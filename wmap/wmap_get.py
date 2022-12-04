node = hou.pwd()
geo = node.geometry()
skinAttr = geo.findPointAttrib("boneCapture")
jlst = []
tbl = skinAttr.indexPairPropertyTables()[0]
n = tbl.numIndices()
for i in range(n):
	jname = tbl.stringPropertyValueAtIndex("pCaptPath", i)
	jname = jname.split("/cregion")[0]
	jlst.append(jname)


pts = geo.points()
for ipnt, pnt in enumerate(pts):
	skin = pnt.floatListAttribValue(skinAttr)
	nwgt = len(skin)//2
	iw = []
	for i in range(nwgt):
		idx = int(skin[i*2])
		if idx >= 0:
			wgt = skin[i*2 + 1]
			if wgt > 0: iw.append([idx, wgt])
	iw.sort(key = lambda iw: -iw[1])
	for w in iw:
		idx = w[0]
		wgt = w[1]
		jname = jlst[idx]
		wmName = "wmap_" + jname
		wmAttr = geo.findPointAttrib(wmName)
		if not wmAttr: wmAttr = geo.addAttrib(hou.attribType.Point, wmName, 0.0)
		pnt.setAttribValue(wmAttr, wgt)
