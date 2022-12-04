geo = hou.pwd().geometry()

wmapAttrs = []
jntNames = []
wmapPrefix = "wmap_"
for attr in geo.pointAttribs():
	if attr.name().startswith(wmapPrefix):
		wmapAttrs.append(attr)
		jntNames.append(attr.name()[len(wmapPrefix):])

njnt = len(jntNames)

pntWgt = [0.0 for j in range(njnt)]
for pnt in geo.points():
	wsum = 0.0
	for j in range(njnt):
		w = pnt.floatAttribValue(wmapAttrs[j])
		pntWgt[j] = w
		wsum += w
	if wsum != 1.0 and wsum > 0:
		wscl = 1.0 / wsum
		for j in range(njnt):
			pntWgt[j] *= wscl
		for j in range(njnt):
			pnt.setAttribValue(wmapAttrs[j], pntWgt[j])
