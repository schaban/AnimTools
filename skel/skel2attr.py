class SkelNode:
	def __init__(self, hnode):
		self.hnode = hnode
		self.name = hnode.name()
		self.pos = hnode.parmTuple("t").eval()
		self.rot = hnode.parmTuple("r").eval()
		self.scl = hou.Vector3(hnode.parmTuple("s").eval())
		self.scl *= hnode.parm("scale").eval()
		parentName = ""
		inp = hnode.inputConnectors()[0]
		if len(inp): parentName = inp[0].inputNode().name()
		self.parent = parentName

class SkelList:
	def __init__(self):
		self.nodes = []

	def build(self, rootPath):
		self.hrcBuild(hou.node(rootPath))

	def hrcBuild(self, hnode):
		self.nodes.append(SkelNode(hnode))
		for link in hnode.outputConnectors()[0]:
			self.hrcBuild(link.outputNode())

geo = hou.pwd().geometry()

nameAttr = geo.addAttrib(hou.attribType.Point, "name", "")
parentAttr = geo.addAttrib(hou.attribType.Point, "parent", "")
rotAttr = geo.addAttrib(hou.attribType.Point, "rot", [0.0, 0.0, 0.0])
sclAttr = geo.addAttrib(hou.attribType.Point, "scl", [0.0, 0.0, 0.0])

lst = SkelList()
lst.build("../../REST/root")
for node in lst.nodes:
	pnt = geo.createPoint()
	pnt.setPosition(node.pos)
	pnt.setAttribValue(rotAttr, node.rot)
	pnt.setAttribValue(sclAttr, node.scl)
	pnt.setAttribValue(nameAttr, node.name)
	pnt.setAttribValue(parentAttr, node.parent)
