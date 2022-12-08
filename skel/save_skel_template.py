import sys
import hou
import os
import imp
import re
import random
import inspect
from math import *
from array import array

exePath = os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))

def writeVec(f, vec):
	for val in vec:
		f.write(" ")
		f.write(str(val))

def saveSkelInfo(skelNode, outPath):
	if not skelNode: return
	if not outPath: return
	f = open(outPath, "w")
	if not f: return
	for node in skelNode.children():
		f.write(node.name())
		parentName = "."
		inp = node.inputConnectors()[0]
		if len(inp): parentName = inp[0].inputNode().name()
		f.write(" " + parentName)
		writeVec(f, node.parmTuple("t").eval())
		writeVec(f, node.parmTuple("r").eval())
		f.write("\n")
	f.close()

if __name__=="__main__":
	skelTmplNode = None
	for node in hou.node("/obj").children():
		if node.name().endswith("_skel_template"):
			skelTmplNode = node
			break
saveSkelInfo(skelTmplNode, exePath + "/_skel_.txt")
