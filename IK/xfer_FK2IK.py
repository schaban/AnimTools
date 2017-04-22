import sys
import hou
import os
import imp
import inspect
from math import *

FK_rot_nodes = [
	"j_Pelvis",
	"j_Spine",
	"j_Chest",
	"j_Neck",
	"j_Head",
	"j_Clavicle_R",
	"j_Clavicle_L"
]

ANIM_base = "/obj/ANIM/"

class Chan:
	def __init__(self, chPath, minFrame, maxFrame):
		self.path = chPath
		self.data = []
		prm = hou.parm(chPath)
		for fno in xrange(minFrame, maxFrame+1):
			val = prm.evalAtFrame(fno)
			self.data.append(val)

def add_rot_chans(lst, fkName, ikName, basePath = ANIM_base):
	nodePath = basePath + fkName
	for rch in ["rx", "ry", "rz"]:
		chPath = nodePath + "/" + rch
		ch = Chan(chPath, minFrame, maxFrame)
		ch.path = ANIM_base + ikName + "/" + rch
		lst.append(ch)

def add_tns_chans(lst, fkName, ikName, basePath = ANIM_base):
	nodePath = basePath + fkName
	for rch in ["tx", "ty", "tz"]:
		chPath = nodePath + "/" + rch
		ch = Chan(chPath, minFrame, maxFrame)
		ch.path = ANIM_base + ikName + "/" + rch
		lst.append(ch)


def IK_xfer(outPath, minFrame, maxFrame, revFoot = False):
	chLst = []
	for nodeName in FK_rot_nodes:
		add_rot_chans(chLst, nodeName, nodeName)
	if not revFoot:
		add_rot_chans(chLst, "j_Toe_R", "j_Toe_R")
		add_rot_chans(chLst, "j_Toe_L", "j_Toe_L")

	add_tns_chans(chLst, "n_Move", "n_Move")
	add_tns_chans(chLst, "n_Center", "n_Center")

	# arms

	add_rot_chans(chLst, "j_Shoulder_R", "c_Shoulder_R")
	add_rot_chans(chLst, "j_Shoulder_L", "c_Shoulder_L")

	add_rot_chans(chLst, "WristEff_R", "c_Wrist_R", "/obj/Limb_capt/")
	add_rot_chans(chLst, "WristEff_L", "c_Wrist_L", "/obj/Limb_capt/")

	add_tns_chans(chLst, "WristEff_R", "c_Wrist_R", "/obj/Limb_capt/")
	add_tns_chans(chLst, "WristEff_L", "c_Wrist_L", "/obj/Limb_capt/")


	# legs

	add_rot_chans(chLst, "j_Hip_R", "c_Hip_R")
	add_rot_chans(chLst, "j_Hip_L", "c_Hip_L")

	if revFoot:
		add_rot_chans(chLst, "AnkleEff_R", "c_Foot_R", "/obj/Limb_capt/")
		add_rot_chans(chLst, "AnkleEff_L", "c_Foot_L", "/obj/Limb_capt/")

		add_rot_chans(chLst, "ToeEff_R", "c_RevFoot_R", "/obj/Limb_capt/")
		add_rot_chans(chLst, "ToeEff_L", "c_RevFoot_L", "/obj/Limb_capt/")

		add_tns_chans(chLst, "ToeEff_R", "c_RevFoot_R", "/obj/Limb_capt/")
		add_tns_chans(chLst, "ToeEff_L", "c_RevFoot_L", "/obj/Limb_capt/")
	else:
		add_rot_chans(chLst, "AnkleEff_R", "c_Ankle_R", "/obj/Limb_capt/")
		add_rot_chans(chLst, "AnkleEff_L", "c_Ankle_L", "/obj/Limb_capt/")

		add_tns_chans(chLst, "AnkleEff_R", "c_Ankle_R", "/obj/Limb_capt/")
		add_tns_chans(chLst, "AnkleEff_L", "c_Ankle_L", "/obj/Limb_capt/")

	f = open(outPath, "w")
	if not f: return
	f.write("chgadd MOT\n")
	for ch in chLst:
		f.write("chgop MOT add " + ch.path + "\n")
	for ch in chLst:
		name = ch.path
		sep = ch.path.rfind("/")
		if sep >= 0: name = ch.path[sep+1:]
		path = ""
		if sep >= 0: path = ch.path[:sep]
		f.write("chadd " + path + " " + name + " \n")
	nfrm = maxFrame - minFrame + 1
	f.write("frange 0 " + str(maxFrame - minFrame) + "\n")
	f.write("chblockbegin\n")
	for ch in chLst:
		f.write("# " + ch.path + "\n")
		for fno in xrange(nfrm):
			f.write("chkey -f " + str(fno) + " -v " + str(ch.data[fno]) + " -F 'linear()' " + ch.path + "\n")
	f.write("chblockend\n")
	f.close()

if __name__=="__main__":
	exePath = os.path.dirname(os.path.abspath(inspect.getframeinfo(inspect.currentframe()).filename))
	outPath = exePath + "/ik.hs"
	frangeVals = hou.hscript("frange")[0].split()
	minFrame = int(frangeVals[2])
	maxFrame = int(frangeVals[4])
	print "frange:", minFrame, maxFrame
	print outPath
	IK_xfer(outPath, minFrame, maxFrame, revFoot = True)
