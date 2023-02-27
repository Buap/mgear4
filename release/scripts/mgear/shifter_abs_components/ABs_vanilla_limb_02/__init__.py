# MGEAR is under the terms of the MIT License

# Copyright (c) 2016 Jeremie Passerin, Miquel Campos

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#############################################
# GLOBAL
#############################################
# Maya
import pymel.core as pm
import pymel.core.datatypes as dt


# mgear
from mgear.shifter.component import MainComponent

import mgear.core.primitive as pri
import mgear.core.transform as tra
import mgear.core.attribute as att
import mgear.core.node as nod
import mgear.core.vector as vec
import mgear.core.applyop as aop
import mgear.core.fcurve as fcu

#############################################
# COMPONENT
#############################################
class Component(MainComponent):

    def addObjects(self):
        
        self.realNegate = self.negate
        self.negate = False

        self.setup = pri.addTransformFromPos(self.setupWS,  self.getName("WS"))
        att.lockAttribute(self.setup)

        self.WIP = self.options["mode"]

        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        self.length0 = vec.getDistance(self.guide.apos[0], self.guide.apos[1])
        self.length1 = vec.getDistance(self.guide.apos[1], self.guide.apos[2])
        self.length2 = vec.getDistance(self.guide.apos[2], self.guide.apos[3])


        # limb chain
        self.limbBones = pri.add2DChain(self.root, self.getName("limbBones%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        # limb chain FK ref
        self.limbBonesFK = pri.add2DChain(self.root, self.getName("limbFK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        # limb chain IK ref
        self.limbBonesIK = pri.add2DChain(self.root, self.getName("limbIK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
    
        # 1 bone chain for upv ref
        self.limbChainUpvRef= pri.add2DChain(self.root, self.getName("limbUpvRef%s_jnt"), [self.guide.apos[0],self.guide.apos[2]], self.normal, False, self.WIP)
        self.limbChainUpvRef[1].setAttr("jointOrientZ", self.limbChainUpvRef[1].getAttr("jointOrientZ")*-1)

        # mid joints
        self.mid1_jnt =  pri.addJoint(self.limbBones[0], self.getName("mid1_jnt"),self.limbBones[1].getMatrix(worldSpace=True), self.WIP )
        self.mid1_jnt.attr("radius").set(3)
        self.mid1_jnt.setAttr("jointOrient", 0, 0, 0)


        # FK Controlers -----------------------------------
        t = tra.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.realNegate)
        self.fk0_npo = pri.addTransform(self.root, self.getName("fk0_npo"), t)
        self.fk0_ctl = self.addCtl(self.fk0_npo, "fk0_ctl", t, self.color_fk, "cube", w=self.length0, h=self.size*.1, d=self.size*.1, po=dt.Vector(.5*self.length0*self.n_factor,0,0), tp=self.parentCtlTag)
        att.setKeyableAttributes(self.fk0_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"])

        t = tra.getTransformLookingAt(self.guide.apos[1], self.guide.apos[2], self.normal, "xz", self.realNegate)
        self.fk1_npo = pri.addTransform(self.fk0_ctl, self.getName("fk1_npo"), t)
        self.fk1_ctl = self.addCtl(self.fk1_npo, "fk1_ctl", t, self.color_fk, "cube", w=self.length1, h=self.size*.1, d=self.size*.1, po=dt.Vector(.5*self.length1*self.n_factor,0,0), tp=self.fk0_ctl)
        att.setKeyableAttributes(self.fk1_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"])

        t = tra.getTransformLookingAt(self.guide.apos[2], self.guide.apos[3], self.normal, "xz", self.realNegate)
        self.fk2_npo = pri.addTransform(self.fk1_ctl, self.getName("fk2_npo"), t)
        self.fk2_ctl = self.addCtl(self.fk2_npo, "fk2_ctl", t, self.color_fk, "cube", w=self.length2, h=self.size*.1, d=self.size*.1, po=dt.Vector(.5*self.length2*self.n_factor,0,0), tp=self.fk1_ctl)
        att.setKeyableAttributes(self.fk2_ctl)

        self.fk_ctl = [self.fk0_ctl, self.fk1_ctl, self.fk2_ctl]

        for  x in self.fk_ctl:
            att.setInvertMirror(x, ["tx", "ty", "tz"])

        
        # IK Controlers -----------------------------------

        self.ik_cns = pri.addTransformFromPos(self.root, self.getName("ik_cns"), self.guide.pos["tip"])

        self.ikcns_ctl = self.addCtl(self.ik_cns, "ikcns_ctl", tra.getTransformFromPos(self.guide.pos["tip"]), self.color_ik, "null", w=self.size*.12, tp=self.parentCtlTag)
        att.setInvertMirror(self.ikcns_ctl, ["tx", "ty", "tz"])

        # if self.settings["WorldAlign"]:
        if True:
            m = tra.getTransformFromPos(self.guide.pos["tip"])
        else:
            if self.negate:
                m = tra.getTransformLookingAt(self.guide.pos["tip"], self.guide.pos["eff"], self.normal, "x-y", True)
            else:
                m = tra.getTransformLookingAt(self.guide.pos["tip"], self.guide.pos["eff"], self.normal, "xy", False)
        self.ik_ctl = self.addCtl(self.ikcns_ctl, "ik_ctl", m, self.color_ik, "cube", w=self.size*.12, h=self.size*.12, d=self.size*.12, tp=self.ikcns_ctl)
        
        if self.negate:
            m = tra.getTransformLookingAt(self.guide.pos["tip"], self.guide.pos["eff"], self.normal, "x-y", True)
        else:
            m = tra.getTransformLookingAt(self.guide.pos["tip"], self.guide.pos["eff"], self.normal, "xy", False)

        if self.settings["mirrorIK"]:
            if self.negate:
                self.ik_cns.sx.set(-1)
                self.ik_ctl.rz.set(self.ik_ctl.rz.get()*-1)
        else:
            att.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        att.setKeyableAttributes(self.ik_ctl)

        
        self.ik_ctl_ref = pri.addTransform(self.ik_ctl, self.getName("ikCtl_ref"), m)

        self.ik2b_bone_ref = pri.addTransform(self.limbBonesIK[2], self.getName("ik2B_B_ref"), m)
        self.ik2b_blend = pri.addTransform(self.ik_ctl, self.getName("ik2B_blend"), m)

        self.ik2b_ik_npo = pri.addTransform(self.ik2b_blend, self.getName("ik2B_ik_npo"), tra.getTransform(self.limbBonesIK[-1]))
        self.ik2b_ik_ref = pri.addTransform(self.ik2b_ik_npo, self.getName("ik2B_ik_ref"),  tra.getTransform(self.fk_ctl[2]))

        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size*.5
        v += self.guide.apos[1]

        self.upv_cns = pri.addTransformFromPos(self.root, self.getName("upv_cns"), v)

        self.upv_ctl = self.addCtl(self.upv_cns, "upv_ctl", tra.getTransform(self.upv_cns), self.color_ik, "diamond", w=self.size*.12, tp=self.parentCtlTag)

        if self.settings["mirrorMid"]:
            if self.negate:
                self.upv_cns.rz.set(180)
                self.upv_cns.sy.set(-1)
        else:
            att.setInvertMirror(self.upv_ctl, ["tx"])
        att.setKeyableAttributes(self.upv_ctl, self.t_params)



        # References --------------------------------------
        # Calculate  again the transfor for the IK ref. This way align with FK
        trnIK_ref = tra.getTransformLookingAt(self.guide.pos["tip"], self.guide.pos["eff"], self.normal, "xz", self.negate)
        self.ik_ref = pri.addTransform(self.ik_ctl_ref, self.getName("ik_ref"), trnIK_ref)
        self.fk_ref = pri.addTransform(self.fk_ctl[2], self.getName("fk_ref"), trnIK_ref)



        #mid control

        tA = tra.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.negate)
        tA = tra.setMatrixPosition(tA, self.guide.apos[1])
        tB = tra.getTransformLookingAt(self.guide.apos[1], self.guide.apos[2], self.normal, "xz", self.negate)
        t = tra.getInterpolateTransformMatrix(tA, tB)
        self.ctrn_loc = pri.addTransform(self.root, self.getName("ctrn_loc"), t)
        
        # #match IK FK references
        self.match_fk0_off = pri.addTransform(self.root, self.getName("matchFk0_npo"), tra.getTransform(self.fk_ctl[1]))
        self.match_fk0 = pri.addTransform(self.match_fk0_off, self.getName("fk0_mth"), tra.getTransform(self.fk_ctl[0]))
        self.match_fk1_off = pri.addTransform(self.root, self.getName("matchFk1_npo"), tra.getTransform(self.fk_ctl[2]))
        self.match_fk1 = pri.addTransform(self.match_fk1_off, self.getName("fk1_mth"), tra.getTransform(self.fk_ctl[1]))
        self.match_fk2 = pri.addTransform(self.ik_ctl, self.getName("fk2_mth"), tra.getTransform(self.fk_ctl[2]))

        self.match_ik = pri.addTransform(self.fk2_ctl, self.getName("ik_mth"), tra.getTransform(self.ik_ctl))
        self.match_ikUpv = pri.addTransform(self.fk0_ctl, self.getName("upv_mth"), tra.getTransform(self.upv_ctl))




        # Mid Controler ------------------------------------

        t = tra.getTransform(self.ctrn_loc)
        self.mid_cns = pri.addTransform(self.ctrn_loc, self.getName("mid_cns"), t)
        self.mid_ctl = self.addCtl(self.mid_cns, "mid_ctl", t, self.color_ik, "sphere", w=self.size*.2, tp=self.parentCtlTag)
        if self.settings["mirrorMid"]:
            if self.negate:
                self.mid_cns.rz.set(180)
                self.mid_cns.sz.set(-1)
        else:
            att.setInvertMirror(self.mid_ctl, ["tx", "ty", "tz"])
        att.setKeyableAttributes(self.mid_ctl, self.t_params)

        # Soft IK objects 2 Bones chain --------------------------------------------------------------------------------------------
        # t = tra.getTransformLookingAt(self.guide.pos["root"], self.guide.pos["tip"], self.x_axis, "zx", False)
        t = tra.getTransformLookingAt(self.guide.pos["root"], self.guide.pos["tip"], self.normal, "xz", False)
        
        self.aim_tra2 = pri.addTransform(self.root, self.getName("aimSoftIK2"), t)
        
        # t = tra.getTransformFromPos(self.guide.pos["tip"])
        t = tra.setMatrixPosition(t, self.guide.pos["tip"])
        self.tipSoftIK = pri.addTransform(self.aim_tra2, self.getName("tipSoftIK"), t)
        
        self.softblendLoc2 = pri.addTransform(self.root, self.getName("softblendLoc2"), t)
        
        #Roll join ref---------------------------------
        self.tws0_loc = pri.addTransform(self.root, self.getName("tws0_loc"), tra.getTransform(self.fk_ctl[0]))

        self.tws1_npo = pri.addTransform(self.ctrn_loc, self.getName("tws1_npo"), tra.getTransform(self.ctrn_loc))
        self.tws1_loc = pri.addTransform(self.tws1_npo, self.getName("tws1_loc"), tra.getTransform(self.ctrn_loc))

        self.tws1A_npo = pri.addTransform(self.mid_ctl, self.getName("tws1A_npo"), tA)
        self.tws1A_loc = pri.addTransform(self.tws1A_npo, self.getName("tws1A_loc"), tA)
        self.tws1B_npo = pri.addTransform(self.mid_ctl, self.getName("tws1B_npo"), tB)
        self.tws1B_loc = pri.addTransform(self.tws1B_npo, self.getName("tws1B_loc"), tB)


        self.tws2_npo = pri.addTransform(self.root, self.getName("tws2_npo"), tra.getTransform(self.fk_ctl[2]))
        self.tws2_loc = pri.addTransform(self.tws2_npo, self.getName("tws2_loc"), tra.getTransform(self.fk_ctl[2]))

        # Roll twist chain ---------------------------------
        #UpperLimb
        self.upperLimbChainPos = []
        ii = 1.0/(self.settings["div0"]+1)
        i = 0.0
        for p in range(self.settings["div0"]+2):
            self.upperLimbChainPos.append(vec.linearlyInterpolate(self.guide.pos["root"], self.guide.pos["mid"], blend=i))
            i=i+ii

        self.upperLimbTwistChain= pri.add2DChain(self.root, self.getName("upperLimbTwist%s_jnt"), self.upperLimbChainPos, self.normal, False, self.WIP)

        #lowerLimb
        self.lowerLimbChainPos = []
        ii = 1.0/(self.settings["div1"]+1)
        i = 0.0
        for p in range(self.settings["div1"]+2):
            self.lowerLimbChainPos.append(vec.linearlyInterpolate(self.guide.pos["mid"], self.guide.pos["tip"], blend=i))
            i=i+ii

        self.lowerLimbTwistChain= pri.add2DChain(self.root, self.getName("lowerLimbTwist%s_jnt"), self.lowerLimbChainPos, self.normal, False, self.WIP)
        pm.parent(self.lowerLimbTwistChain[0], self.mid_ctl)

        #Hand Aux chain and nonroll
        self.auxChainPos = []
        ii = .5
        i = 0.0
        for p in range(3):
            self.auxChainPos.append(vec.linearlyInterpolate(self.guide.pos["tip"], self.guide.pos["eff"], blend=i))
            i=i+ii
        t = self.root.getMatrix(worldSpace=True)
        self.aux_npo = pri.addTransform(self.root, self.getName("aux_npo"), t)
        self.auxTwistChain = pri.add2DChain(self.aux_npo, self.getName("auxTwist%s_jnt"), self.auxChainPos, self.normal, False, self.WIP)

        #Non Roll join ref ---------------------------------
        self.upperLimbRollRef = pri.add2DChain(self.root, self.getName("upperLimbRollRef%s_jnt"), self.upperLimbChainPos[:2], self.normal, False, self.WIP)


        self.lowerLimbRollRef = pri.add2DChain(self.aux_npo, self.getName("lowerLimbRollRef%s_jnt"), self.auxChainPos[:2], self.normal, False, self.WIP)


        # Divisions ----------------------------------------
        # We have at least one division at the start, the end and one for the mid. + 2 for mid angle control
        self.divisions = self.settings["div0"] + self.settings["div1"] + 3

        self.div_cns = []
        for i in range(self.divisions):

            div_cns = pri.addTransform(self.root, self.getName("div%s_loc" % i))

            self.div_cns.append(div_cns)

            self.jnt_pos.append([div_cns, i, None, True, False])

        # End reference ------------------------------------
        # To help the deformation on the tip
        #self.eff_loc  = pri.addTransform(self.root, self.getName("eff_loc"), tra.getTransform(self.tws2_loc))
        
        self.eff_loc  = pri.addTransform(self.limbBones[2], self.getName("eff_loc"), tra.getTransform(self.fk_ctl[2]))

        self.eff_ref  = pri.addTransform(self.root, self.getName("eff_ref"), tra.getTransform(self.fk_ctl[2]))
        pm.parentConstraint(self.eff_loc,self.eff_ref)

        self.end_ref = pri.addTransform(self.eff_loc, self.getName("end_ref"),  tra.getTransform(self.fk_ctl[2]))
        if self.negate:
            self.end_ref.attr("rz").set(180.0)

        self.jnt_pos.append([self.end_ref, "end", None, True, False])

        # Tangent controls
        t = tra.getInterpolateTransformMatrix(self.fk_ctl[0], self.tws1A_npo, .1)
        self.upperLimbTangentA_loc = pri.addTransform(self.root, self.getName("upperLimbTangentA_loc"), self.fk_ctl[0].getMatrix(worldSpace=True))
        self.upperLimbTangentA_npo = pri.addTransform(self.upperLimbTangentA_loc, self.getName("upperLimbTangentA_npo"), t)
        self.upperLimbTangentA_ctl = self.addCtl(self.upperLimbTangentA_npo, "upperLimbTangentA_ctl", t, self.color_ik, "circle", w=self.size*.2, ro=dt.Vector(0,0,1.570796), tp=self.mid_ctl)
        if self.negate:
            self.upperLimbTangentA_npo.rz.set(180)
            self.upperLimbTangentA_npo.sz.set(-1)
        att.setKeyableAttributes(self.upperLimbTangentA_ctl, self.t_params)

        t = tra.getInterpolateTransformMatrix(self.fk_ctl[0], self.tws1A_npo, .5)
        self.upperLimbTangentB_npo = pri.addTransform(self.tws1A_loc, self.getName("upperLimbTangentB_npo"), t)
        self.upperLimbTangentB_ctl = self.addCtl(self.upperLimbTangentB_npo, "upperLimbTangentB_ctl", t, self.color_ik, "circle", w=self.size*.1, ro=dt.Vector(0,0,1.570796), tp=self.mid_ctl)
        if self.negate:
            self.upperLimbTangentB_npo.rz.set(180)
            self.upperLimbTangentB_npo.sz.set(-1)
        att.setKeyableAttributes(self.upperLimbTangentB_ctl, self.t_params)

        tC = self.tws1B_npo.getMatrix(worldSpace=True)
        tC = tra.setMatrixPosition(tC, self.guide.apos[2])
        t = tra.getInterpolateTransformMatrix(self.tws1B_npo, tC, .5)
        self.lowerLimbTangentA_npo = pri.addTransform(self.tws1B_loc, self.getName("lowerLimbTangentA_npo"), t)
        self.lowerLimbTangentA_ctl = self.addCtl(self.lowerLimbTangentA_npo, "lowerLimbTangentA_ctl", t, self.color_ik, "circle", w=self.size*.1, ro=dt.Vector(0,0,1.570796), tp=self.mid_ctl)
        if self.negate:
            self.lowerLimbTangentA_npo.rz.set(180)
            self.lowerLimbTangentA_npo.sz.set(-1)
        att.setKeyableAttributes(self.lowerLimbTangentA_ctl, self.t_params)

        t = tra.getInterpolateTransformMatrix(self.tws1B_npo, tC, .9)
        self.lowerLimbTangentB_loc = pri.addTransform(self.root, self.getName("lowerLimbTangentB_loc"), tC)
        self.lowerLimbTangentB_npo = pri.addTransform(self.lowerLimbTangentB_loc, self.getName("lowerLimbTangentB_npo"), t)
        self.lowerLimbTangentB_ctl = self.addCtl(self.lowerLimbTangentB_npo, "lowerLimbTangentB_ctl", t, self.color_ik, "circle", w=self.size*.2, ro=dt.Vector(0,0,1.570796), tp=self.mid_ctl)
        if self.negate:
            self.lowerLimbTangentB_npo.rz.set(180)
            self.lowerLimbTangentB_npo.sz.set(-1)
        att.setKeyableAttributes(self.lowerLimbTangentB_ctl, self.t_params)

        t = self.mid_ctl.getMatrix(worldSpace=True)
        self.midTangent_npo = pri.addTransform(self.mid_ctl, self.getName("midTangent_npo"), t)
        self.midTangent_ctl = self.addCtl(self.midTangent_npo, "midTangent_ctl", t, self.color_fk, "circle", w=self.size*.15, ro=dt.Vector(0,0,1.570796), tp=self.mid_ctl)
        if self.negate:
            self.midTangent_npo.rz.set(180)
            self.midTangent_npo.sz.set(-1)
        att.setKeyableAttributes(self.midTangent_ctl, self.t_params)


    def addAttributes(self):
        
        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam("blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1)
        self.roll_att = self.addAnimParam("roll", "Roll", "double", 0, -180, 180)

        # self.scale_att = self.addAnimParam("ikscale", "Scale", "double", 1, .001, 99)
        self.soft_attr = self.addAnimParam("softIKRange", "Soft IK Range", "double", 0.0001, 0.0001,100)
        self.softSpeed_attr = self.addAnimParam("softIKSpeed", "Soft IK Speed", "double", 2.5,1.001,10)
        self.stretch_attr = self.addAnimParam("stretch", "Stretch", "double", 0,0,1)

        # self.reverse_att = self.addAnimParam("reverse", "Reverse", "double", 0, 0, 1)
        self.roundness_att = self.addAnimParam("roundness", "Roundness", "double", 0, 0, 1)
        self.tangentVis_att =  self.addAnimParam("Tangent_vis", "Tangent vis", "bool", False)

        self.boneALenghtMult_attr = self.addAnimParam("boneALenMult", "Bone A Mult", "double", 1)
        self.boneBLenghtMult_attr = self.addAnimParam("boneBLenMult", "Bone B Mult", "double", 1)
        self.boneALenght_attr = self.addAnimParam("boneALen", "Bone A Length", "double", self.length0, keyable=False)
        self.boneBLenght_attr = self.addAnimParam("boneBLen", "Bone B Length", "double", self.length1, keyable=False)

        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.settings["ikrefarray"].split(",")
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam("ikref", "Ik Ref", 0, self.settings["ikrefarray"].split(","))

        if self.settings["ikTR"]:
            ref_names = ["Auto", "ik_ctl"]
            if self.settings["ikrefarray"]:
                ref_names = ref_names + self.settings["ikrefarray"].split(",")
            self.ikRotRef_att = self.addAnimEnumParam("ikRotRef", "Ik Rot Ref", 0, ref_names)

        if self.settings["upvrefarray"]:
            ref_names = self.settings["upvrefarray"].split(",")
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.upvref_att = self.addAnimEnumParam("upvref", "UpV Ref", 0, ref_names)

        if self.settings["pinrefarray"]:
            ref_names = self.settings["pinrefarray" ].split(",")
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.pin_att = self.addAnimEnumParam("midref", "mid Ref", 0, ref_names)

        # Setup ------------------------------------------
        # Eval Fcurve
        # self.st_value = fcu.getFCurveValues(self.settings["st_profile"], self.divisions)
        # self.sq_value = fcu.getFCurveValues(self.settings["sq_profile"], self.divisions)
        if self.guide.paramDefs["st_profile"].value:
            self.st_value = self.guide.paramDefs["st_profile"].value
            self.sq_value = self.guide.paramDefs["sq_profile"].value
        else:
            self.st_value = fcurve.getFCurveValues(self.settings["st_profile"],
                                                   self.divisions)
            self.sq_value = fcurve.getFCurveValues(self.settings["sq_profile"],
                                                   self.divisions)

        self.st_att = [ self.addSetupParam("stretch_%s"%i, "Stretch %s"%i, "double", self.st_value[i], -1, 0) for i in range(self.divisions) ]
        self.sq_att = [ self.addSetupParam("squash_%s"%i, "Squash %s"%i, "double", self.sq_value[i], 0, 1) for i in range(self.divisions) ]

        self.resample_att = self.addSetupParam("resample", "Resample", "bool", True)
        self.absolute_att = self.addSetupParam("absolute", "Absolute", "bool", False)

    def addOperators(self):
        self.ikSolver = "ikRPsolver"
        

        # 1 bone chain Upv ref =====================================================================================
        self.ikHandleUpvRef = pri.addIkHandle(self.root, self.getName("ikHandlelimbChainUpvRef"), self.limbChainUpvRef, "ikSCsolver")
        pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        pm.parentConstraint( self.limbChainUpvRef[0],  self.upv_cns, mo=True)

        # mid joints =====================================================================================
        nod.createPairBlend(None, self.limbBones[1], .5, 1, self.mid1_jnt)
        pm.connectAttr(self.limbBones[1]+".translate", self.mid1_jnt+".translate", f=True)

        pm.parentConstraint(self.mid1_jnt, self.ctrn_loc)


        # Visibilities -------------------------------------
        # fk
        fkvis_node = nod.createReverseNode(self.blend_att)

        for shp in self.fk0_ctl.getShapes():
            pm.connectAttr(fkvis_node+".outputX", shp.attr("visibility"))
        for shp in self.fk1_ctl.getShapes():
            pm.connectAttr(fkvis_node+".outputX", shp.attr("visibility"))
        for shp in self.fk2_ctl.getShapes():
            pm.connectAttr(fkvis_node+".outputX", shp.attr("visibility"))

        # ik
        for shp in self.upv_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ikcns_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ik_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # Controls ROT order -----------------------------------
        att.setRotOrder(self.fk0_ctl, "XZY")
        att.setRotOrder(self.fk1_ctl, "XYZ")
        att.setRotOrder(self.fk2_ctl, "YZX")
        att.setRotOrder(self.ik_ctl, "XYZ")


        #joint length multiply
        multJnt1_node = nod.createMulNode(self.boneALenght_attr, self.boneALenghtMult_attr)
        multJnt2_node = nod.createMulNode(self.boneBLenght_attr, self.boneBLenghtMult_attr)

        # IK 2 bones ======================================================================================================

        self.ikHandle2 = pri.addIkHandle(self.softblendLoc2, self.getName("ik2BonesHandle"), self.limbBonesIK, self.ikSolver, self.upv_ctl)


        if self.negate:
            mulVal = 1
        else:
            mulVal = -1

        nod.createMulNode(self.roll_att, mulVal,self.ikHandle2.attr("twist"))

        # softIK 2 bones operators
        aop.aimCns(self.aim_tra2, self.ik2b_ik_ref, axis="xz", wupType=4, wupVector=[1,0,0], wupObject=self.root, maintainOffset=False)

        plusTotalLength_node = nod.createPlusMinusAverage1D([multJnt1_node.attr("outputX"), multJnt2_node.attr("outputX")])
        subtract1_node = nod.createPlusMinusAverage1D([plusTotalLength_node.attr("output1D"), self.soft_attr], 2)
        distance1_node = nod.createDistNode(self.ik2b_ik_ref, self.aim_tra2)
        div1_node = nod.createDivNode(1, self.rig.global_ctl+".sx")

        mult1_node = nod.createMulNode(distance1_node+".distance", div1_node+".outputX")
        subtract2_node = nod.createPlusMinusAverage1D([mult1_node.attr("outputX"), subtract1_node.attr("output1D")], 2)
        div2_node =  nod.createDivNode(subtract2_node+".output1D", self.soft_attr)
        mult2_node = nod.createMulNode(-1, div2_node+".outputX")
        power_node = nod.createPowNode(self.softSpeed_attr, mult2_node+".outputX")
        mult3_node = nod.createMulNode(self.soft_attr, power_node+".outputX" )
        subtract3_node = nod.createPlusMinusAverage1D([plusTotalLength_node.attr("output1D"),mult3_node.attr("outputX")], 2)

        cond1_node = nod.createConditionNode(self.soft_attr, 0, 2,subtract3_node+".output1D", plusTotalLength_node+".output1D")
        cond2_node = nod.createConditionNode(mult1_node+".outputX", subtract1_node+".output1D", 2,cond1_node+".outColorR", mult1_node+".outputX")

        pm.connectAttr(cond2_node+".outColorR", self.tipSoftIK+".tx")

        #soft blend
        pc_node = pm.pointConstraint( self.tipSoftIK, self.ik2b_ik_ref, self.softblendLoc2)
        nod.createReverseNode(self.stretch_attr, pc_node+".target[0].targetWeight")
        pm.connectAttr(self.stretch_attr, pc_node+".target[1].targetWeight", f=True)

        #Stretch
        distance2_node = nod.createDistNode(self.softblendLoc2, self.tipSoftIK)
        mult4_node = nod.createMulNode(distance2_node+".distance", div1_node+".outputX")


        for i, mulNode in enumerate([multJnt1_node, multJnt2_node]):
            div3_node = nod.createDivNode(mulNode+".outputX", plusTotalLength_node+".output1D")
            mult5_node = nod.createMulNode(mult4_node+".outputX", div3_node+".outputX")
            mult6_node = nod.createMulNode(self.stretch_attr, mult5_node+".outputX")
            nod.createPlusMinusAverage1D([mulNode.attr("outputX"), mult6_node.attr("outputX")],1, self.limbBonesIK[i+1]+".tx")

        ###  IK/FK connections

        for i, x in enumerate(self.fk_ctl):
            pm.parentConstraint( x, self.limbBonesFK[i], mo=True)


        nod.createPairBlend(self.limbBonesFK[0], self.limbBonesIK[0], self.blend_att, 1, self.limbBones[0])
        nod.createPairBlend(self.limbBonesFK[1], self.limbBonesIK[1], self.blend_att, 1,self.limbBones[1])
        nod.createPairBlend(self.limbBonesFK[2], self.limbBonesIK[2], self.blend_att, 1,self.limbBones[2], rot=False)
        parentc_node = pm.parentConstraint( self.limbBonesFK[2], self.ik2b_ik_ref, self.limbBones[2], mo=True, st=["x", "y", "z"])
        nod.createReverseNode(self.blend_att,  parentc_node+".target[0].targetWeight")
        pm.connectAttr(self.blend_att, parentc_node+".target[1].targetWeight", f=True)

        # Twist references ---------------------------------

        node = nod.createMultMatrixNode(self.eff_loc.attr("worldMatrix"), self.root.attr("worldInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(node+".matrixSum", dm_node+".inputMatrix")
        pm.connectAttr(dm_node+".outputTranslate", self.tws2_npo.attr("translate"))


        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(node+".matrixSum", dm_node+".inputMatrix")
        pm.connectAttr(dm_node+".outputRotate", self.tws2_npo.attr("rotate"))

        #spline IK for  twist jnts
        self.ikhUpperLimbTwist, self.upperLimbTwistCrv = aop.splineIK(self.getName("upperLimbTwist"), self.upperLimbTwistChain, parent=self.root, cParent=self.limbBones[0] )
        self.ikhlowerLimbTwist, self.lowerLimbTwistCrv = aop.splineIK(self.getName("lowerLimbTwist"), self.lowerLimbTwistChain, parent=self.root, cParent=self.limbBones[1] )
        pm.parent(self.upperLimbTwistCrv, self.setup)
        pm.parent(self.lowerLimbTwistCrv, self.setup)

        #references
        self.ikhUpperLimbRef, self.tmpCrv = aop.splineIK(self.getName("upperLimbRollRef"), self.upperLimbRollRef, parent=self.root, cParent=self.limbBones[0] )
        self.ikhlowerLimbRef, self.tmpCrv = aop.splineIK(self.getName("lowerLimbRollRef"), self.lowerLimbRollRef, parent=self.root, cParent=self.eff_loc )
        self.ikhAuxTwist, self.tmpCrv = aop.splineIK(self.getName("auxTwist"), self.auxTwistChain, parent=self.root, cParent=self.eff_loc )

        #setting connexions for ikhUpperLimbTwist
        self.ikhUpperLimbTwist.attr("dTwistControlEnable").set(True)
        self.ikhUpperLimbTwist.attr("dWorldUpType").set(4)
        self.ikhUpperLimbTwist.attr("dWorldUpAxis").set(3)
        self.ikhUpperLimbTwist.attr("dWorldUpVectorZ").set(1.0)
        self.ikhUpperLimbTwist.attr("dWorldUpVectorY").set(0.0)
        self.ikhUpperLimbTwist.attr("dWorldUpVectorEndZ").set(1.0)
        self.ikhUpperLimbTwist.attr("dWorldUpVectorEndY").set(0.0)
        pm.connectAttr(self.upperLimbRollRef[0].attr("worldMatrix[0]"), self.ikhUpperLimbTwist.attr("dWorldUpMatrix"))
        pm.connectAttr(self.limbBones[0].attr("worldMatrix[0]"), self.ikhUpperLimbTwist.attr("dWorldUpMatrixEnd"))

        #setting connexions for ikhAuxTwist
        self.ikhAuxTwist.attr("dTwistControlEnable").set(True)
        self.ikhAuxTwist.attr("dWorldUpType").set(4)
        self.ikhAuxTwist.attr("dWorldUpAxis").set(3)
        self.ikhAuxTwist.attr("dWorldUpVectorZ").set(1.0)
        self.ikhAuxTwist.attr("dWorldUpVectorY").set(0.0)
        self.ikhAuxTwist.attr("dWorldUpVectorEndZ").set(1.0)
        self.ikhAuxTwist.attr("dWorldUpVectorEndY").set(0.0)
        pm.connectAttr(self.lowerLimbRollRef[0].attr("worldMatrix[0]"), self.ikhAuxTwist.attr("dWorldUpMatrix"))
        pm.connectAttr(self.eff_loc.attr("worldMatrix[0]"), self.ikhAuxTwist.attr("dWorldUpMatrixEnd"))
        pm.connectAttr(self.auxTwistChain[1].attr("rx"), self.ikhlowerLimbTwist.attr("twist"))

        pm.parentConstraint(self.limbBones[1], self.aux_npo, maintainOffset=True)

        #scale upperLimb length for twist chain (not the squash and stretch)
        arclen_node = pm.arclen(self.upperLimbTwistCrv, ch=True)
        alAttrUpperLimb = arclen_node.attr("arcLength")
        muldiv_nodeUpperLimb =  pm.createNode("multiplyDivide")
        pm.connectAttr(arclen_node.attr("arcLength"), muldiv_nodeUpperLimb.attr("input1X"))
        muldiv_nodeUpperLimb.attr("input2X").set(alAttrUpperLimb.get())
        muldiv_nodeUpperLimb.attr("operation").set(2)
        for jnt in self.upperLimbTwistChain:
            pm.connectAttr(muldiv_nodeUpperLimb.attr("outputX"),jnt.attr("sx"))

        #scale lowerLimb length for twist chain (not the squash and stretch)
        arclen_node = pm.arclen(self.lowerLimbTwistCrv, ch=True)
        alAttrlowerLimb = arclen_node.attr("arcLength")
        muldiv_nodelowerLimb =  pm.createNode("multiplyDivide")
        pm.connectAttr(arclen_node.attr("arcLength"), muldiv_nodelowerLimb.attr("input1X"))
        muldiv_nodelowerLimb.attr("input2X").set(alAttrlowerLimb.get())
        muldiv_nodelowerLimb.attr("operation").set(2)
        for jnt in self.lowerLimbTwistChain:
            pm.connectAttr(muldiv_nodelowerLimb.attr("outputX"),jnt.attr("sx"))

        #scale compensation for the first  twist join
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(self.root.attr("worldMatrix[0]"), dm_node.attr("inputMatrix"))
        pm.connectAttr(dm_node.attr("outputScale"), self.upperLimbTwistChain[0].attr("inverseScale"))
        pm.connectAttr(dm_node.attr("outputScale"), self.lowerLimbTwistChain[0].attr("inverseScale"))

        #tangent controls
        muldiv_node =  pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws1A_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias =  pm.createNode("multiplyDivide")
        pm.connectAttr(muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X"))
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(muldiv_nodeBias.attr("outputX"), self.tws1A_loc.attr("rz") )
        if self.negate:
            axis = "xz"
        else:
            axis = "-xz"
        aop.aimCns(self.tws1A_npo, self.tws0_loc, axis=axis, wupType=2, wupVector=[0,0,1], wupObject=self.mid_ctl, maintainOffset=False)

        aop.aimCns(self.lowerLimbTangentB_loc, self.lowerLimbTangentA_npo, axis=axis, wupType=2, wupVector=[0,0,1], wupObject=self.mid_ctl, maintainOffset=False)
        pm.pointConstraint(self.eff_loc, self.lowerLimbTangentB_loc)


        muldiv_node =  pm.createNode("multiplyDivide")
        muldiv_node.attr("input2X").set(-1)
        pm.connectAttr(self.tws1B_npo.attr("rz"), muldiv_node.attr("input1X"))
        muldiv_nodeBias =  pm.createNode("multiplyDivide")
        pm.connectAttr(muldiv_node.attr("outputX"), muldiv_nodeBias.attr("input1X"))
        pm.connectAttr(self.roundness_att, muldiv_nodeBias.attr("input2X"))
        pm.connectAttr(muldiv_nodeBias.attr("outputX"), self.tws1B_loc.attr("rz") )
        if self.negate:
            axis = "-xz"
        else:
            axis = "xz"
        aop.aimCns(self.tws1B_npo, self.tws2_loc, axis=axis, wupType=2, wupVector=[0,0,1], wupObject=self.mid_ctl, maintainOffset=False)

        aop.aimCns(self.upperLimbTangentA_loc, self.upperLimbTangentB_npo, axis=axis, wupType=2, wupVector=[0,0,1], wupObject=self.mid_ctl, maintainOffset=False)

        # Volume -------------------------------------------
        distA_node = nod.createDistNode(self.tws0_loc, self.tws1_loc)
        distB_node = nod.createDistNode(self.tws1_loc, self.tws2_loc)
        add_node = nod.createAddNode(distA_node+".distance", distB_node+".distance")
        div_node = nod.createDivNode(add_node+".output", self.root.attr("sx"))

        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(self.root.attr("worldMatrix"), dm_node+".inputMatrix")

        div_node2 = nod.createDivNode(div_node+".outputX", dm_node+".outputScaleX")
        self.volDriver_att = div_node2+".outputX"

        # connecting tangent scaele compensation after volume to aboid duplicate some nodes ------------------------------
        distA_node = nod.createDistNode(self.tws0_loc, self.mid_ctl)
        distB_node = nod.createDistNode(self.mid_ctl, self.tws2_loc)


        div_nodeUpperLimb = nod.createDivNode(distA_node+".distance",  dm_node.attr("outputScaleX"))
        div_node2 = nod.createDivNode(div_nodeUpperLimb+".outputX", distA_node.attr("distance").get())
        pm.connectAttr(div_node2.attr("outputX"), self.tws1A_loc.attr("sx"))
        pm.connectAttr(div_node2.attr("outputX"), self.upperLimbTangentA_loc.attr("sx"))

        div_nodelowerLimb = nod.createDivNode(distB_node+".distance", dm_node.attr("outputScaleX"))
        div_node2 = nod.createDivNode(div_nodelowerLimb+".outputX", distB_node.attr("distance").get())
        pm.connectAttr(div_node2.attr("outputX"), self.tws1B_loc.attr("sx"))
        pm.connectAttr(div_node2.attr("outputX"), self.lowerLimbTangentB_loc.attr("sx"))

        #conection curve
        aop.curvecns_op(self.upperLimbTwistCrv, [ self.upperLimbTangentA_loc, self.upperLimbTangentA_ctl, self.upperLimbTangentB_ctl,self.midTangent_ctl ])
        aop.curvecns_op(self.lowerLimbTwistCrv, [ self.midTangent_ctl, self.lowerLimbTangentA_ctl, self.lowerLimbTangentB_ctl,self.lowerLimbTangentB_loc ])

        #Tangent controls vis
        for shp in self.upperLimbTangentA_ctl.getShapes():
            pm.connectAttr( self.tangentVis_att, shp.attr("visibility"))
        for shp in self.upperLimbTangentB_ctl.getShapes():
            pm.connectAttr( self.tangentVis_att, shp.attr("visibility"))
        for shp in self.lowerLimbTangentA_ctl.getShapes():
            pm.connectAttr( self.tangentVis_att, shp.attr("visibility"))
        for shp in self.lowerLimbTangentB_ctl.getShapes():
            pm.connectAttr( self.tangentVis_att, shp.attr("visibility"))
        for shp in self.midTangent_ctl.getShapes():
            pm.connectAttr( self.tangentVis_att, shp.attr("visibility"))

        # Divisions ----------------------------------------
        # at 0 or 1 the division will follow exactly the rotation of the controler.. and we wont have this nice tangent + roll
        for i, div_cns in enumerate(self.div_cns):
            if i < (self.settings["div0"]+1):
                mulmat_node = nod.createMultMatrixNode(self.upperLimbTwistChain[i]+".worldMatrix", div_cns+".parentInverseMatrix")
                lastUpperLimbDiv = div_cns
            elif i >= (self.settings["div0"]+1):
                mulmat_node = nod.createMultMatrixNode(self.lowerLimbTwistChain[i-(self.settings["div0"]+1)]+".worldMatrix", div_cns+".parentInverseMatrix")
                lastForeDiv = div_cns
            dm_node = nod.createDecomposeMatrixNode(mulmat_node+".matrixSum")
            pm.connectAttr(dm_node+".outputTranslate", div_cns+".t")
            pm.connectAttr(dm_node+".outputRotate", div_cns+".r")


        #force translation for last loc upperLimb and foreamr
        # nod.createMultMatrixNode(self.midTangent_ctl.worldMatrix,lastUpperLimbDiv.parentInverseMatrix, lastUpperLimbDiv, "t" )
        nod.createMultMatrixNode(self.tws2_loc.worldMatrix,lastForeDiv.parentInverseMatrix, lastForeDiv, "t" )
        # return

        # NOTE: next line fix the issue on meters.
        # This is special case becasuse the IK solver from mGear use the scale as lenght and we have shear
        # TODO: check for a more clean and elimbant solution instead of re-match the world matrix again
        tra.matchWorldTransform(self.fk_ctl[0], self.match_fk0_off)
        tra.matchWorldTransform(self.fk_ctl[1], self.match_fk1_off)
        tra.matchWorldTransform(self.fk_ctl[0], self.match_fk0)
        tra.matchWorldTransform(self.fk_ctl[1], self.match_fk1)

        # # match IK/FK ref
        pm.parentConstraint(self.limbBones[0], self.match_fk0_off, mo=True)
        pm.parentConstraint(self.limbBones[1], self.match_fk1_off, mo=True)



    # =====================================================
    # CONNECTOR
    # =====================================================
    ## Set the relation beetween object from guide to rig.\n
    # @param self
    def setRelation(self):
        
        self.relatives["root"] = self.div_cns[0]
        self.relatives["mid"] = self.div_cns[self.settings["div0"] + 2]
        self.relatives["tip"] = self.div_cns[-1]
        self.relatives["eff"] = self.eff_ref

        self.jointRelatives["root"] = 0
        self.jointRelatives["mid"] = self.settings["div0"] + 2
        self.jointRelatives["tip"] = len(self.div_cns)-2
        self.jointRelatives["eff"] = -1

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["mid"] = self.fk1_ctl
        self.controlRelatives["tip"] = self.fk2_ctl
        self.controlRelatives["eff"] = self.fk2_ctl

    ## Add more connection definition to the set.
    # @param self
    def addConnection(self):
        
        self.connections["shoulder_01"] = self.connect_shoulder_01

    ## standard connection definition.
    # @param self
    def connect_standard(self):
        
        self.connect_standardWithIkRef()

        if self.settings["pinrefarray"]:
            self.connectRef2("Auto,"+ self.settings["pinrefarray"], self.mid_cns, self.pin_att, [self.ctrn_loc], False)


    def connect_shoulder_01(self):
        
        self.connect_standard()
        pm.parent(self.rollRef[0],self.parent_comp.ctl)
