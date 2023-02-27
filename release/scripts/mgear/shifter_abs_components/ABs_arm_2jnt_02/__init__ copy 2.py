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


        self.limbIk_root = pri.addTransform(self.root, self.getName("limbIk_root"), self.guide.atra[0])

        # ik control
        self.ik_npo = pri.addTransform(self.limbIk_root, self.getName("ik_npo"), self.guide.atra[-2])
        self.ik_ctl = self.addCtl(self.ik_npo, "ik_ctl", self.guide.atra[-2], self.color_ik, "cube", w=self.size*.12, h=self.size*.12, d=self.size*.12)

        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size*.5
        v += self.guide.apos[1]

        self.upv_cns = pri.addTransformFromPos(self.limbIk_root, self.getName("upv_cns"), v)
        self.upv_ctl = self.addCtl(self.upv_cns, "upv_ctl", tra.getTransform(self.upv_cns), self.color_ik, "diamond", w=self.size*.12, tp=self.parentCtlTag)

        att.setInvertMirror(self.upv_ctl, ["tx"])
        att.setKeyableAttributes(self.upv_ctl, self.t_params)


        # create locators
        # self.upperArm_loc = pri.addLocator(self.limbIk_root, self.getName("upperArm_loc"), self.guide.atra[0], size=0.2)
        self.upperArmIK_loc = pri.addLocator(self.limbIk_root, self.getName("upperArmIK_loc"), self.guide.atra[0], size=0.2)
        # self.upperArmFK_loc = pri.addLocator(self.limbIk_root, self.getName("upperArmFK_loc"), self.guide.atra[0], size=0.2)
        self.wrist_loc = pri.addLocator(self.upperArmIK_loc, self.getName("wrist_loc"), self.guide.atra[-2], size=0.2)
        self.softBlend_loc = pri.addLocator(self.limbIk_root, self.getName("softBlend_loc"), self.guide.atra[-2], size=0.2)
        self.ctlDist_loc = pri.addLocator(self.ik_ctl, self.getName("ctlDist_loc"), self.guide.atra[-2], size=0.2)
        self.elbow_loc = pri.addLocator(self.upv_ctl, self.getName("elbow_loc"), tra.getTransform(self.upv_ctl), size=0.2)

        ik_loc = [self.upperArmIK_loc, self.wrist_loc, self.softBlend_loc, self.ctlDist_loc, self.elbow_loc]
        for loc in ik_loc:
            loc.visibility.set(self.WIP)


        # limb chain IK ref
        # self.limbBonesIK = pri.add2DChain(self.root, self.getName("limbIK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        # pm.parent(self.limbBonesIK[0], self.upperArm_loc)

        # limb chain
        self.limbBones = pri.add2DChain(self.root, self.getName("limbBones%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        pm.parent(self.limbBones[0], self.limbIk_root)

        # limb chain FK ref
        self.limbBonesFK = pri.add2DChain(self.root, self.getName("limbFK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        pm.parent(self.limbBonesFK[0], self.limbIk_root)

        # limb chain IK ref
        self.limbBonesIK = pri.add2DChain(self.root, self.getName("limbIK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        pm.parent(self.limbBonesIK[0], self.limbIk_root)

        # 1 bone chain for upv ref
        # self.limbChainUpvRef= pri.add2DChain(self.arm_npo, self.getName("limbUpvRef%s_jnt"), [self.guide.apos[0],self.guide.apos[2]], self.normal, False, self.WIP)
        # self.limbChainUpvRef[1].setAttr("jointOrientZ", self.limbChainUpvRef[1].getAttr("jointOrientZ")*-1)

        
        # FK Controlers -----------------------------------
        t = tra.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.realNegate)
        self.fk0_npo = pri.addTransform(self.limbIk_root, self.getName("fk0_npo"), t)
        self.fk0_ctl = self.addCtl(self.fk0_npo, "fk0_ctl", t, self.color_fk, "circle", w=self.length0, ro=dt.Vector(0,0,1.5708), degree=3, tp=self.parentCtlTag)
        att.setKeyableAttributes(self.fk0_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz"])

        t = tra.getTransformLookingAt(self.guide.apos[1], self.guide.apos[2], self.normal, "xz", self.realNegate)
        self.fk1_npo = pri.addTransform(self.fk0_ctl, self.getName("fk1_npo"), t)
        self.fk1_ctl = self.addCtl(self.fk1_npo, "fk1_ctl", t, self.color_fk, "circle", w=self.length0, ro=dt.Vector(0,0,1.5708), degree=3, tp=self.fk0_ctl)
        att.setKeyableAttributes(self.fk1_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz"])

        t = tra.getTransformLookingAt(self.guide.apos[2], self.guide.apos[3], self.normal, "xz", self.realNegate)
        self.fk2_npo = pri.addTransform(self.fk1_ctl, self.getName("fk2_npo"), t)
        self.fk2_ctl = self.addCtl(self.fk2_npo, "fk2_ctl", t, self.color_fk, "circle", w=self.length0, ro=dt.Vector(0,0,1.5708), degree=3, tp=self.fk1_ctl)
        att.setKeyableAttributes(self.fk2_ctl)

        self.fk_ctl = [self.fk0_ctl, self.fk1_ctl, self.fk2_ctl]

        for  x in self.fk_ctl:
            att.setInvertMirror(x, ["tx", "ty", "tz"])


    def addAttributes(self):
        
        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam("blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1)

        self.stretch_att = self.addAnimParam("stretch", "Stretch", "double", 0, 0, 1)
        self.softIk_att = self.addAnimParam("softIk", "Soft Ik", "double", 0, 0, 1)
        self.scaleBase_att = self.addAnimParam("scaleBase", "Scale Base", "double", 1, 0.01, 10)
        self.slide_att = self.addAnimParam("slide", "Slide", "double", 0, -1, 1)
        self.pin_att = self.addAnimParam("pin", "Pin", "double", 0, 0, 1)


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

    def addOperators(self):
        
        self.ikSolver = "ikRPsolver"

        self.ikHandle = pri.addIkHandle(self.softBlend_loc, self.getName("ik2BonesHandle"), self.limbBonesIK, self.ikSolver, self.elbow_loc)

        pm.aimConstraint(self.ik_ctl, self.upperArmIK_loc, offset=(0, 0, 0),  weight=1, aimVector=(1, 0, 0),  upVector = (0, 1, 0), worldUpType="objectrotation", worldUpVector=(0, 1, 0), worldUpObject=self.limbIk_root)
        
        stretchIbverse_node = nod.createReverseNode(self.stretch_att)
        vCon = pm.pointConstraint(self.wrist_loc, self.ik_ctl, self.softBlend_loc)
        
        self.stretch_att.connect(vCon.target[1].targetWeight, f=True)
        stretchIbverse_node.outputX.connect(vCon.target[0].targetWeight, f=True)


        # Soft Ik
        boneBaseLength = vec.getDistance(self.guide.pos["root"], self.guide.pos["tip"])
        upperArmLength = vec.getDistance(self.guide.pos["root"], self.guide.pos["mid"])
        lowerArmLength = vec.getDistance(self.guide.pos["mid"], self.guide.pos["tip"])

        control_dist = nod.createDistNode(self.ctlDist_loc, self.upperArmIK_loc)
        scale_mul =  nod.createFloatDiv(1, self.limbIk_root.scaleX)
        
        mul2 = nod.createFloatMul(scale_mul.outFloat, control_dist.distance)
        sub = nod.createFloatSub(boneBaseLength, self.softIk_att)
        sub1 = nod.createFloatSub(mul2.outFloat, sub.outFloat)
        div = nod.createFloatDiv(sub1.outFloat, self.softIk_att)
        mul = nod.createFloatMul(div.outFloat, -1)
        exp = nod.createFloatPow(2.718, mul.outFloat)
        mul1 = nod.createFloatMul(exp.outFloat, self.softIk_att)
        sub2 = nod.createFloatSub(boneBaseLength, mul1.outFloat)

        cond1 = nod.createConditionNode(firstTerm=self.softIk_att, secondTerm=0, operator=2, ifTrue=sub2.outFloat, ifFalse=boneBaseLength)
        cond2 = nod.createConditionNode(firstTerm=mul2.outFloat, secondTerm=sub.outFloat, operator=2, ifTrue=cond1.outColorR, ifFalse=mul2.outFloat)

        mul16 = nod.createFloatMul(control_dist.distance, scale_mul.outFloat)

        blend_attr = pm.createNode("blendTwoAttr")
        cond2.outColorR.connect(blend_attr.input[0])
        mul16.outFloat.connect(blend_attr.input[1])
        self.pin_att.connect(blend_attr.attributesBlender)

        blend_attr.output.connect(self.wrist_loc.translateX)


        # Tretchy Ik
        soft_dist = nod.createDistNode(self.wrist_loc, self.softBlend_loc)
        mul3 = nod.createFloatMul(soft_dist.distance, scale_mul.outFloat)
        div2 = nod.createFloatDiv(upperArmLength, boneBaseLength)
        div4 = nod.createFloatDiv(lowerArmLength, boneBaseLength)
        mul4 = nod.createFloatMul(div2.outFloat, mul3.outFloat)
        mul6 = nod.createFloatMul(div4.outFloat, mul3.outFloat)
        mul7 = nod.createFloatMul(self.stretch_att, mul6.outFloat)
        mul5 = nod.createFloatMul(self.stretch_att, mul4.outFloat)
        add = nod.createFloatAdd(mul5.outFloat, upperArmLength)
        add1 = nod.createFloatAdd(mul7.outFloat, lowerArmLength)
        mul12 = nod.createFloatMul(boneBaseLength, div4.outFloat)
        mul13 = nod.createFloatMul(mul12.outFloat, self.slide_att)
        mul11 = nod.createFloatMul(boneBaseLength, div2.outFloat)
        mul10 = nod.createFloatMul(mul11.outFloat,self.slide_att)

        cond4 = nod.createConditionNode(firstTerm=self.slide_att, secondTerm=0, operator=4, ifTrue=mul10.outFloat, ifFalse=mul13.outFloat)

        add2 = nod.createFloatAdd(add.outFloat, cond4.outColorR)
        mul14 = nod.createFloatMul(add2.outFloat, self.scaleBase_att)
        sub3 = nod.createFloatSub(add1.outFloat, cond4.outColorR)
        mul15 = nod.createFloatMul(sub3.outFloat, self.scaleBase_att)

        upperArm_dist = nod.createDistNode(self.upperArmIK_loc, self.elbow_loc)
        lowerArm_dist = nod.createDistNode(self.elbow_loc, self.softBlend_loc)

        mul17 = nod.createFloatMul(upperArm_dist.distance, scale_mul.outFloat)
        mul18 = nod.createFloatMul(lowerArm_dist.distance, scale_mul.outFloat)

        blend_attr2 = pm.createNode("blendTwoAttr")
        mul14.outFloat.connect(blend_attr2.input[0])
        mul17.outFloat.connect(blend_attr2.input[1])
        self.pin_att.connect(blend_attr2.attributesBlender)

        blend_attr3 = pm.createNode("blendTwoAttr")
        mul15.outFloat.connect(blend_attr3.input[0])
        mul18.outFloat.connect(blend_attr3.input[1])
        self.pin_att.connect(blend_attr3.attributesBlender)

        blend_attr2.output.connect(self.limbBonesIK[1].translateX)
        blend_attr3.output.connect(self.limbBonesIK[2].translateX)

        
        # attach Fk controls to fk chain
        # aop.ABs_matrix_cns(self.fk_ctl[0], self.limbBonesFK[0], connect_srt="rt")
        # aop.ABs_matrix_cns(self.fk_ctl[1], self.limbBonesFK[1], connect_srt="rt")
        # aop.ABs_matrix_cns(self.fk_ctl[2], self.limbBonesFK[2], connect_srt="rt")
        pm.parentConstraint(self.fk_ctl[0], self.limbBonesFK[0])
        pm.parentConstraint(self.fk_ctl[1], self.limbBonesFK[1])
        pm.parentConstraint(self.fk_ctl[2], self.limbBonesFK[2])

        
        # 1 bone chain Upv ref =====================================================================================
        # self.ikHandleUpvRef = pri.addIkHandle(self.root, self.getName("ikHandlelimbChainUpvRef"), self.limbChainUpvRef, "ikSCsolver")
        # pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        # pm.parentConstraint( self.limbChainUpvRef[0],  self.upv_cns, mo=True)


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
        # for shp in self.ikcns_ctl.getShapes():
        #     pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ik_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # Controls ROT order -----------------------------------
        att.setRotOrder(self.fk0_ctl, "XZY")
        att.setRotOrder(self.fk1_ctl, "XYZ")
        att.setRotOrder(self.fk2_ctl, "YZX")
        att.setRotOrder(self.ik_ctl, "XYZ")


        #Stretch FK
        # mulNode = nod.createMulNode(self.scale1X_att, self.limbBonesFK[1].translateX.get())
        # mulNode.outputX.connect(self.fk1_npo.translateX)

        # mulNode = nod.createMulNode(self.scale2X_att, self.limbBonesFK[2].translateX.get())
        # mulNode.outputX.connect(self.fk2_npo.translateX)

        # Connect IK/FK systems

        # for i, limb in enumerate(self.limbBones):
        #     nod.createPairBlend(self.limbBonesFK[i], self.limbBonesIK[i], blender=self.blend_att, output = limb)

        for i, limb in enumerate(self.limbBones):
            bc = pm.createNode("blendColors")
            self.limbBonesIK[i].rotate.connect(bc.color1)
            self.limbBonesFK[i].rotate.connect(bc.color2)
            bc.output.connect(limb.rotate)
            self.blend_att.connect(bc.blender)

        for i, limb in enumerate(self.limbBones):
            bc = pm.createNode("blendColors")
            self.limbBonesIK[i].translate.connect(bc.color1)
            self.limbBonesFK[i].translate.connect(bc.color2)
            bc.output.connect(limb.translate)
            self.blend_att.connect(bc.blender)


        # Divisions
        # for i, d_cns in enumerate(self.div_cns):
        #     aop.ABs_matrix_cns(self.limbBones[i], d_cns)
        


    # =====================================================
    # CONNECTOR
    # =====================================================
    ## Set the relation beetween object from guide to rig.\n
    # @param self
    def setRelation(self):
        return
        self.relatives["root"] = self.div_cns[0]
        self.relatives["mid"] = self.div_cns[1]
        self.relatives["tip"] = self.div_cns[2]

        self.jointRelatives["root"] = 0
        self.jointRelatives["mid"] = 1
        self.jointRelatives["tip"] = -1 

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["mid"] = self.fk1_ctl
        self.controlRelatives["tip"] = self.fk2_ctl
