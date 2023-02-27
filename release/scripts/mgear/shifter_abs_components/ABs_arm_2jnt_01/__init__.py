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

        t = tra.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.realNegate)
        self.arm_npo = pri.addTransform(self.root, self.getName("arm_npo"), t)

        # limb chain
        self.limbBones = pri.add2DChain(self.arm_npo, self.getName("limbBones%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        # limb chain FK ref
        self.limbBonesFK = pri.add2DChain(self.arm_npo, self.getName("limbFK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        # limb chain IK ref
        self.limbBonesIK = pri.add2DChain(self.arm_npo, self.getName("limbIK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
    
        # 1 bone chain for upv ref
        self.limbChainUpvRef= pri.add2DChain(self.arm_npo, self.getName("limbUpvRef%s_jnt"), [self.guide.apos[0],self.guide.apos[2]], self.normal, False, self.WIP)
        self.limbChainUpvRef[1].setAttr("jointOrientZ", self.limbChainUpvRef[1].getAttr("jointOrientZ")*-1)

        
        # FK Controlers -----------------------------------
        t = tra.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal, "xz", self.realNegate)
        self.fk0_npo = pri.addTransform(self.arm_npo, self.getName("fk0_npo"), t)
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
        
        
        # IK Controlers -----------------------------------

        self.ik_cns = pri.addTransformFromPos(self.root, self.getName("ik_cns"), self.guide.pos["tip"])

        self.ikcns_ctl = self.addCtl(self.ik_cns, "ikcns_ctl", tra.getTransformFromPos(self.guide.pos["tip"]), self.color_ik, "null", w=self.size*.12, tp=self.parentCtlTag)
        att.setInvertMirror(self.ikcns_ctl, ["tx", "ty", "tz"])

        m = tra.getTransformFromPos(self.guide.pos["tip"])
        self.ik_ctl = self.addCtl(self.ikcns_ctl, "ik_ctl", m, self.color_ik, "cube", w=self.size*.12, h=self.size*.12, d=self.size*.12, tp=self.ikcns_ctl)

        att.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        att.setKeyableAttributes(self.ik_ctl)

        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size*.5
        v += self.guide.apos[1]

        self.upv_cns = pri.addTransformFromPos(self.root, self.getName("upv_cns"), v)

        self.upv_ctl = self.addCtl(self.upv_cns, "upv_ctl", tra.getTransform(self.upv_cns), self.color_ik, "diamond", w=self.size*.12, tp=self.parentCtlTag)

        att.setInvertMirror(self.upv_ctl, ["tx"])
        att.setKeyableAttributes(self.upv_ctl, self.t_params)

        #Stretch
        self.stretchBase_loc = pri.addLocator(self.arm_npo, self.getName("stretchBase_loc"), self.root.getMatrix(ws=True))
        self.stretchBase_loc.visibility.set(self.WIP)
        

        # Divisions
        self.div_cns = []
        for i in range(len(self.limbBones)):
            div_cns = pri.addTransform(self.root, self.getName("div%s_loc" % i))
            self.div_cns.append(div_cns)
            self.jnt_pos.append([div_cns, i, None, True, False])


    def addAttributes(self):
        
        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam("blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1)

        self.stretch_att = self.addAnimParam("stretch", "Stretch", "double", 1, 0, 1)

        self.scaleIk_att = self.addAnimParam("scaleIk", "Scale Ik", "double", 1, 0.1, 3)

        self.scale1X_att = self.addAnimParam("scale1X", "scale1X", "double", 1, 0, 5)
        self.scale2X_att = self.addAnimParam("scale2X", "scale2X", "double", 1, 0, 5)

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
        aop.ABs_matrix_cns(self.fk_ctl[0], self.limbBonesFK[0], connect_srt="rt")
        aop.ABs_matrix_cns(self.fk_ctl[1], self.limbBonesFK[1], connect_srt="rt")
        aop.ABs_matrix_cns(self.fk_ctl[2], self.limbBonesFK[2], connect_srt="rt")

        self.ikSolver = "ikRPsolver"

        # 1 bone chain Upv ref =====================================================================================
        self.ikHandleUpvRef = pri.addIkHandle(self.root, self.getName("ikHandlelimbChainUpvRef"), self.limbChainUpvRef, "ikSCsolver")
        pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        pm.parentConstraint( self.limbChainUpvRef[0],  self.upv_cns, mo=True)


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

        
        # IK 2 bones ======================================================================================================
        self.ikHandle2 = pri.addIkHandle(self.ik_ctl, self.getName("ik2BonesHandle"), self.limbBonesIK, self.ikSolver, self.upv_ctl)
        aop.ABs_matrix_cns_offset(self.ik_ctl, self.limbBonesIK[-1], connect_srt="r")

        #Stretch IK
        dist_node = nod.createDistNode(self.stretchBase_loc, self.ik_ctl)
        base_dist = nod.createMulNode(dist_node.distance.get(), self.rig.global_ctl.scaleX)

        divNode = nod.createDivNode(dist_node.distance, base_dist.outputX)
        condNode = nod.createConditionNode(dist_node.distance, base_dist.outputX, 2, divNode.outputX, 1)

        blAttr = pm.createNode("blendTwoAttr")
        condNode.outColorR.connect(blAttr.input[1])
        blAttr.input[0].set(1)
        self.stretch_att.connect(blAttr.attributesBlender)

        scaleMul = nod.createMulNode(blAttr.output, self.scaleIk_att)

        tranMult1 = nod.createMulNode(scaleMul.outputX, self.limbBonesIK[1].translateX.get())
        tranMult2 = nod.createMulNode(scaleMul.outputX, self.limbBonesIK[2].translateX.get())
        tranMult1.outputX.connect(self.limbBonesIK[1].translateX)
        tranMult2.outputX.connect(self.limbBonesIK[2].translateX)

        #Stretch FK
        mulNode = nod.createMulNode(self.scale1X_att, self.limbBonesFK[1].translateX.get())
        mulNode.outputX.connect(self.fk1_npo.translateX)

        mulNode = nod.createMulNode(self.scale2X_att, self.limbBonesFK[2].translateX.get())
        mulNode.outputX.connect(self.fk2_npo.translateX)

        # Connect IK/FK systems

        for i, limb in enumerate(self.limbBones):
            nod.createPairBlend(self.limbBonesFK[i], self.limbBonesIK[i], blender=self.blend_att, output = limb)

        # for i, limb in enumerate(self.limbBones):
        #     bc = pm.createNode("blendColors")
        #     self.limbBonesIK[i].rotate.connect(bc.color1)
        #     self.limbBonesFK[i].rotate.connect(bc.color2)
        #     bc.output.connect(limb.rotate)
        #     self.blend_att.connect(bc.blender)

        # for i, limb in enumerate(self.limbBones):
        #     bc = pm.createNode("blendColors")
        #     self.limbBonesIK[i].translate.connect(bc.color1)
        #     self.limbBonesFK[i].translate.connect(bc.color2)
        #     bc.output.connect(limb.translate)
        #     self.blend_att.connect(bc.blender)


        # Divisions
        for i, d_cns in enumerate(self.div_cns):
            aop.ABs_matrix_cns(self.limbBones[i], d_cns)
        


    # =====================================================
    # CONNECTOR
    # =====================================================
    ## Set the relation beetween object from guide to rig.\n
    # @param self
    def setRelation(self):
        self.relatives["root"] = self.div_cns[0]
        self.relatives["mid"] = self.div_cns[1]
        self.relatives["tip"] = self.div_cns[2]

        self.jointRelatives["root"] = 0
        self.jointRelatives["mid"] = 1
        self.jointRelatives["tip"] = -1 

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["mid"] = self.fk1_ctl
        self.controlRelatives["tip"] = self.fk2_ctl
