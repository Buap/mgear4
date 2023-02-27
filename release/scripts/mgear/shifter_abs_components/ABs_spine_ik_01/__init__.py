"""Component Spine IK 01 module"""

import pymel.core as pm
from pymel.core import datatypes

import mgear
from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, curve
from mgear.core import attribute, transform, primitive

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""
        self.WIP = self.options["mode"]

        self.normal = self.guide.blades["blade"].z * -1
        self.binormal = self.guide.blades["blade"].x

        self.setup = primitive.addTransformFromPos(self.setupWS,  self.getName("WS"))
        attribute.lockAttribute(self.setup)

        self.length0 = vector.getDistance(self.guide.apos[0], self.guide.apos[1])

        #Spine mechanical joint chain
        positions = []
        for i in range(self.settings["division"]+1):
            p1 = self.guide.apos[0]
            p2 = self.guide.apos[1]
            blend = 1/self.settings["division"]*i
            
            pos = vector.linearlyInterpolate(p1,p2, blend)
            positions.append(pos)

        self.mch_spine = primitive.add2DChain(self.root, self.getName("mchSpine%s_jnt"), positions, self.normal, vis=self.WIP )


        #Ik controls
        self.hipIk_jnt =  primitive.addJoint(self.root, self.getName("hipIkCrv_jnt"),self.mch_spine[0].getMatrix(worldSpace=True), vis=self.WIP)
        self.hipIk_jnt.attr("radius").set(3)
        self.hipIk_jnt.attr("jointOrient").set(self.hipIk_jnt.attr("rotate").get())
        self.hipIk_jnt.attr("rotate").set((0,0,0))

        self.shoulderIk_jnt =  primitive.addJoint(self.root, self.getName("shoulderIkCrv_jnt"),self.mch_spine[-1].getMatrix(worldSpace=True), vis=self.WIP)
        self.shoulderIk_jnt.attr("radius").set(3)
        self.shoulderIk_jnt.attr("jointOrient").set(self.shoulderIk_jnt.attr("rotate").get())
        self.shoulderIk_jnt.attr("rotate").set((0,0,0))

        p = transform.getPositionFromMatrix(self.mch_spine[0].getMatrix(worldSpace=True))
        t = self.root.getMatrix()
        t = transform.setMatrixPosition(t,p)
        self.hips_npo = primitive.addTransform(self.root, self.getName("hips_npo"), t)
        self.hips_ctl = self.addCtl(self.hips_npo, "hips_ctl", t, self.color_fk, "cube", w=2, h=1, d=1, tp=self.parentCtlTag)
        attribute.setKeyableAttributes(self.hips_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sy"])

        p = transform.getPositionFromMatrix(self.mch_spine[-1].getMatrix(worldSpace=True))
        t = self.root.getMatrix()
        t = transform.setMatrixPosition(t,p)
        self.shldrs_npo = primitive.addTransform(self.root, self.getName("shoulders_npo"), t)
        self.shldrs_ctl = self.addCtl(self.shldrs_npo, "shoulders_ctl", t, self.color_fk, "cube", w=2, h=1, d=1, tp=self.parentCtlTag)
        attribute.setKeyableAttributes(self.shldrs_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sy"])


        #Fk controls
        t = vector.linearlyInterpolate(self.guide.apos[0], self.guide.apos[1], 0.33333)
        t = transform.setMatrixPosition(self.root.getMatrix(worldSpace=True), t)
        self.hipsFk1_npo = primitive.addTransform(self.root, self.getName("hipsFk1_npo"), t)
        self.hipsFk1_ctl = self.addCtl(self.hipsFk1_npo, "hipsFk1_ctl", t, self.color_fk, "circle", w=1, h=1, d=1, tp=self.parentCtlTag)
        attribute.setKeyableAttributes(self.hipsFk1_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"])

        t = vector.linearlyInterpolate(self.guide.apos[0], self.guide.apos[1], 0.66666)
        t = transform.setMatrixPosition(self.root.getMatrix(worldSpace=True), t)
        self.hipsFk2_npo = primitive.addTransform(self.hipsFk1_ctl, self.getName("hipsFk2_npo"), t)
        self.hipsFk2_ctl = self.addCtl(self.hipsFk2_npo, "hipsFk2_ctl", t, self.color_fk, "circle", w=1, h=1, d=1, tp=self.parentCtlTag)
        attribute.setKeyableAttributes(self.hipsFk2_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"])

        pm.parent(self.shldrs_ctl, self.hipsFk2_ctl)

        #Ik spline curve
        cv_pos = []
        cv_pos.append(list(self.guide.apos[0]))
        cv_pos.append(list(vector.linearlyInterpolate(self.guide.apos[0],self.guide.apos[1], 0.1)))
        cv_pos.append(list(vector.linearlyInterpolate(self.guide.apos[0],self.guide.apos[1], 0.5)))
        cv_pos.append(list(vector.linearlyInterpolate(self.guide.apos[0],self.guide.apos[1], 0.9)))
        cv_pos.append(list(self.guide.apos[1]))

        self.ik_crv = curve.addCurve(self.setup, "ikSpl_crv", cv_pos)
        self.ik_crv.visibility.set(self.WIP)

        for i,j in enumerate(self.mch_spine):
            self.jnt_pos.append([j, i, None, True, False])

        self.cnx0 = primitive.addTransform(self.root, self.getName("0_cnx"))
        self.cnx1 = primitive.addTransform(self.shldrs_ctl, self.getName("1_cnx"))

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        return

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):    

        splineIkHndl, splineIkCrv = applyop.splineIK(name="spineIk", chn=self.mch_spine, parent=self.root, cParent=self.setup, curve=self.ik_crv)
        self.ik_crv.worldSpace[0].connect(splineIkHndl.inCurve, f=True)
        pm.delete(splineIkCrv)
        
        splineIkHndl.dTwistControlEnable.set(1)
        splineIkHndl.dWorldUpType.set(4)
        splineIkHndl.dForwardAxis.set(0)
        splineIkHndl.dWorldUpAxis.set(0)
        splineIkHndl.dWorldUpVector.set((0,0,-1))
        splineIkHndl.dWorldUpVectorEnd.set((0,0,-1))
        self.hipIk_jnt.worldMatrix[0].connect(splineIkHndl.dWorldUpMatrix)
        self.shoulderIk_jnt.worldMatrix[0].connect(splineIkHndl.dWorldUpMatrixEnd)

        pm.skinCluster(self.ik_crv, self.hipIk_jnt, self.shoulderIk_jnt)

        pm.parentConstraint(self.hips_ctl,self.hipIk_jnt, mo=True)
        pm.parentConstraint(self.shldrs_ctl,self.shoulderIk_jnt, mo=True)
        pm.scaleConstraint(self.shldrs_ctl,self.shoulderIk_jnt)


        cinfo = node.createCurveInfoNode(self.ik_crv)
        md = node.createDivNode(cinfo.arcLength, cinfo.getAttr("arcLength"))
        
        sr = node.createDivNode(md.outputX, 0.5)
        sr.setAttr("operation",3)

        sri = node.createDivNode(1, sr.outputX)
        sri.setAttr("operation",2)

        for j in self.mch_spine:
            md.outputX.connect(j.scaleX)
            sri.outputX.connect(j.scaleY)
            sri.outputX.connect(j.scaleZ)

        


        return

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        self.relatives["root"] = self.cnx0
        self.relatives["eff"] = self.cnx1

        self.controlRelatives["root"] = self.hips_ctl
        self.controlRelatives["eff"] = self.shldrs_ctl

        self.jointRelatives["root"] = 0
        self.jointRelatives["eff"] = -1

        self.aliasRelatives["root"] = "base"
        self.aliasRelatives["eff"] = "tip"

        
