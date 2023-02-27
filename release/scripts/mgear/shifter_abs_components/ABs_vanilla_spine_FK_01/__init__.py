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
    def addObjects(self):
        self.WIP = self.options["mode"]

        self.normal = self.guide.blades["blade"].z * -1
        self.binormal = self.guide.blades["blade"].x

        self.setup = primitive.addTransformFromPos(self.setupWS,  self.getName("WS"))
        attribute.lockAttribute(self.setup)

        self.spine_fk = primitive.add2DChain(self.root, self.getName("spineFK%s_jnt"), self.guide.apos, self.normal, vis=self.WIP)

        self.fk_npo = []
        self.fk_ctl = []

        parent = self.root
        self.previusTag = self.parentCtlTag
        for i, j in enumerate(self.spine_fk):

            # t = j.getMatrix(ws=True)
            t = transform.setMatrixPosition(self.root.getMatrix(), transform.getTranslation(j))

            fk_npo = primitive.addTransform(parent, self.getName("FK" + str(i) + "_npo"), t)
            fk_ctl = self.addCtl(fk_npo, "FK" + str(i) + "_ctl", t, self.color_ik, "circle", w=2, tp=self.previusTag)

            
            self.fk_npo.append(fk_npo)
            self.fk_ctl.append(fk_ctl)
            
            self.previusTag = fk_ctl
            parent = fk_ctl

        self.spine_ik = primitive.add2DChain(self.setup, self.getName("spineIK%s_jnt"), self.guide.apos, self.normal, vis=self.WIP)
        self.chest_ik = primitive.addJoint(self.root, self.getName("chestIK"), self.spine_fk[-1].getMatrix(ws=True), vis=self.WIP)
        self.hips_ik = primitive.addJoint(self.root, self.getName("hipsIK"), self.spine_fk[0].getMatrix(ws=True), vis=self.WIP)

        num = int(len(self.guide.apos)/2)
        if len(self.guide.apos)%2 == 0:
            m = self.spine_fk[num - 1].getMatrix(ws=True)
            p = vector.linearlyInterpolate(self.guide.apos[num - 1], self.guide.apos[num], 0.5)
            t = transform.setMatrixPosition(m, p)

        else:
            pass
        
        self.mid_ik = primitive.addJoint(self.root, self.getName("midIK"), t, vis=self.WIP)
       
        t = transform.setMatrixPosition(self.root.getMatrix(), transform.getTranslation(self.fk_ctl[-1]))
        self.chest_ik_npo = primitive.addTransform(self.fk_ctl[-1], self.getName("chest_ik_npo"), t)
        self.chest_ik_ctl = self.addCtl(self.chest_ik_npo, "chest_ik_ctl", t, self.color_ik, "compas", w=1, tp=self.previusTag)
        pm.parent(self.chest_ik, self.chest_ik_ctl)

        t = self.root.getMatrix()
        self.hips_ik_npo = primitive.addTransform(self.fk_ctl[0], self.getName("hips_ik_npo"), t)
        self.hips_ik_ctl = self.addCtl(self.hips_ik_npo, "hips_ik_ctl", t, self.color_ik, "compas", w=1, tp=self.previusTag)
        pm.parent(self.hips_ik, self.hips_ik_ctl)

        t = transform.setMatrixPosition(self.root.getMatrix(), transform.getTranslation(self.mid_ik))
        self.mid_ik_npo = primitive.addTransform(self.fk_ctl[num - 1], self.getName("mid_ik_npo"), t)
        self.mid_ik_cns = primitive.addTransform(self.mid_ik_npo, self.getName("mid_ik_cns"), t)
        self.mid_ik_ctl = self.addCtl(self.mid_ik_cns, "mid_ik_ctl", t, self.color_ik, "sphere", w=1, tp=self.previusTag)
        pm.parent(self.mid_ik, self.mid_ik_ctl)


        for i,j in enumerate(self.spine_ik):
            if i == len(self.spine_ik) - 1:
                self.jnt_pos.append([self.chest_ik_ctl, i, None, True, False])
            else:
                self.jnt_pos.append([j, i, None, True, False])

            # self.jnt_pos.append([j, i, None, True, False])

        self.cnx0 = primitive.addTransform(self.root, self.getName("0_cnx"))
        self.cnx1 = primitive.addTransform(self.root, self.getName("1_cnx"))

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        return

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):    
        for i,j in enumerate(self.spine_fk):
            # applyop.ABs_matrix_cns_mantOff(self.fk_ctl[i], j)
            pm.parentConstraint(self.fk_ctl[i], j)

        ikHandle, ikCrv = applyop.ABs_SplineIK(self.getName("spineSplineIk"), self.spine_ik, self.root, numSpans=2)
        
        ikHandle.dTwistControlEnable.set(1)
        ikHandle.dWorldUpType.set(4)
        ikHandle.dForwardAxis.set(0)
        ikHandle.dWorldUpAxis.set(0)
        ikHandle.dWorldUpVector.set((0,0,-1))
        ikHandle.dWorldUpVectorEnd.set((0,0,-1))
        self.hips_ik.worldMatrix[0].connect(ikHandle.dWorldUpMatrix)
        self.chest_ik.worldMatrix[0].connect(ikHandle.dWorldUpMatrixEnd)

        # vName = ikCrv.name().replace("Ik_", "Fk_")
        # fkCrv = pm.duplicate(ikCrv, name = vName)

        pm.skinCluster(ikCrv, self.hips_ik, self.mid_ik, self.chest_ik, mi=2)
        # pm.skinCluster(fkCrv, self.spine_fk, mi=2)

        cInf = node.createCurveInfoNode(ikCrv)
        divNode = node.createDivNode(cInf.arcLength, cInf.arcLength.get())
        for i,j in enumerate(self.spine_ik):
            divNode.outputX.connect(j.scaleX)


        # distNode1 = node.createDistNode(self.hips_ik_ctl, self.chest_ik_ctl)
        # subNode1 = node.createSubNode(distNode1.distance, distNode1.distance.get())
        # divNode = node.createDivNode(subNode1.output, 2)

        # divNode.outputX.connect(self.mid_ik_cns.translateY)

        pm.parentConstraint(self.hips_ik_ctl, self.chest_ik_ctl, self.mid_ik_cns, st=["x","z"], sr=["x", "y", "z"] )


        # pm.pointConstraint(self.scl_transforms[0], self.cnx0)
        # pm.scaleConstraint(self.scl_transforms[0], self.cnx0)
        # pm.orientConstraint(self.ik0_ctl, self.cnx0)
        # pm.pointConstraint(self.scl_transforms[-1], self.cnx1)
        # pm.scaleConstraint(self.scl_transforms[-1], self.cnx1)
        # pm.orientConstraint(self.ik1_ctl, self.cnx1)
        

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        i = len(self.spine_fk) - 2

        self.relatives["root"] = self.hips_ik_ctl
        self.controlRelatives["root"] = self.hips_ik_ctl
        self.jointRelatives["root"] = 0
        self.aliasRelatives["root"] = "base"

        # self.relatives["2_loc"] = self.chest_ik_ctl
        # self.controlRelatives["2_loc"] = self.chest_ik_ctl
        # self.jointRelatives["2_loc"] = -1
        # self.aliasRelatives["2_loc"] = "tip"

        self.relatives["%s_loc" % i] = self.chest_ik_ctl
        self.controlRelatives["%s_loc" % i] = self.chest_ik_ctl
        self.jointRelatives["%s_loc" % i] = -1
        self.aliasRelatives["%s_loc" % i] = "tip"
