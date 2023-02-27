"""Component Leg 2 joints 01 module"""

import pymel.core as pm
from pymel.core import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, icon
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
        self.WIP = self.options["mode"]
        
        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        t = transform.getTransformFromPos(self.guide.apos[0])
        self.root_npo = primitive.addTransform(self.root, self.getName("root_npo"), t)
        self.root_ctl = self.addCtl(self.root_npo, "root_ctl", t, self.color_fk, "circle", w=1)

        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size*.5
        v += self.guide.apos[1]

        self.upv_cns = primitive.addTransform(self.root, self.getName("upv_cns"), v)
        self.upv_ctl = self.addCtl(self.upv_cns, "upv_ctl", transform.getTransform(self.upv_cns), self.color_fk, "circle", w=self.size * .12, tp=self.root_ctl)

        self.ik_npo = primitive.addTransform(self.root, self.getName("ik_npo"), transform.getTransformFromPos(self.guide.pos["ankle"]))
        self.ik_ctl = self.addCtl( self.ik_npo, "ik_ctl", transform.getTransformFromPos(self.guide.pos["ankle"]), self.color_ik, "cube", w=self.size * .12, h=self.size * .12, d=self.size * .12)

        self.legChain = primitive.add2DChain(
            self.root_ctl,
            self.getName("legHz%s_jnt"),
            [self.guide.apos[0], self.guide.apos[1], self.guide.apos[2]],
            self.normal,
            False)

        self.ikHandle = primitive.addIkHandle(
            self.root, 
            self.getName("ik2BonesHandle"),
            self.legChain,
            "ikRPsolver",
            self.upv_ctl
            )
        pm.pointConstraint(self.ik_ctl, self.ikHandle)
        #pm.parentConstraint( self.legChain[0],  self.upv_cns, mo=True)

        self.jnt_pos.append([self.legChain[0], "l1"])
        self.jnt_pos.append([self.legChain[1], "l2"])
        self.jnt_pos.append([self.  legChain[2], "l3"])



        return
        self.ik_npo = primitive.addTransform(self.root, self.getName("ik_npo"), transform.getTransformFromPos(self.guide.pos["ankle"]))
        self.ik_ctl = self.addCtl( self.ik_npo, "ik_ctl", transform.getTransformFromPos(self.guide.pos["ankle"]), self.color_ik, "cube", w=self.size * .12, h=self.size * .12, d=self.size * .12)
        
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size * .5
        v += self.guide.apos[1]

        self.upv_cns = primitive.addTransformFromPos(self.root, self.getName("upv_cns"), v)
        self.upv_ctl = self.addCtl( self.upv_cns, "upv_ctl", transform.getTransform(self.upv_cns), self.color_ik, "diamond", w=self.size * .12, tp=self.root_ctl)

        self.ik_ref = primitive.addTransform( self.ik_ctl, self.getName("ik_ref"), transform.getTransform(self.ik_ctl))

        #self.limbBones = primitive.add2DChain(self.root, self.getName("limbBones%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        # self.jnt_pos.append([self.root, "root"])
        # self.jnt_pos.append([self.knee_npo, "knee"])
        # self.jnt_pos.append([self.ik_ctl, "ik"])
        


        return

    def addAttributes(self):
        return

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        return

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        
        self.relatives["root"] = self.root_ctl
        self.controlRelatives["root"] = self.root_ctl
        self.aliasRelatives["root"] = "root_ctl"
        
        self.jointRelatives["root"] = 0
        self.jointRelatives["eff"] = -1

        self.relatives["eff"] = self.ik_ctl
        self.controlRelatives["eff"] = self.ik_ctl


        
    def connect_standard(self):
        self.connect_standardWithIkRef()
