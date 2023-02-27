"""Component Control 01 module"""
import ast
from mgear.shifter import component

from mgear.core import attribute, transform, primitive, vector, node
from pymel.core import datatypes
import pymel.core as pm

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

        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        # self.radius = vector.getDistance(self.guide.apos[0], self.guide.apos[1])

        t = transform.getTransformLookingAt(self.guide.apos[0], self.guide.apos[1], self.normal)

        self.wheel_npo = primitive.addTransform(self.root, self.getName("wheel_npo"), t)
        self.wheel_ctl = self.addCtl(self.wheel_npo, "wheel_ctl", t, self.color_fk, "circle", w=1, h=1, d=1, tp=self.parentCtlTag)
        self.wheel_ref = primitive.addTransform(self.wheel_ctl, self.getName("wheel_ref"), t)

        self.size1_loc = primitive.addLocator(self.root,self.getName("size1"), self.guide.atra[1])
        self.size2_loc = primitive.addLocator(self.root, self.getName("size2"), self.root.getMatrix())
        self.size2_loc.translateZ.set(-self.size1_loc.translateZ.get())

        self.wheel_loc = primitive.addLocator(self.root,self.getName("wheel_loc"), self.guide.atra[0])



    def addAttributes(self):
        self.diameter_att = self.addAnimParam("diameter", "diameter", "double", 1 , 0, 1)
        self.autoroll_att = self.addAnimParam("autoroll", "autoroll", "long", 1 , 0, 1)
        self.ball_att = self.addAnimParam("ball", "ball", "bool", 1)
    def addOperators(self):
        return

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        return

