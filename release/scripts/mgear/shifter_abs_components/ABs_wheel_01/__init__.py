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

        self.main_ctl = pm.PyNode("local_C0_ctl")

        self.jnt_pos.append([self.wheel_ref, 0, None, True, False])


    def addAttributes(self):
        self.wheelAuto_att = self.addAnimParam("wheelAuto", "wheelAuto", "double", 1 , 0, 1)
        self.speed_att = self.addAnimParam("speed", "speed", "double", 1 , 0, 10)
    def addOperators(self):
        size_node = node.createDistNode(self.size1_loc, self.size2_loc)
        dist_node = node.createDistNode(self.wheel_loc, self.wheel_ref)
        
        mul_node1 = node.createMulNode(dist_node.distance, 360)
        mul_node2 = node.createMulNode(size_node.distance, 3.14)
        mul_node3 = node.createMulNode(mul_node1.outputX, mul_node2.outputX)
        mul_node3.operation.set(2)

        mul_node4 = node.createMulNode(mul_node3.outputX, self.speed_att)

        blend_node = node.createBlendNode(mul_node4.outputX, [0,0,0], self.wheelAuto_att)

        blend_node.outputR.connect(self.wheel_ref.rotateY)

        return
        
        
        
        
        # moveVecOld = node.createDecomposeMatrixNode(self.old_loc.attr("worldMatrix[0]"))
        # moveVec = node.createDecomposeMatrixNode(self.wheel_ref.attr("worldMatrix[0]"))
        # dirVec = node.createDecomposeMatrixNode(self.dir_loc.attr("worldMatrix[0]"))

        # wheelVec = pm.createNode("plusMinusAverage")
        # wheelVec.attr("operation").set(2)
        # dirVec.outputTranslate.connect(wheelVec.input3D[0])
        # moveVec.outputTranslate.connect(wheelVec.input3D[1])
        
        # motionVec =pm.createNode("plusMinusAverage")
        # motionVec.attr("operation").set(2)
        # moveVec.outputTranslate.connect(motionVec.input3D[0])
        # moveVecOld.outputTranslate.connect(motionVec.input3D[1])

        # dist = pm.createNode("distanceBetween")
        # moveVec.outputTranslate.connect(dist.point1)
        # moveVecOld.outputTranslate.connect(dist.point2)


        # dot = pm.createNode("vectorProduct")
        # dot.attr("operation").set(1)
        # dot.attr("normalizeOutput").set(1)
        # motionVec.output3D.connect(dot.input1)
        # wheelVec.output3D.connect(dot.input2)
        # return



    # float $radius = 25;
    # vector $moveVectorOld = `xform -q -ws -t "leftWheelOld"`;
    # vector $moveVector = `xform -q -ws -t "L_Wheel"`;
    # vector $dirVector = `xform -q -ws -t "leftWheelDir"`;
    # vector $wheelVector = ($dirVector - $moveVector);
    # vector $motionVector = ($moveVector - $moveVectorOld);
    # float $distance = mag($motionVector);
    # $dot = dotProduct($motionVector, $wheelVector, 1);

    # L_Wheel.rotateZ = L_Wheel.rotateZ + 360 / (6.283*$radius) * ($dot*$distance) * (Root_CTRL.leftWheelAuto);
    # xform -t ($moveVector.x) ($moveVector.y) ($moveVector.z) leftWheelOld;
    # if (frame==0) { L_Wheel.rotateZ = 0;}

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        return

