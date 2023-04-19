"""Component Foot Banking 01 module"""

import pymel.core as pm
from pymel.core import datatypes

from mgear.shifter import component

from mgear.core import node, applyop, vector
import mgear.core.primitive as pri
import mgear.core.transform as tra
import mgear.core.attribute as att
import mgear.core.node as nod
import mgear.core.vector as vec
import mgear.core.applyop as aop


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):

        self.WIP = self.options["mode"]

        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        self.length0 = vec.getDistance(self.guide.apos[0], self.guide.apos[1])
        self.length1 = vec.getDistance(self.guide.apos[1], self.guide.apos[2])

        self.footBones = pri.add2DChain(self.root, self.getName("footBones%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        self.footBonesFK = pri.add2DChain(self.root, self.getName("footBonesFK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)
        self.footBonesIK = pri.add2DChain(self.root, self.getName("footBonesIK%s_jnt"), self.guide.apos[0:3], self.normal, False, self.WIP)

        centers = [self.guide.pos["heel"], self.guide.apos[2] ,self.guide.apos[1],self.guide.apos[0]]
        self.reverseFoot = pri.add2DChain(self.root, self.getName("revFoot%s_jnt"), centers, self.normal, False, self.WIP)
        return
        self.div_count = len(self.guide.apos) - 5

        plane = [self.guide.apos[0], self.guide.apos[-4], self.guide.apos[-3]]
        self.normal = self.getNormalFromPos(plane)
        self.binormal = self.getBiNormalFromPos(plane)

        # Heel ---------------------------------------------
        # bank pivot

        t = transform.getTransformLookingAt(
            self.guide.pos["heel"],
            self.guide.apos[-4],
            self.normal,
            "xz",
            self.negate,
        )

        t = transform.setMatrixPosition(t, self.guide.pos["inpivot"])

        self.in_npo = primitive.addTransform(
            self.root, self.getName("in_npo"), t
        )

        self.in_piv = primitive.addTransform(
            self.in_npo, self.getName("in_piv"), t
        )

        t = transform.setMatrixPosition(t, self.guide.pos["outpivot"])

        self.out_piv = primitive.addTransform(
            self.in_piv, self.getName("out_piv"), t
        )

        # heel
        t = transform.getTransformLookingAt(
            self.guide.pos["heel"],
            self.guide.apos[-4],
            self.normal,
            "xz",
            self.negate,
        )

        self.heel_loc = primitive.addTransform(
            self.out_piv, self.getName("heel_loc"), t
        )

        attribute.setRotOrder(self.heel_loc, "YZX")
        self.heel_ctl = self.addCtl(
            self.heel_loc,
            "heel_ctl",
            t,
            self.color_ik,
            "sphere",
            w=self.size * 0.1,
            tp=self.parentCtlTag,
        )

        attribute.setKeyableAttributes(self.heel_ctl, self.r_params)

        # Tip ----------------------------------------------
        v = datatypes.Vector(
            self.guide.apos[-5].x, self.guide.apos[-1].y, self.guide.apos[-5].z
        )
        t = transform.setMatrixPosition(t, v)
        self.tip_ctl = self.addCtl(
            self.heel_ctl,
            "tip_ctl",
            t,
            self.color_ik,
            "circle",
            w=self.size,
            tp=self.heel_ctl,
        )
        attribute.setKeyableAttributes(self.tip_ctl, self.r_params)

        # Roll ---------------------------------------------
        if self.settings["useRollCtl"]:
            t = transform.getTransformLookingAt(
                self.guide.pos["heel"],
                self.guide.apos[-4],
                self.normal,
                "xz",
                self.negate,
            )
            t = transform.setMatrixPosition(t, self.guide.pos["root"])

            self.roll_np = primitive.addTransform(
                self.root, self.getName("roll_npo"), t
            )

            self.roll_ctl = self.addCtl(
                self.roll_np,
                "roll_ctl",
                t,
                self.color_ik,
                "cylinder",
                w=self.size * 0.5,
                h=self.size * 0.5,
                ro=datatypes.Vector(3.1415 * 0.5, 0, 0),
                tp=self.tip_ctl,
            )

            attribute.setKeyableAttributes(self.roll_ctl, ["rx", "rz"])

        # Backward Controlers ------------------------------
        bk_pos = self.guide.apos[1:-3]
        bk_pos.reverse()
        parent = self.tip_ctl
        self.bk_ctl = []
        self.bk_loc = []
        self.previousTag = self.tip_ctl
        for i, pos in enumerate(bk_pos):

            if i == 0:
                t = transform.getTransform(self.heel_ctl)
                t = transform.setMatrixPosition(t, pos)
            else:
                dir = bk_pos[i - 1]
                t = transform.getTransformLookingAt(
                    pos, dir, self.normal, "xz", self.negate
                )

            bk_loc = primitive.addTransform(
                parent, self.getName("bk%s_loc" % i), t
            )
            bk_ctl = self.addCtl(
                bk_loc,
                "bk%s_ctl" % i,
                t,
                self.color_ik,
                "sphere",
                w=self.size * 0.15,
                tp=self.previousTag,
            )
            attribute.setKeyableAttributes(bk_ctl, self.r_params)
            self.previousTag = bk_ctl

            self.bk_loc.append(bk_loc)
            self.bk_ctl.append(bk_ctl)
            parent = bk_ctl

        # FK Reference ------------------------------------
        self.fk_ref = primitive.addTransformFromPos(
            self.bk_ctl[-1], self.getName("fk_ref"), self.guide.apos[0]
        )
        self.fk_npo = primitive.addTransform(
            self.fk_ref,
            self.getName("fk0_npo"),
            transform.getTransform(self.bk_ctl[-1]),
        )

        # Forward Controlers ------------------------------
        self.fk_ctl = []
        self.fk_loc = []
        parent = self.fk_npo
        self.previousTag = self.tip_ctl
        for i, bk_ctl in enumerate(reversed(self.bk_ctl[1:])):
            t = transform.getTransform(bk_ctl)
            dist = vector.getDistance(
                self.guide.apos[i + 1], self.guide.apos[i + 2]
            )

            fk_loc = primitive.addTransform(
                parent, self.getName("fk%s_loc" % i), t
            )

            po_vec = datatypes.Vector(dist * 0.5 * self.n_factor, 0, 0)
            fk_ctl = self.addCtl(
                fk_loc,
                "fk%s_ctl" % i,
                t,
                self.color_fk,
                "cube",
                w=dist,
                h=self.size * 0.5,
                d=self.size * 0.5,
                po=po_vec,
                tp=self.previousTag,
            )

            self.previousTag = fk_ctl
            attribute.setKeyableAttributes(fk_ctl)
            self.jnt_pos.append([fk_ctl, i])

            parent = fk_ctl
            self.fk_ctl.append(fk_ctl)
            self.fk_loc.append(fk_loc)

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        self.blend_att = self.addAnimParam("blend", "Fk/Ik Blend", "double", 1, 0, 1)

        return
        # Anim -------------------------------------------
        # Roll Angles
        if not self.settings["useRollCtl"]:
            self.roll_att = self.addAnimParam(
                "roll", "Roll", "double", 0, -180, 180
            )
            self.bank_att = self.addAnimParam(
                "bank", "Bank", "double", 0, -180, 180
            )

        self.angles_att = [
            self.addAnimParam(
                "angle_%s" % i,
                "Angle %s" % i,
                "double",
                self.settings["rollAngle"],
            )
            for i in range(self.div_count)
        ]

        # Setup ------------------------------------------
        self.blend_att = self.addSetupParam(
            "blend", "Fk/Ik Blend", "double", 1, 0, 1
        )

        # track match references for IK/FK match
        self.root.addAttr("bk_ctl", at="message", m=True)
        self.root.addAttr("fk_ctl", at="message", m=True)
        for i, ctl in enumerate(self.bk_ctl):
            pm.connectAttr(
                ctl.message, self.root.attr("bk_ctl[{}]".format(str(i)))
            )
        for i, ctl in enumerate(self.fk_ctl):
            pm.connectAttr(
                ctl.message, self.root.attr("fk_ctl[{}]".format(str(i)))
            )

        self.root.addAttr("heel_ctl", at="message", m=False)
        self.heel_ctl.message >> self.root.heel_ctl
        self.root.addAttr("tip_ctl", at="message", m=False)
        self.tip_ctl.message >> self.root.tip_ctl
        if self.settings["useRollCtl"]:
            self.root.addAttr("roll_ctl", at="message", m=False)
            self.roll_ctl.message >> self.root.roll_ctl
        else:
            self.roll_cnx = self.addSetupParam(
                "roll_cnx", "Roll_cnx", "double", 0, -180, 180
            )
            self.roll_att >> self.roll_cnx
            self.bank_cnx = self.addSetupParam(
                "bank_cnx", "Bank_cnx", "double", 0, -180, 180
            )
            self.bank_att >> self.bank_cnx

    # =====================================================
    # OPERATORS
    # =====================================================

    def addOperators(self):
        for i,foot in enumerate(self.footBones):
            con = pm.parentConstraint(self.footBonesFK[i], self.footBonesIK[i], foot)

            nod.createReverseNode(self.blend_att,  con+".target[0].targetWeight")
            pm.connectAttr(self.blend_att, con+".target[1].targetWeight", f=True)
        

        # Single ik solvers
        pri.addIkHandle(self.root, self.getName("ikHandle1"), [self.footBonesIK[0], self.footBonesIK[1]] , "ikSCsolver")
        pri.addIkHandle(self.root, self.getName("ikHandle2"), [self.footBonesIK[1], self.footBonesIK[2]] , "ikSCsolver")


        return
        # Visibilities -------------------------------------

        # ik
        if self.settings["useRollCtl"]:
            for shp in self.roll_ctl.getShapes():
                pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for bk_ctl in self.bk_ctl:
            for shp in bk_ctl.getShapes():
                pm.connectAttr(self.blend_att, shp.attr("visibility"))

        for shp in self.heel_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.tip_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # Roll / Bank --------------------------------------
        if self.settings["useRollCtl"]:  # Using the controler
            self.roll_att = self.roll_ctl.attr("rz")
            self.bank_att = self.roll_ctl.attr("rx")

        clamp_node = node.createClampNode(
            [self.roll_att, self.bank_att, self.bank_att],
            [0, -180, 0],
            [180, 0, 180],
        )

        inAdd_nod = node.createAddNode(
            clamp_node + ".outputB",
            pm.getAttr(self.in_piv.attr("rx")) * self.n_factor,
        )

        pm.connectAttr(clamp_node + ".outputR", self.heel_loc.attr("rz"))
        pm.connectAttr(clamp_node + ".outputG", self.out_piv.attr("rx"))
        pm.connectAttr(inAdd_nod + ".output", self.in_piv.attr("rx"))

        # Reverse Controler offset -------------------------
        angle_outputs = node.createAddNodeMulti(self.angles_att)
        for i, bk_loc in enumerate(reversed(self.bk_loc)):

            if i == 0:  # First
                input = self.roll_att
                min_input = self.angles_att[i]

            elif i == len(self.angles_att):  # Last
                sub_nod = node.createSubNode(
                    self.roll_att, angle_outputs[i - 1]
                )
                input = sub_nod + ".output"
                min_input = -360

            else:  # Others
                sub_nod = node.createSubNode(
                    self.roll_att, angle_outputs[i - 1]
                )
                input = sub_nod + ".output"
                min_input = self.angles_att[i]

            clamp_node = node.createClampNode(input, min_input, 0)

            add_node = node.createAddNode(
                clamp_node + ".outputR", bk_loc.getAttr("rz")
            )

            pm.connectAttr(add_node + ".output", bk_loc.attr("rz"))

        # Reverse compensation -----------------------------
        for i, fk_loc in enumerate(self.fk_loc):
            bk_ctl = self.bk_ctl[-i - 1]
            bk_loc = self.bk_loc[-i - 1]
            fk_ctl = self.fk_ctl[i]

            # Inverse Rotorder
            o_node = applyop.gear_inverseRotorder_op(bk_ctl, fk_ctl)
            pm.connectAttr(o_node + ".output", bk_loc.attr("ro"))
            pm.connectAttr(fk_ctl.attr("ro"), fk_loc.attr("ro"))
            attribute.lockAttribute(bk_ctl, "ro")

            # Compensate the backward rotation
            # ik
            addx_node = node.createAddNode(
                bk_ctl.attr("rx"), bk_loc.attr("rx")
            )
            addy_node = node.createAddNode(
                bk_ctl.attr("ry"), bk_loc.attr("ry")
            )
            addz_node = node.createAddNode(
                bk_ctl.attr("rz"), bk_loc.attr("rz")
            )
            addz_node = node.createAddNode(
                addz_node + ".output",
                -bk_loc.getAttr("rz") - fk_loc.getAttr("rz"),
            )

            neg_node = node.createMulNode(
                [
                    addx_node + ".output",
                    addy_node + ".output",
                    addz_node + ".output",
                ],
                [-1, -1, -1],
            )
            ik_outputs = [
                neg_node + ".outputX",
                neg_node + ".outputY",
                neg_node + ".outputZ",
            ]

            # fk
            fk_outputs = [0, 0, fk_loc.getAttr("rz")]

            # blend
            blend_node = node.createBlendNode(
                ik_outputs, fk_outputs, self.blend_att
            )
            pm.connectAttr(blend_node + ".output", fk_loc.attr("rotate"))

        return

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        return
        """Set the relation beetween object from guide to rig"""

        self.relatives["root"] = self.fk_ctl[0]
        self.relatives["heel"] = self.fk_ctl[0]
        self.relatives["inpivot"] = self.fk_ctl[0]
        self.relatives["outpivot"] = self.fk_ctl[0]

        self.controlRelatives["root"] = self.fk_ctl[0]
        self.controlRelatives["heel"] = self.fk_ctl[0]
        self.controlRelatives["inpivot"] = self.fk_ctl[0]
        self.controlRelatives["outpivot"] = self.fk_ctl[0]

        self.jointRelatives["root"] = 0
        self.jointRelatives["heel"] = 0
        self.jointRelatives["inpivot"] = 0
        self.jointRelatives["outpivot"] = 0

        for i in range(self.div_count):
            self.relatives["%s_loc" % i] = self.fk_ctl[i]
            self.jointRelatives["%s_loc" % i] = i

        if self.div_count > 0:
            self.relatives["%s_loc" % self.div_count] = self.fk_ctl[-1]
            self.jointRelatives["%s_loc" % self.div_count] = self.div_count - 1

    def addConnection(self):
        self.connections["vanilla_leg_2jnt_01"] = self.connect_vanilla_leg_2jnt_01

    def connect_vanilla_leg_2jnt_01(self):
        if self.parent_comp is None:
            return

        pm.connectAttr(self.parent_comp.blend_att, self.blend_att)

        return
        pm.parent(self.root, self.parent_comp.ik_ctl)
        pm.parent(self.parent_comp.ik_ref, self.bk_ctl[-1])
        pm.parent(self.parent_comp.match_fk2, self.bk_ctl[-1])
        pm.parentConstraint(
            self.parent_comp.tws2_rot, self.fk_ref, maintainOffset=True
        )

        # add message connections to parent component.
        # this connection will be used to track the ctl relation when IK/FK
        # match is performed
        self.parent_comp.root.addAttr("footCnx", at="message", m=False)
        self.root.message >> self.parent_comp.root.footCnx

        return