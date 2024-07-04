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
        """Add all the objects needed to create the component."""

        self.WIP = self.options["mode"]
        self.up_axis = pm.upAxis(q=True, axis=True)

        self.blade_normal = self.guide.blades["blade"].z * -1
        self.blade_binormal = self.guide.blades["blade"].x

        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        self.length0 = vector.getDistance(
            self.guide.apos[0], self.guide.apos[1]
        )
        self.length1 = vector.getDistance(
            self.guide.apos[1], self.guide.apos[2]
        )
        self.length2 = vector.getDistance(
            self.guide.apos[2], self.guide.apos[3]
        )

        # 1 bone chain for upv ref
        self.armChainUpvRef = primitive.add2DChain(
            self.root,
            self.getName("armUpvRef%s_jnt"),
            [self.guide.apos[0], self.guide.apos[2]],
            self.normal,
            False,
            self.WIP,
        )

        negateOri = self.armChainUpvRef[1].getAttr("jointOrientZ") * -1
        self.armChainUpvRef[1].setAttr("jointOrientZ", negateOri)

        # FK Controlers -----------------------------------
        t = transform.getTransformLookingAt(
            self.guide.apos[0],
            self.guide.apos[1],
            self.normal,
            "xz",
            self.negate,
        )

        if self.settings["FK_rest_T_Pose"]:
            x = datatypes.Vector(1, 0, 0)
            if self.negate:
                z_dir = -1
            else:
                z_dir = 1

            if self.up_axis == "y":
                z = datatypes.Vector(0, z_dir, 0)
            else:
                z = datatypes.Vector(0, 0, z_dir)

            t_npo = transform.getRotationFromAxis(x, z, "xz", False)
            t_npo = transform.setMatrixPosition(t_npo, self.guide.apos[0])
        else:
            t_npo = t

        self.fk0_npo = primitive.addTransform(
            self.root, self.getName("fk0_npo"), t_npo
        )
        vec_po = datatypes.Vector(0.5 * self.length0 * self.n_factor, 0, 0)
        self.fk0_ctl = self.addCtl(
            self.fk0_npo,
            "fk0_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length0,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=vec_po,
            tp=self.parentCtlTag,
        )

        attribute.setKeyableAttributes(
            self.fk0_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"]
        )

        t = transform.getTransformLookingAt(
            self.guide.apos[1],
            self.guide.apos[2],
            self.normal,
            "xz",
            self.negate,
        )

        if self.settings["FK_rest_T_Pose"]:
            t_npo = transform.setMatrixPosition(
                transform.getTransform(self.fk0_ctl), self.guide.apos[1]
            )
        else:
            t_npo = t

        self.fk1_npo = primitive.addTransform(
            self.fk0_ctl, self.getName("fk1_npo"), t_npo
        )

        vec_po = datatypes.Vector(0.5 * self.length1 * self.n_factor, 0, 0)
        self.fk1_ctl = self.addCtl(
            self.fk1_npo,
            "fk1_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length1,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=vec_po,
            tp=self.fk0_ctl,
        )

        attribute.setKeyableAttributes(
            self.fk1_ctl, ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"]
        )

        if self.settings["use_blade"]:
            t = transform.getTransformLookingAt(
                self.guide.apos[2],
                self.guide.apos[3],
                self.blade_normal,
                "x-z",
                self.negate,
            )
        else:
            t = transform.getTransformLookingAt(
                self.guide.apos[2],
                self.guide.apos[3],
                self.normal,
                "xz",
                self.negate,
            )

        if self.settings["FK_rest_T_Pose"]:
            t_npo = transform.setMatrixPosition(
                transform.getTransform(self.fk1_ctl), self.guide.apos[2]
            )

        else:
            t_npo = t

        self.fk2_npo = primitive.addTransform(
            self.fk1_ctl, self.getName("fk2_npo"), t_npo
        )

        vec_po = datatypes.Vector(0.5 * self.length2 * self.n_factor, 0, 0)
        self.fk2_ctl = self.addCtl(
            self.fk2_npo,
            "fk2_ctl",
            t,
            self.color_fk,
            "cube",
            w=self.length2,
            h=self.size * 0.1,
            d=self.size * 0.1,
            po=vec_po,
            tp=self.fk1_ctl,
        )

        attribute.setKeyableAttributes(self.fk2_ctl)

        self.fk_ctl = [self.fk0_ctl, self.fk1_ctl, self.fk2_ctl]

        for x in self.fk_ctl:
            attribute.setInvertMirror(x, ["tx", "ty", "tz"])

        # IK upv ---------------------------------

        # create tip point
        self.tip_ref = primitive.addTransform(
            self.armChainUpvRef[0],
            self.getName("tip_ref"),
            self.armChainUpvRef[0].getMatrix(worldSpace=True),
        )

        # create interpolate obj
        self.interpolate_lvl = primitive.addTransform(
            self.armChainUpvRef[0],
            self.getName("int_lvl"),
            self.armChainUpvRef[0].getMatrix(worldSpace=True),
        )

        # create roll npo and ctl
        self.roll_ctl_npo = primitive.addTransform(
            self.root,
            self.getName("roll_ctl_npo"),
            self.armChainUpvRef[0].getMatrix(worldSpace=True),
        )
        if self.negate:
            off_x = -1.5708
        else:
            off_x = 1.5708
        off_y = 1.5708

        self.roll_ctl = self.addCtl(
            self.roll_ctl_npo,
            "roll_ctl",
            transform.getTransform(self.roll_ctl_npo),
            self.color_ik,
            "compas",
            w=self.size * 0.3,
            ro=datatypes.Vector([off_x, off_y, 0]),
            tp=self.parentCtlTag,
        )
        attribute.setKeyableAttributes(self.roll_ctl, ["rx"])
        # create upv control
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size * 0.8
        v += self.guide.apos[1]

        self.upv_cns = primitive.addTransformFromPos(
            self.root, self.getName("upv_cns"), v
        )

        self.upv_ctl = self.addCtl(
            self.upv_cns,
            "upv_ctl",
            transform.getTransform(self.upv_cns),
            self.color_ik,
            "diamond",
            w=self.size * 0.12,
            tp=self.parentCtlTag,
        )

        if self.settings["mirrorMid"]:
            if self.negate:
                self.upv_cns.rz.set(180)
                self.upv_cns.sy.set(-1)
        else:
            attribute.setInvertMirror(self.upv_ctl, ["tx"])
        attribute.setKeyableAttributes(self.upv_ctl, self.t_params)

        # IK Controlers -----------------------------------

        self.ik_cns = primitive.addTransformFromPos(
            self.root, self.getName("ik_cns"), self.guide.pos["wrist"]
        )

        t = transform.getTransformFromPos(self.guide.pos["wrist"])
        self.ikcns_ctl = self.addCtl(
            self.ik_cns,
            "ikcns_ctl",
            t,
            self.color_ik,
            "null",
            w=self.size * 0.12,
            tp=self.parentCtlTag,
        )

        attribute.setInvertMirror(self.ikcns_ctl, ["tx", "ty", "tz"])

        if self.settings["use_blade"]:
            normal = self.blade_normal
            axis_ref1 = "xz"
            axis_ref2 = "x-z"
        else:
            normal = self.normal
            axis_ref1 = "x-y"
            axis_ref2 = "xy"

        if self.negate:

            m = transform.getTransformLookingAt(
                self.guide.pos["wrist"],
                self.guide.pos["eff"],
                normal,
                axis_ref1,
                True,
            )
        else:

            m = transform.getTransformLookingAt(
                self.guide.pos["wrist"],
                self.guide.pos["eff"],
                normal,
                axis_ref2,
                False,
            )

        self.ik_ctl = self.addCtl(
            self.ikcns_ctl,
            "ik_ctl",
            m,
            self.color_ik,
            "cube",
            w=self.size * 0.12,
            h=self.size * 0.12,
            d=self.size * 0.12,
            tp=self.roll_ctl,
        )

        if self.settings["mirrorIK"]:
            if self.negate:
                self.ik_cns.sx.set(-1)
                self.ik_ctl.ry.set(self.ik_ctl.ry.get() * -1)
                self.ik_ctl.rz.set(self.ik_ctl.rz.get() * -1)
        else:
            attribute.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        attribute.setKeyableAttributes(self.ik_ctl)
        self.ik_ctl_ref = primitive.addTransform(
            self.ik_ctl, self.getName("ikCtl_ref"), m
        )

        # IK rotation controls
        if self.settings["ikTR"]:
            self.ikRot_npo = primitive.addTransform(
                self.root, self.getName("ikRot_npo"), m
            )
            self.ikRot_cns = primitive.addTransform(
                self.ikRot_npo, self.getName("ikRot_cns"), m
            )
            self.ikRot_ctl = self.addCtl(
                self.ikRot_cns,
                "ikRot_ctl",
                m,
                self.color_ik,
                "sphere",
                w=self.size * 0.12,
                tp=self.ik_ctl,
            )

            attribute.setKeyableAttributes(self.ikRot_ctl, self.r_params)

        # References --------------------------------------
        # Calculate  again the transfor for the IK ref. This way align with FK

        if self.settings["use_blade"]:
            normal = self.blade_normal
            axis_ref = "x-z"
        else:
            normal = self.normal
            axis_ref = "xz"
        trnIK_ref = transform.getTransformLookingAt(
            self.guide.pos["wrist"],
            self.guide.pos["eff"],
            normal,
            axis_ref,
            self.negate,
        )

        self.ik_ref = primitive.addTransform(
            self.ik_ctl_ref, self.getName("ik_ref"), trnIK_ref
        )
        self.fk_ref = primitive.addTransform(
            self.fk_ctl[2], self.getName("fk_ref"), trnIK_ref
        )

        # Chain --------------------------------------------
        # The outputs of the ikfk2bone solver
        self.bone0 = primitive.addLocator(
            self.root,
            self.getName("0_bone"),
            transform.getTransform(self.fk_ctl[0]),
        )
        self.bone0_shp = self.bone0.getShape()
        self.bone0_shp.setAttr("localPositionX", self.n_factor * 0.5)
        self.bone0_shp.setAttr("localScale", 0.5, 0, 0)
        self.bone0.setAttr("sx", self.length0)
        self.bone0.setAttr("visibility", False)

        self.bone1 = primitive.addLocator(
            self.root,
            self.getName("1_bone"),
            transform.getTransform(self.fk_ctl[1]),
        )
        self.bone1_shp = self.bone1.getShape()
        self.bone1_shp.setAttr("localPositionX", self.n_factor * 0.5)
        self.bone1_shp.setAttr("localScale", 0.5, 0, 0)
        self.bone1.setAttr("sx", self.length1)
        self.bone1.setAttr("visibility", False)

        self.ctrn_loc = primitive.addTransformFromPos(
            self.root, self.getName("ctrn_loc"), self.guide.apos[1]
        )
        self.eff_loc = primitive.addTransformFromPos(
            self.root, self.getName("eff_loc"), self.guide.apos[2]
        )
        # set the offset rotation for the hand
        self.off_t = transform.getTransformLookingAt(
            self.guide.pos["wrist"],
            self.guide.pos["eff"],
            self.blade_normal,
            axis="xy",
            negate=self.negate,
        )
        if self.settings["use_blade"]:
            # set the offset rotation for the hand
            self.off_t = transform.getTransformLookingAt(
                self.guide.pos["wrist"],
                self.guide.pos["eff"],
                self.blade_normal,
                axis="xy",
                negate=self.negate,
            )
            self.eff_jnt_off = primitive.addTransform(
                self.eff_loc, self.getName("eff_off"), self.off_t
            )

        # Mid Controler ------------------------------------
        t = transform.getTransform(self.ctrn_loc)

        self.mid_cns = primitive.addTransform(
            self.ctrn_loc, self.getName("mid_cns"), t
        )

        self.mid_ctl = self.addCtl(
            self.mid_cns,
            "mid_ctl",
            t,
            self.color_ik,
            "sphere",
            w=self.size * 0.2,
            tp=self.parentCtlTag,
        )

        attribute.setKeyableAttributes(
            self.mid_ctl,
            params=["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx"],
        )

        if self.settings["mirrorMid"]:
            if self.negate:
                self.mid_cns.rz.set(180)
                self.mid_cns.sz.set(-1)
            self.mid_ctl_twst_npo = primitive.addTransform(
                self.mid_ctl, self.getName("mid_twst_npo"), t
            )
            self.mid_ctl_twst_ref = primitive.addTransform(
                self.mid_ctl_twst_npo, self.getName("mid_twst_ref"), t
            )
        else:
            self.mid_ctl_twst_ref = self.mid_ctl
            attribute.setInvertMirror(self.mid_ctl, ["tx", "ty", "tz"])

        # Roll join ref
        self.rollRef = primitive.add2DChain(
            self.root,
            self.getName("rollChain"),
            self.guide.apos[:2],
            self.normal,
            self.negate,
        )
        t = transform.getTransformLookingAt(
            self.guide.pos["base"],
            self.guide.apos[1],
            self.normal,
            "xz",
            self.negate,
        )
        for x in self.rollRef:
            x.setAttr("visibility", False)

        self.tws0_loc = primitive.addTransform(
            self.root, self.getName("tws0_loc"), t
        )
        self.tws0_rot = primitive.addTransform(
            self.tws0_loc, self.getName("tws0_rot"), t
        )

        self.tws1_npo = primitive.addTransform(
            self.ctrn_loc,
            self.getName("tws1_npo"),
            transform.getTransform(self.ctrn_loc),
        )
        self.tws1_loc = primitive.addTransform(
            self.tws1_npo,
            self.getName("tws1_loc"),
            transform.getTransform(self.ctrn_loc),
        )
        self.tws1_rot = primitive.addTransform(
            self.tws1_loc,
            self.getName("tws1_rot"),
            transform.getTransform(self.ctrn_loc),
        )

        self.tws2_npo = primitive.addTransform(
            self.root,
            self.getName("tws2_npo"),
            transform.getTransform(self.fk_ctl[2]),
        )
        self.tws2_loc = primitive.addTransform(
            self.tws2_npo,
            self.getName("tws2_loc"),
            transform.getTransform(self.fk_ctl[2]),
        )
        self.tws2_rot = primitive.addTransform(
            self.tws2_loc,
            self.getName("tws2_rot"),
            transform.getTransform(self.fk_ctl[2]),
        )

        # Divisions ----------------------------------------

        self.divisions = self.settings["div0"] + self.settings["div1"] + 2

        self.div_cns = []

        if self.settings["extraTweak"]:
            tagP = self.parentCtlTag
            self.tweak_ctl = []

        for i in range(self.divisions):

            div_cns = primitive.addTransform(
                self.root, self.getName("div%s_loc" % i)
            )

            self.div_cns.append(div_cns)

            if self.settings["extraTweak"]:
                t = transform.getTransform(div_cns)
                tweak_ctl = self.addCtl(
                    div_cns,
                    "tweak%s_ctl" % i,
                    t,
                    self.color_fk,
                    "square",
                    w=self.size * 0.15,
                    d=self.size * 0.15,
                    ro=datatypes.Vector([0, 0, 1.5708]),
                    tp=tagP,
                )
                attribute.setKeyableAttributes(tweak_ctl)

                tagP = tweak_ctl
                self.tweak_ctl.append(tweak_ctl)
                driver = tweak_ctl
            else:
                driver = div_cns

            # setting the joints
            if i == 0:
                self.jnt_pos.append(
                    {
                        "obj": driver,
                        "name": "upperarm",
                        "guide_relative": "root",
                        "data_contracts": "Ik",
                    }
                )
                current_parent = "root"
                twist_name = "upperarm_twist_"
                twist_idx = 1
                increment = 1
            elif i == self.settings["div0"] + 1:
                self.jnt_pos.append(
                    {
                        "obj": driver,
                        "name": "lowerarm",
                        "newActiveJnt": current_parent,
                        "guide_relative": "elbow",
                        "data_contracts": "Ik",
                    }
                )
                twist_name = "lowerarm_twist_"
                current_parent = "elbow"
                twist_idx = self.settings["div1"]
                increment = -1
            else:
                self.jnt_pos.append(
                    {
                        "obj": driver,
                        "name": twist_name + str(twist_idx).zfill(2),
                        "newActiveJnt": current_parent,
                        "data_contracts": "Twist,Squash",
                    }
                )
                twist_idx += increment

        if self.settings["use_blade"]:
            eff_loc = self.eff_jnt_off
        else:
            eff_loc = self.eff_loc
        self.jnt_pos.append(
            {
                "obj": eff_loc,
                "name": "hand",
                "newActiveJnt": current_parent,
                "guide_relative": "wrist",
                "data_contracts": "Ik",
            }
        )

        # match IK FK references
        self.match_fk0_off = self.add_match_ref(
            self.fk_ctl[1], self.root, "matchFk0_npo", False
        )

        self.match_fk0 = self.add_match_ref(
            self.fk_ctl[0], self.match_fk0_off, "fk0_mth"
        )

        self.match_fk1_off = self.add_match_ref(
            self.fk_ctl[2], self.root, "matchFk1_npo", False
        )

        self.match_fk1 = self.add_match_ref(
            self.fk_ctl[1], self.match_fk1_off, "fk1_mth"
        )

        if self.settings["ikTR"]:
            reference = self.ikRot_ctl

            self.match_ikRot = self.add_match_ref(
                self.ikRot_ctl, self.fk2_ctl, "ikRot_mth"
            )
        else:
            reference = self.ik_ctl

        self.match_fk2 = self.add_match_ref(
            self.fk_ctl[2], reference, "fk2_mth"
        )

        self.match_ik = self.add_match_ref(self.ik_ctl, self.fk2_ctl, "ik_mth")

        self.match_ikUpv = self.add_match_ref(
            self.upv_ctl, self.fk0_ctl, "upv_mth"
        )

        # add visual reference
        self.line_ref = icon.connection_display_curve(
            self.getName("visalRef"), [self.upv_ctl, self.mid_ctl]
        )

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        """Create the anim and setupr rig attributes for the component"""

        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam(
            "blend", "Fk/Ik Blend", "double", self.settings["blend"], 0, 1
        )
        self.roll_att = self.addAnimParam(
            "roll", "Roll", "double", 0, -180, 180
        )
        self.armpit_roll_att = self.addAnimParam(
            "aproll", "Armpit Roll", "double", 0, -360, 360
        )

        self.scale_att = self.addAnimParam(
            "ikscale", "Scale", "double", 1, 0.001, 99
        )
        self.maxstretch_att = self.addAnimParam(
            "maxstretch",
            "Max Stretch",
            "double",
            self.settings["maxstretch"],
            1,
            99,
        )
        self.slide_att = self.addAnimParam(
            "slide", "Slide", "double", 0.5, 0, 1
        )
        self.softness_att = self.addAnimParam(
            "softness", "Softness", "double", 0, 0, 1
        )
        self.reverse_att = self.addAnimParam(
            "reverse", "Reverse", "double", 0, 0, 1
        )
        self.roundness_att = self.addAnimParam(
            "roundness", "Roundness", "double", 0, 0, self.size
        )
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1
        )

        if self.settings["extraTweak"]:
            self.tweakVis_att = self.addAnimParam(
                "Tweak_vis", "Tweak Vis", "bool", False
            )

        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(",")
            )

            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref", "Ik Ref", 0, ref_names
                )

        if self.settings["ikTR"]:
            ref_names = ["Auto", "ik_ctl"]
            if self.settings["ikrefarray"]:
                ref_names = ref_names + self.get_valid_alias_list(
                    self.settings["ikrefarray"].split(",")
                )

            self.ikRotRef_att = self.addAnimEnumParam(
                "ikRotRef", "Ik Rot Ref", 0, ref_names
            )

        if self.settings["upvrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["upvrefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.upvref_att = self.addAnimEnumParam(
                    "upvref", "UpV Ref", 0, ref_names
                )

        if self.settings["pinrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["pinrefarray"].split(",")
            )
            ref_names = ["Auto"] + ref_names
            if len(ref_names) > 1:
                self.pin_att = self.addAnimEnumParam(
                    "elbowref", "Elbow Ref", 0, ref_names
                )

        if self.validProxyChannels:
            attrs_list = [self.blend_att, self.roundness_att]
            if self.settings["extraTweak"]:
                attrs_list += [self.tweakVis_att]
            attribute.addProxyAttribute(
                attrs_list,
                [
                    self.fk0_ctl,
                    self.fk1_ctl,
                    self.fk2_ctl,
                    self.ik_ctl,
                    self.upv_ctl,
                    self.mid_ctl,
                ],
            )
            attribute.addProxyAttribute(
                self.roll_att, [self.ik_ctl, self.upv_ctl]
            )

        # Setup ------------------------------------------
        # Eval Fcurve
        if self.guide.paramDefs["st_profile"].value:
            self.st_value = self.guide.paramDefs["st_profile"].value
            self.sq_value = self.guide.paramDefs["sq_profile"].value
        else:
            self.st_value = fcurve.getFCurveValues(
                self.settings["st_profile"], self.divisions
            )
            self.sq_value = fcurve.getFCurveValues(
                self.settings["sq_profile"], self.divisions
            )

        self.st_att = [
            self.addSetupParam(
                "stretch_%s" % i,
                "Stretch %s" % i,
                "double",
                self.st_value[i],
                -1,
                0,
            )
            for i in range(self.divisions)
        ]

        self.sq_att = [
            self.addSetupParam(
                "squash_%s" % i,
                "Squash %s" % i,
                "double",
                self.sq_value[i],
                0,
                1,
            )
            for i in range(self.divisions)
        ]

        self.resample_att = self.addSetupParam(
            "resample", "Resample", "bool", True
        )
        self.absolute_att = self.addSetupParam(
            "absolute", "Absolute", "bool", False
        )

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.

        """
        # 1 bone chain Upv ref ==============================================
        self.ikHandleUpvRef = primitive.addIkHandle(
            self.root,
            self.getName("ikHandleArmChainUpvRef"),
            self.armChainUpvRef,
            "ikSCsolver",
        )
        pm.pointConstraint(self.ik_ctl, self.ikHandleUpvRef)
        p_cns = pm.parentConstraint(
            self.armChainUpvRef[0], self.upv_cns, mo=True
        )
        p_cns.interpType.set(0)

        # Visibilities -------------------------------------
        # fk
        fkvis_node = node.createReverseNode(self.blend_att)
        try:
            for shp in self.fk0_ctl.getShapes():
                pm.connectAttr(fkvis_node.outputX, shp.attr("visibility"))
            for shp in self.fk1_ctl.getShapes():
                pm.connectAttr(fkvis_node.outputX, shp.attr("visibility"))
            for shp in self.fk2_ctl.getShapes():
                pm.connectAttr(fkvis_node.outputX, shp.attr("visibility"))
        except RuntimeError:
            pm.displayInfo("Visibility already connected")

        # ik
        for shp in self.upv_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ikcns_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ik_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.line_ref.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        if self.settings["ikTR"]:
            for shp in self.ikRot_ctl.getShapes():
                pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.roll_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # Controls ROT order -----------------------------------
        attribute.setRotOrder(self.fk0_ctl, "XZY")
        attribute.setRotOrder(self.fk1_ctl, "XYZ")
        attribute.setRotOrder(self.fk2_ctl, "YZX")
        attribute.setRotOrder(self.ik_ctl, "XYZ")

        # IK Solver -----------------------------------------
        out = [self.bone0, self.bone1, self.ctrn_loc, self.eff_loc]
        o_node = applyop.gear_ikfk2bone_op(
            out,
            self.root,
            self.ik_ref,
            self.upv_ctl,
            self.fk_ctl[0],
            self.fk_ctl[1],
            self.fk_ref,
            self.length0,
            self.length1,
            self.negate,
        )

        # NOTE: Ideally we should not change hierarchy or move object after
        # object generation method. But is much easier this way since every
        # part is in the final and correct position
        # after the  ctrn_loc is in the correct position with the ikfk2bone op

        # point constrain tip reference
        pm.pointConstraint(self.ik_ctl, self.tip_ref, mo=False)

        # interpolate transform  mid point locator
        int_matrix = applyop.gear_intmatrix_op(
            self.armChainUpvRef[0].attr("worldMatrix"),
            self.tip_ref.attr("worldMatrix"),
            0.5,
        )
        applyop.gear_mulmatrix_op(
            int_matrix.attr("output"),
            self.interpolate_lvl.attr("parentInverseMatrix[0]"),
            self.interpolate_lvl,
        )

        # match roll ctl npo to ctrn_loc current transform (so correct orient)
        transform.matchWorldTransform(self.ctrn_loc, self.roll_ctl_npo)

        # match roll ctl npo to interpolate transform current position
        pos = self.interpolate_lvl.getTranslation(space="world")
        self.roll_ctl_npo.setTranslation(pos, space="world")

        # parent constraint roll control npo to interpolate trans
        pm.parentConstraint(self.interpolate_lvl, self.roll_ctl_npo, mo=True)

        if self.settings["ikTR"]:
            # connect the control inputs
            outEff_dm = o_node.listConnections(c=True)[-1][1]

            inAttr = self.ikRot_npo.attr("translate")
            outEff_dm.attr("outputTranslate") >> inAttr

            outEff_dm.attr("outputScale") >> self.ikRot_npo.attr("scale")
            dm_node = node.createDecomposeMatrixNode(o_node.attr("outB"))
            dm_node.attr("outputRotate") >> self.ikRot_npo.attr("rotate")

            # rotation
            mulM_node = applyop.gear_mulmatrix_op(
                self.ikRot_ctl.attr("worldMatrix"),
                self.eff_loc.attr("parentInverseMatrix"),
            )
            intM_node = applyop.gear_intmatrix_op(
                o_node.attr("outEff"),
                mulM_node.attr("output"),
                o_node.attr("blend"),
            )
            dm_node = node.createDecomposeMatrixNode(intM_node.attr("output"))
            dm_node.attr("outputRotate") >> self.eff_loc.attr("rotate")
            transform.matchWorldTransform(self.fk2_ctl, self.ikRot_cns)

        # scale: this fix the scalin popping issue
        intM_node = applyop.gear_intmatrix_op(
            self.fk2_ctl.attr("worldMatrix"),
            self.ik_ctl_ref.attr("worldMatrix"),
            o_node.attr("blend"),
        )
        mulM_node = applyop.gear_mulmatrix_op(
            intM_node.attr("output"), self.eff_loc.attr("parentInverseMatrix")
        )
        dm_node = node.createDecomposeMatrixNode(mulM_node.attr("output"))
        dm_node.attr("outputScale") >> self.eff_loc.attr("scale")

        pm.connectAttr(self.blend_att, o_node + ".blend")
        if self.negate:
            mulVal = -1
            rollMulVal = 1
        else:
            mulVal = 1
            rollMulVal = -1
        roll_m_node = node.createMulNode(self.roll_att, mulVal)
        roll_m_node2 = node.createMulNode(self.roll_ctl.attr("rx"), rollMulVal)
        node.createPlusMinusAverage1D(
            [roll_m_node.outputX, roll_m_node2.outputX],
            operation=1,
            output=o_node + ".roll",
        )
        pm.connectAttr(self.scale_att, o_node + ".scaleA")
        pm.connectAttr(self.scale_att, o_node + ".scaleB")
        pm.connectAttr(self.maxstretch_att, o_node + ".maxstretch")
        pm.connectAttr(self.slide_att, o_node + ".slide")
        pm.connectAttr(self.softness_att, o_node + ".softness")
        pm.connectAttr(self.reverse_att, o_node + ".reverse")

        # Twist references ---------------------------------

        pm.pointConstraint(
            self.mid_ctl_twst_ref, self.tws1_npo, maintainOffset=False
        )
        pm.connectAttr(self.mid_ctl.scaleX, self.tws1_loc.scaleX)
        pm.orientConstraint(
            self.mid_ctl_twst_ref, self.tws1_npo, maintainOffset=False
        )

        o_node = applyop.gear_mulmatrix_op(
            self.eff_loc.attr("worldMatrix"),
            self.root.attr("worldInverseMatrix"),
        )
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(
            dm_node + ".outputTranslate", self.tws2_npo.attr("translate")
        )

        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node + ".outputRotate", self.tws2_npo.attr("rotate"))

        o_node = applyop.gear_mulmatrix_op(
            self.eff_loc.attr("worldMatrix"),
            self.tws2_rot.attr("parentInverseMatrix"),
        )
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        attribute.setRotOrder(self.tws2_rot, "XYZ")
        pm.connectAttr(dm_node + ".outputRotate", self.tws2_rot + ".rotate")

        if self.settings["div0"]:
            ori_ref = self.rollRef[0]
        else:
            ori_ref = self.bone0

        applyop.oriCns(ori_ref, self.tws0_loc, maintainOffset=True)

        self.tws0_rot.setAttr("sx", 0.001)
        self.tws2_rot.setAttr("sx", 0.001)

        add_node = node.createAddNode(self.roundness_att, 0.00001)
        pm.connectAttr(add_node + ".output", self.tws1_rot.attr("sx"))

        pm.connectAttr(self.armpit_roll_att, self.tws0_rot + ".rotateX")

        # Roll Shoulder
        applyop.splineIK(
            self.getName("rollRef"),
            self.rollRef,
            parent=self.root,
            cParent=self.bone0,
        )

        # Volume -------------------------------------------
        distA_node = node.createDistNode(self.tws0_loc, self.tws1_loc)
        distB_node = node.createDistNode(self.tws1_loc, self.tws2_loc)
        add_node = node.createAddNode(
            distA_node + ".distance", distB_node + ".distance"
        )
        div_node = node.createDivNode(
            add_node + ".output", self.root.attr("sx")
        )

        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(self.root.attr("worldMatrix"), dm_node + ".inputMatrix")

        div_node2 = node.createDivNode(
            div_node + ".outputX", dm_node + ".outputScaleX"
        )
        self.volDriver_att = div_node2 + ".outputX"

        if self.settings["extraTweak"]:
            for tweak_ctl in self.tweak_ctl:
                for shp in tweak_ctl.getShapes():
                    pm.connectAttr(self.tweakVis_att, shp.attr("visibility"))

        # Divisions ----------------------------------------
        # at 0 or 1 the division will follow exactly the rotation of the
        # controler.. and we wont have this nice tangent + roll
        for i, div_cns in enumerate(self.div_cns):

            if i < (self.settings["div0"] + 1):
                perc = i * 0.5 / (self.settings["div0"] + 1.0)
            elif i < (self.settings["div0"] + 2):
                perc = 0.501
            else:
                perc = 0.5 + (i - self.settings["div0"] - 1.0) * 0.5 / (
                    self.settings["div1"] + 1.0
                )

            perc = max(0.001, min(0.999, perc))

            # Roll
            if self.negate:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cns,
                    [self.tws2_rot, self.tws1_rot, self.tws0_rot],
                    1.0 - perc,
                    40,
                )
            else:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cns,
                    [self.tws0_rot, self.tws1_rot, self.tws2_rot],
                    perc,
                    40,
                )

            pm.connectAttr(self.resample_att, o_node + ".resample")
            pm.connectAttr(self.absolute_att, o_node + ".absolute")

            # Squash n Stretch
            o_node = applyop.gear_squashstretch2_op(
                div_cns, None, pm.getAttr(self.volDriver_att), "x"
            )
            pm.connectAttr(self.volume_att, o_node + ".blend")
            pm.connectAttr(self.volDriver_att, o_node + ".driver")
            pm.connectAttr(self.st_att[i], o_node + ".stretch")
            pm.connectAttr(self.sq_att[i], o_node + ".squash")

        # match IK/FK ref
        pm.parentConstraint(self.bone0, self.match_fk0_off, mo=True)
        pm.parentConstraint(self.bone1, self.match_fk1_off, mo=True)
        if self.settings["ikTR"]:
            transform.matchWorldTransform(self.ikRot_ctl, self.match_ikRot)
            transform.matchWorldTransform(self.fk_ctl[2], self.match_fk2)

        # recover hand offset transform
        if self.settings["use_blade"]:
            self.eff_jnt_off.setMatrix(self.off_t, worldSpace=True)

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""

        self.relatives["root"] = self.div_cns[0]
        self.relatives["elbow"] = self.div_cns[self.settings["div0"] + 1]
        self.relatives["wrist"] = self.div_cns[-1]
        self.relatives["eff"] = self.eff_loc

        self.jointRelatives["root"] = 0
        self.jointRelatives["elbow"] = self.settings["div0"] + 1
        self.jointRelatives["wrist"] = -1
        self.jointRelatives["eff"] = -1
        # as an alternative option ingest all joint by index
        for x in range(len(self.jnt_pos)):
            self.jointRelatives[str(x)] = x

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["elbow"] = self.fk1_ctl
        self.controlRelatives["wrist"] = self.fk2_ctl
        self.controlRelatives["eff"] = self.fk2_ctl

        # here is really don't needed because the name is the same as the alias
        self.aliasRelatives["root"] = "root"
        self.aliasRelatives["elbow"] = "elbow"
        self.aliasRelatives["wrist"] = "wrist"
        self.aliasRelatives["eff"] = "hand"

    def addConnection(self):
        """Add more connection definition to the set"""

        self.connections["shoulder_01"] = self.connect_shoulder_01

    def connect_standard(self):
        """standard connection definition for the component"""

        if self.settings["ikTR"]:
            self.parent.addChild(self.root)
            self.connectRef(self.settings["ikrefarray"], self.ik_cns)
            self.connectRef(self.settings["upvrefarray"], self.upv_cns, True)

            init_refNames = ["lower_arm", "ik_ctl"]
            self.connectRef2(
                self.settings["ikrefarray"],
                self.ikRot_cns,
                self.ikRotRef_att,
                [self.ikRot_npo, self.ik_ctl],
                True,
                init_refNames,
            )
        else:
            self.connect_standardWithIkRef()

        if self.settings["pinrefarray"]:
            self.connectRef2(
                self.settings["pinrefarray"],
                self.mid_cns,
                self.pin_att,
                [self.ctrn_loc],
                False,
                ["Auto"],
            )

    def connect_shoulder_01(self):
        """Custom connection to be use with shoulder 01 component"""
        self.connect_standard()
        pm.parent(self.rollRef[0], self.ikHandleUpvRef, self.parent_comp.ctl)
