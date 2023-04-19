# MGEAR is under the terms of the MIT License

# Copyright (c) 2016 Jeremie Passerin, Miquel Campos
"""Guide Foot banking 01 module"""

from functools import partial
import pymel.core as pm

from mgear.shifter.component import guide
from mgear.core import transform, pyqt
from mgear.vendor.Qt import QtWidgets, QtCore

from maya.app.general.mayaMixin import MayaQWidgetDockableMixin
from maya.app.general.mayaMixin import MayaQDockWidget

from . import settingsUI as sui

# guide info
AUTHOR = "Jeremie Passerin, Miquel Campos"
URL = "www.jeremiepasserin.com, www.miquel-campos.com"
EMAIL = "geerem@hotmail.com, hello@miquel-campos.com"
VERSION = [1, 0, 0]
TYPE = "vanilla_foot_bk_01"
NAME = "foot"
DESCRIPTION = "Foot with reversed controllers to control foot roll."

##########################################################
# CLASS
##########################################################


class Guide(guide.ComponentGuide):
    """Component Guide Class"""

    compType = TYPE
    compName = NAME
    description = DESCRIPTION

    author = AUTHOR
    url = URL
    email = EMAIL
    version = VERSION

    connectors = ["vanilla_leg_2jnt_01"]

    def postInit(self):
        """Initialize the position for the guide"""
        self.save_transform = ["root", "ball", "toe", "heel"]

    def addObjects(self):
        """Add the Guide Root, blade and locators"""
        self.root = self.addRoot()

        vTemp = transform.getOffsetPosition(self.root, [0,-0.55, 1])
        self.ball = self.addLoc("ball", self.root, vTemp)
        vTemp = transform.getOffsetPosition(self.root, [0, -1, 2])
        self.toe = self.addLoc("toe", self.ball, vTemp)

        centers = [self.root, self.ball, self.toe]
        self.dispcrv = self.addDispCurve("crv", centers)

        vTemp = transform.getOffsetPosition(self.root, [0, -1, -1])
        self.heel = self.addLoc("heel", self.root, vTemp)

        centers = [self.root, self.heel]
        self.dispcrv = self.addDispCurve("crv", centers)

    def addParameters(self):
        """Add the configurations settings"""

        self.pRoll = self.addParam("useRollCtl", "bool", True)
        self.pUseIndex = self.addParam("useIndex", "bool", False)
        self.pRollAngle = self.addParam(
            "rollAngle", "double", -20, -180, 180)
        self.pParentJointIndex = self.addParam(
            "parentJointIndex", "long", -1, None, None)


##########################################################
# Setting Page
##########################################################

class settingsTab(QtWidgets.QDialog, sui.Ui_Form):
    """The Component settings UI"""

    def __init__(self, parent=None):
        super(settingsTab, self).__init__(parent)
        self.setupUi(self)


class componentSettings(MayaQWidgetDockableMixin, guide.componentMainSettings):
    """Create the component setting window"""

    def __init__(self, parent=None):
        self.toolName = TYPE
        # Delete old instances of the componet settings window.
        pyqt.deleteInstances(self, MayaQDockWidget)

        super(self.__class__, self).__init__(parent=parent)
        self.settingsTab = settingsTab()

        self.setup_componentSettingWindow()
        self.create_componentControls()
        self.populate_componentControls()
        self.create_componentLayout()
        self.create_componentConnections()

    def setup_componentSettingWindow(self):
        self.mayaMainWindow = pyqt.maya_main_window()

        self.setObjectName(self.toolName)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle(TYPE)
        self.resize(280, 350)

    def create_componentControls(self):
        return

    def populate_componentControls(self):
        """Populate Controls

        Populate the controls values from the custom attributes of the
        component.

        """
        # populate tab
        self.tabs.insertTab(1, self.settingsTab, "Component Settings")

        # populate component settings
        self.populateCheck(self.settingsTab.useRollCtl_checkBox, "useRollCtl")
        self.settingsTab.rollAngle_spinBox.setValue(
            self.root.attr("rollAngle").get())

        # populate connections in main settings
        for cnx in Guide.connectors:
            self.mainSettingsTab.connector_comboBox.addItem(cnx)
        cBox = self.mainSettingsTab.connector_comboBox
        self.connector_items = [cBox.itemText(i) for i in range(cBox.count())]
        currentConnector = self.root.attr("connector").get()
        if currentConnector not in self.connector_items:
            self.mainSettingsTab.connector_comboBox.addItem(currentConnector)
            self.connector_items.append(currentConnector)
            pm.displayWarning("The current connector: %s, is not a valid "
                              "connector for this component. "
                              "Build will Fail!!")
        comboIndex = self.connector_items.index(currentConnector)
        self.mainSettingsTab.connector_comboBox.setCurrentIndex(comboIndex)

    def create_componentLayout(self):

        self.settings_layout = QtWidgets.QVBoxLayout()
        self.settings_layout.addWidget(self.tabs)
        self.settings_layout.addWidget(self.close_button)

        self.setLayout(self.settings_layout)

    def create_componentConnections(self):

        self.settingsTab.useRollCtl_checkBox.stateChanged.connect(
            partial(self.updateCheck,
                    self.settingsTab.useRollCtl_checkBox,
                    "useRollCtl"))
        self.mainSettingsTab.connector_comboBox.currentIndexChanged.connect(
            partial(self.updateConnector,
                    self.mainSettingsTab.connector_comboBox,
                    self.connector_items))
        self.settingsTab.rollAngle_spinBox.valueChanged.connect(
            partial(self.updateSpinBox,
                    self.settingsTab.rollAngle_spinBox, "rollAngle"))

    def dockCloseEventTriggered(self):
        pyqt.deleteInstances(self, MayaQDockWidget)
