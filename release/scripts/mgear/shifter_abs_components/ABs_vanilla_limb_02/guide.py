from functools import partial
import pymel.core as pm

# mgear
#from mgear.shifter.component.guide import ComponentGuide
from mgear.shifter.component import guide

import mgear.core.transform as tra

#Pyside
from mgear.shifter.component.guide import componentMainSettings
import mgear.core.pyqt as gqt
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin
from maya.app.general.mayaMixin import MayaQDockWidget
from . import settingsUI as sui
QtGui, QtCore, QtWidgets, wrapInstance = gqt.qt_import()

# guide info
AUTHOR = "Miquel Campos"
URL = "www.miquel-campos.com"
EMAIL = ""
VERSION = [1,0,0]
TYPE = "ABs_vanilla_limb_02"
NAME = "limb"
DESCRIPTION = "2 bones arm/leg using only vanilla Maya nodes"

##########################################################
# CLASS
##########################################################
class Guide(guide.ComponentGuide):

    compType = TYPE
    compName = NAME
    description = DESCRIPTION

    author = AUTHOR
    url = URL
    email = EMAIL
    version = VERSION

    connectors = ["shoulder_01"]

    # =====================================================
    ##
    # @param self
    def postInit(self):
        self.save_transform = ["root", "mid", "tip", "eff"]

    # =====================================================
    ## Add more object to the object definition list.
    # @param self
    def addObjects(self):

        self.root = self.addRoot()

        vTemp = tra.getOffsetPosition( self.root, [3,0,-.01])
        self.mid = self.addLoc("mid", self.root, vTemp)
        vTemp = tra.getOffsetPosition( self.root, [6,0,0])
        self.tip = self.addLoc("tip", self.mid, vTemp)
        vTemp = tra.getOffsetPosition( self.root, [7,0,0])
        self.eff = self.addLoc("eff", self.tip, vTemp)

        self.dispcrv = self.addDispCurve("crv", [self.root, self.mid, self.tip, self.eff])

    # =====================================================
    ## Add more parameter to the parameter definition list.
    # @param self
    def addParameters(self):
        
        # Default Values
        self.pBlend       = self.addParam("blend", "double", 1, 0, 1)
        self.pIkRefArray  = self.addParam("ikrefarray", "string", "")
        self.pUpvRefArray = self.addParam("upvrefarray", "string", "")
        self.pUpvRefArray = self.addParam("pinrefarray", "string", "")
        self.pMaxStretch  = self.addParam("maxstretch", "double", 1.5 , 1, None)
        self.pIKTR       = self.addParam("ikTR", "bool", False)
        self.pMirrorMid = self.addParam("mirrorMid", "bool", False)
        self.pMirrorIK = self.addParam("mirrorIK", "bool", False)
        self.pWorldAlign       = self.addParam("WorldAlign", "bool", False)


        # # Divisions
        self.pDiv0 = self.addParam("div0", "long", 2, 0, None)
        self.pDiv1 = self.addParam("div1", "long", 2, 0, None)

        # # FCurves
        self.pSt_profile = self.addFCurveParam("st_profile",[[0, 0], [.5, -.5], [1, 0]])
        self.pSq_profile = self.addFCurveParam("sq_profile",[[0, 0], [.5, .5], [1, 0]])

        self.pUseIndex       = self.addParam("useIndex", "bool", False)
        self.pParentJointIndex = self.addParam("parentJointIndex", "long", -1, None, None)

    def get_divisions(self):
        """ Returns correct segments divisions """
        ej = 2
        self.divisions = self.root.div0.get() + self.root.div1.get() + 3 + ej

        return self.divisions

    def postDraw(self):
        "Add post guide draw elements to the guide"
        size = pm.xform(self.wrist, q=True, ws=True, scale=True)[0]
        self.add_ref_axis(self.wrist,
                          self.root.guideOrientWrist,
                          width=.5 / size)



##########################################################
# Setting Page
##########################################################

class settingsTab(QtWidgets.QDialog, sui.Ui_Form):

    def __init__(self, parent=None):
        super(settingsTab, self).__init__(parent)
        self.setupUi(self)


class componentSettings(MayaQWidgetDockableMixin, componentMainSettings):

    def __init__(self, parent = None):
        self.toolName = TYPE
        # Delete old instances of the componet settings window.
        gqt.deleteInstances(self, MayaQDockWidget)

        super(self.__class__, self).__init__(parent = parent)
        self.settingsTab = settingsTab()


        self.setup_componentSettingWindow()
        self.create_componentControls()
        self.populate_componentControls()
        self.create_componentLayout()
        self.create_componentConnections()

    def setup_componentSettingWindow(self):
        self.mayaMainWindow = gqt.maya_main_window()

        self.setObjectName(self.toolName)
        self.setWindowFlags(QtCore.Qt.Window)
        self.setWindowTitle(TYPE)
        self.resize(280, 780)

    def create_componentControls(self):
        return


    def populate_componentControls(self):
        """
        Populate the controls values from the custom attributes of the component.

        """
        #populate tab
        self.tabs.insertTab(1, self.settingsTab, "Component Settings")

        #populate component settings
        self.settingsTab.ikfk_slider.setValue(int(self.root.attr("blend").get()*100))
        self.settingsTab.ikfk_spinBox.setValue(int(self.root.attr("blend").get()*100))
        self.settingsTab.maxStretch_spinBox.setValue(self.root.attr("maxstretch").get())
        self.populateCheck(self.settingsTab.ikTR_checkBox, "ikTR")
        self.populateCheck(self.settingsTab.mirrorMid_checkBox, "mirrorMid")
        self.populateCheck(self.settingsTab.mirrorIK_checkBox, "mirrorIK")
        self.settingsTab.div0_spinBox.setValue(self.root.attr("div0").get())
        self.settingsTab.div1_spinBox.setValue(self.root.attr("div1").get())
        ikRefArrayItems = self.root.attr("ikrefarray").get().split(",")
        for item in ikRefArrayItems:
            self.settingsTab.ikRefArray_listWidget.addItem(item)
        upvRefArrayItems = self.root.attr("upvrefarray").get().split(",")
        for item in upvRefArrayItems:
            self.settingsTab.upvRefArray_listWidget.addItem(item)
        pinRefArrayItems = self.root.attr("pinrefarray").get().split(",")
        for item in pinRefArrayItems:
            self.settingsTab.pinRefArray_listWidget.addItem(item)

        #populate connections in main settings
        for cnx in Guide.connectors:
            self.mainSettingsTab.connector_comboBox.addItem(cnx)
        self.connector_items = [ self.mainSettingsTab.connector_comboBox.itemText(i) for i in range( self.mainSettingsTab.connector_comboBox.count())]
        currentConnector = self.root.attr("connector").get()
        if currentConnector not in self.connector_items:
            self.mainSettingsTab.connector_comboBox.addItem(currentConnector)
            self.connector_items.append(currentConnector)
            pm.displayWarning("The current connector: %s, is not a valid connector for this component. Build will Fail!!")
        comboIndex = self.connector_items.index(currentConnector)
        self.mainSettingsTab.connector_comboBox.setCurrentIndex(comboIndex)


    def create_componentLayout(self):

        self.settings_layout = QtWidgets.QVBoxLayout()
        self.settings_layout.addWidget(self.tabs)
        self.settings_layout.addWidget(self.close_button)

        self.setLayout(self.settings_layout)

    def create_componentConnections(self):

        self.settingsTab.ikfk_slider.valueChanged.connect(partial(self.updateSlider, self.settingsTab.ikfk_slider, "blend"))
        self.settingsTab.ikfk_spinBox.valueChanged.connect(partial(self.updateSlider, self.settingsTab.ikfk_spinBox, "blend"))
        self.settingsTab.maxStretch_spinBox.valueChanged.connect(partial(self.updateSpinBox, self.settingsTab.maxStretch_spinBox, "maxstretch"))
        self.settingsTab.div0_spinBox.valueChanged.connect(partial(self.updateSpinBox, self.settingsTab.div0_spinBox, "div0"))
        self.settingsTab.div1_spinBox.valueChanged.connect(partial(self.updateSpinBox, self.settingsTab.div1_spinBox, "div1"))
        self.settingsTab.squashStretchProfile_pushButton.clicked.connect(self.setProfile)
        self.settingsTab.ikTR_checkBox.stateChanged.connect(partial(self.updateCheck, self.settingsTab.ikTR_checkBox, "ikTR"))
        self.settingsTab.mirrorMid_checkBox.stateChanged.connect(partial(self.updateCheck, self.settingsTab.mirrorMid_checkBox, "mirrorMid"))
        self.settingsTab.mirrorIK_checkBox.stateChanged.connect(partial(self.updateCheck, self.settingsTab.mirrorIK_checkBox, "mirrorIK"))

        self.settingsTab.ikRefArrayAdd_pushButton.clicked.connect(partial(self.addItem2listWidget, self.settingsTab.ikRefArray_listWidget, "ikrefarray"))
        self.settingsTab.ikRefArrayRemove_pushButton.clicked.connect(partial(self.removeSelectedFromListWidget, self.settingsTab.ikRefArray_listWidget, "ikrefarray"))
        self.settingsTab.ikRefArray_copyRef_pushButton.clicked.connect(partial(self.copyFromListWidget, self.settingsTab.upvRefArray_listWidget, self.settingsTab.ikRefArray_listWidget, "ikrefarray"))
        self.settingsTab.ikRefArray_listWidget.installEventFilter(self)

        self.settingsTab.upvRefArrayAdd_pushButton.clicked.connect(partial(self.addItem2listWidget, self.settingsTab.upvRefArray_listWidget, "upvrefarray"))
        self.settingsTab.upvRefArrayRemove_pushButton.clicked.connect(partial(self.removeSelectedFromListWidget, self.settingsTab.upvRefArray_listWidget, "upvrefarray"))
        self.settingsTab.upvRefArray_copyRef_pushButton.clicked.connect(partial(self.copyFromListWidget, self.settingsTab.ikRefArray_listWidget, self.settingsTab.upvRefArray_listWidget, "upvrefarray"))
        self.settingsTab.upvRefArray_listWidget.installEventFilter(self)

        self.settingsTab.pinRefArrayAdd_pushButton.clicked.connect(partial(self.addItem2listWidget, self.settingsTab.pinRefArray_listWidget, "pinrefarray"))
        self.settingsTab.pinRefArrayRemove_pushButton.clicked.connect(partial(self.removeSelectedFromListWidget, self.settingsTab.pinRefArray_listWidget, "pinrefarray"))
        self.settingsTab.pinRefArray_copyRef_pushButton.clicked.connect(partial(self.copyFromListWidget, self.settingsTab.ikRefArray_listWidget, self.settingsTab.pinRefArray_listWidget, "pinrefarray"))
        self.settingsTab.pinRefArray_listWidget.installEventFilter(self)

    def eventFilter(self, sender, event):
        if event.type() == QtCore.QEvent.ChildRemoved:
            if sender == self.settingsTab.ikRefArray_listWidget:
                self.updateListAttr(sender, "ikrefarray")
            elif sender == self.settingsTab.upvRefArray_listWidget:
                self.updateListAttr(sender, "upvrefarray")
            elif sender == self.settingsTab.pinRefArray_listWidget:
                self.updateListAttr(sender, "pinrefarray")
            return True
        else:
            return QtWidgets.QDialog.eventFilter(self, sender, event)



    def dockCloseEventTriggered(self):
        gqt.deleteInstances(self, MayaQDockWidget)
