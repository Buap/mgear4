# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:/datawork/repo/mgear4/release/scripts/mgear/shifter_epic_components/EPIC_leg_02/settingsUI.ui'
#
# Created: Fri Jan 12 15:22:15 2024
#      by: pyside2-uic  running on PySide2 2.0.0~alpha0
#
# WARNING! All changes made in this file will be lost!

from PySide2 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(330, 932)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName("formLayout")
        self.ikfk_label = QtWidgets.QLabel(self.groupBox)
        self.ikfk_label.setObjectName("ikfk_label")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.ikfk_label)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.ikfk_slider = QtWidgets.QSlider(self.groupBox)
        self.ikfk_slider.setMinimumSize(QtCore.QSize(0, 15))
        self.ikfk_slider.setMaximum(100)
        self.ikfk_slider.setOrientation(QtCore.Qt.Horizontal)
        self.ikfk_slider.setObjectName("ikfk_slider")
        self.horizontalLayout_3.addWidget(self.ikfk_slider)
        self.ikfk_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.ikfk_spinBox.setMaximum(100)
        self.ikfk_spinBox.setObjectName("ikfk_spinBox")
        self.horizontalLayout_3.addWidget(self.ikfk_spinBox)
        self.formLayout.setLayout(0, QtWidgets.QFormLayout.FieldRole, self.horizontalLayout_3)
        self.maxStretch_label = QtWidgets.QLabel(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.maxStretch_label.sizePolicy().hasHeightForWidth())
        self.maxStretch_label.setSizePolicy(sizePolicy)
        self.maxStretch_label.setObjectName("maxStretch_label")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.maxStretch_label)
        self.maxStretch_spinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.maxStretch_spinBox.sizePolicy().hasHeightForWidth())
        self.maxStretch_spinBox.setSizePolicy(sizePolicy)
        self.maxStretch_spinBox.setMinimum(1.0)
        self.maxStretch_spinBox.setMaximum(1000.0)
        self.maxStretch_spinBox.setProperty("value", 100.0)
        self.maxStretch_spinBox.setObjectName("maxStretch_spinBox")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.maxStretch_spinBox)
        self.verticalLayout.addLayout(self.formLayout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.divisions_label = QtWidgets.QLabel(self.groupBox)
        self.divisions_label.setObjectName("divisions_label")
        self.horizontalLayout.addWidget(self.divisions_label)
        self.div0_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.div0_spinBox.setMinimum(0)
        self.div0_spinBox.setProperty("value", 2)
        self.div0_spinBox.setObjectName("div0_spinBox")
        self.horizontalLayout.addWidget(self.div0_spinBox)
        self.div1_spinBox = QtWidgets.QSpinBox(self.groupBox)
        self.div1_spinBox.setMinimum(0)
        self.div1_spinBox.setProperty("value", 2)
        self.div1_spinBox.setObjectName("div1_spinBox")
        self.horizontalLayout.addWidget(self.div1_spinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.leafJoints_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.leafJoints_checkBox.setObjectName("leafJoints_checkBox")
        self.verticalLayout.addWidget(self.leafJoints_checkBox)
        self.mirrorMid_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.mirrorMid_checkBox.setObjectName("mirrorMid_checkBox")
        self.verticalLayout.addWidget(self.mirrorMid_checkBox)
        self.TPoseRest_checkBox = QtWidgets.QCheckBox(self.groupBox)
        self.TPoseRest_checkBox.setObjectName("TPoseRest_checkBox")
        self.verticalLayout.addWidget(self.TPoseRest_checkBox)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.squashStretchProfile_pushButton = QtWidgets.QPushButton(self.groupBox)
        self.squashStretchProfile_pushButton.setObjectName("squashStretchProfile_pushButton")
        self.horizontalLayout_2.addWidget(self.squashStretchProfile_pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.gridLayout_2.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)
        self.ikRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.ikRefArray_groupBox.setObjectName("ikRefArray_groupBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.ikRefArray_groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.ikRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.ikRefArray_horizontalLayout.setObjectName("ikRefArray_horizontalLayout")
        self.ikRefArray_verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.ikRefArray_verticalLayout_1.setObjectName("ikRefArray_verticalLayout_1")
        self.ikRefArray_listWidget = QtWidgets.QListWidget(self.ikRefArray_groupBox)
        self.ikRefArray_listWidget.setDragDropOverwriteMode(True)
        self.ikRefArray_listWidget.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.ikRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.ikRefArray_listWidget.setAlternatingRowColors(True)
        self.ikRefArray_listWidget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.ikRefArray_listWidget.setSelectionRectVisible(False)
        self.ikRefArray_listWidget.setObjectName("ikRefArray_listWidget")
        self.ikRefArray_verticalLayout_1.addWidget(self.ikRefArray_listWidget)
        self.ikRefArray_copyRef_pushButton = QtWidgets.QPushButton(self.ikRefArray_groupBox)
        self.ikRefArray_copyRef_pushButton.setObjectName("ikRefArray_copyRef_pushButton")
        self.ikRefArray_verticalLayout_1.addWidget(self.ikRefArray_copyRef_pushButton)
        self.ikRefArray_horizontalLayout.addLayout(self.ikRefArray_verticalLayout_1)
        self.ikRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.ikRefArray_verticalLayout_2.setObjectName("ikRefArray_verticalLayout_2")
        self.ikRefArrayAdd_pushButton = QtWidgets.QPushButton(self.ikRefArray_groupBox)
        self.ikRefArrayAdd_pushButton.setObjectName("ikRefArrayAdd_pushButton")
        self.ikRefArray_verticalLayout_2.addWidget(self.ikRefArrayAdd_pushButton)
        self.ikRefArrayRemove_pushButton = QtWidgets.QPushButton(self.ikRefArray_groupBox)
        self.ikRefArrayRemove_pushButton.setObjectName("ikRefArrayRemove_pushButton")
        self.ikRefArray_verticalLayout_2.addWidget(self.ikRefArrayRemove_pushButton)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.ikRefArray_verticalLayout_2.addItem(spacerItem)
        self.ikRefArray_horizontalLayout.addLayout(self.ikRefArray_verticalLayout_2)
        self.gridLayout_3.addLayout(self.ikRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.ikRefArray_groupBox, 1, 0, 1, 1)
        self.upvRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.upvRefArray_groupBox.setObjectName("upvRefArray_groupBox")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.upvRefArray_groupBox)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.upvRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.upvRefArray_horizontalLayout.setObjectName("upvRefArray_horizontalLayout")
        self.upvRefArray_verticalLayout_1 = QtWidgets.QVBoxLayout()
        self.upvRefArray_verticalLayout_1.setObjectName("upvRefArray_verticalLayout_1")
        self.upvRefArray_listWidget = QtWidgets.QListWidget(self.upvRefArray_groupBox)
        self.upvRefArray_listWidget.setDragDropOverwriteMode(True)
        self.upvRefArray_listWidget.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.upvRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.upvRefArray_listWidget.setAlternatingRowColors(True)
        self.upvRefArray_listWidget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.upvRefArray_listWidget.setSelectionRectVisible(False)
        self.upvRefArray_listWidget.setObjectName("upvRefArray_listWidget")
        self.upvRefArray_verticalLayout_1.addWidget(self.upvRefArray_listWidget)
        self.upvRefArray_copyRef_pushButton = QtWidgets.QPushButton(self.upvRefArray_groupBox)
        self.upvRefArray_copyRef_pushButton.setObjectName("upvRefArray_copyRef_pushButton")
        self.upvRefArray_verticalLayout_1.addWidget(self.upvRefArray_copyRef_pushButton)
        self.upvRefArray_horizontalLayout.addLayout(self.upvRefArray_verticalLayout_1)
        self.upvRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.upvRefArray_verticalLayout_2.setObjectName("upvRefArray_verticalLayout_2")
        self.upvRefArrayAdd_pushButton = QtWidgets.QPushButton(self.upvRefArray_groupBox)
        self.upvRefArrayAdd_pushButton.setObjectName("upvRefArrayAdd_pushButton")
        self.upvRefArray_verticalLayout_2.addWidget(self.upvRefArrayAdd_pushButton)
        self.upvRefArrayRemove_pushButton = QtWidgets.QPushButton(self.upvRefArray_groupBox)
        self.upvRefArrayRemove_pushButton.setObjectName("upvRefArrayRemove_pushButton")
        self.upvRefArray_verticalLayout_2.addWidget(self.upvRefArrayRemove_pushButton)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.upvRefArray_verticalLayout_2.addItem(spacerItem1)
        self.upvRefArray_horizontalLayout.addLayout(self.upvRefArray_verticalLayout_2)
        self.gridLayout_5.addLayout(self.upvRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.upvRefArray_groupBox, 2, 0, 1, 1)
        self.pinRefArray_groupBox = QtWidgets.QGroupBox(Form)
        self.pinRefArray_groupBox.setObjectName("pinRefArray_groupBox")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.pinRefArray_groupBox)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.pinRefArray_horizontalLayout = QtWidgets.QHBoxLayout()
        self.pinRefArray_horizontalLayout.setObjectName("pinRefArray_horizontalLayout")
        self.pinRefArray_verticalLayout = QtWidgets.QVBoxLayout()
        self.pinRefArray_verticalLayout.setObjectName("pinRefArray_verticalLayout")
        self.pinRefArray_listWidget = QtWidgets.QListWidget(self.pinRefArray_groupBox)
        self.pinRefArray_listWidget.setDragDropOverwriteMode(True)
        self.pinRefArray_listWidget.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.pinRefArray_listWidget.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.pinRefArray_listWidget.setAlternatingRowColors(True)
        self.pinRefArray_listWidget.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.pinRefArray_listWidget.setSelectionRectVisible(False)
        self.pinRefArray_listWidget.setObjectName("pinRefArray_listWidget")
        self.pinRefArray_verticalLayout.addWidget(self.pinRefArray_listWidget)
        self.pinRefArray_copyRef_pushButton = QtWidgets.QPushButton(self.pinRefArray_groupBox)
        self.pinRefArray_copyRef_pushButton.setObjectName("pinRefArray_copyRef_pushButton")
        self.pinRefArray_verticalLayout.addWidget(self.pinRefArray_copyRef_pushButton)
        self.pinRefArray_horizontalLayout.addLayout(self.pinRefArray_verticalLayout)
        self.pinRefArray_verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.pinRefArray_verticalLayout_2.setObjectName("pinRefArray_verticalLayout_2")
        self.pinRefArrayAdd_pushButton = QtWidgets.QPushButton(self.pinRefArray_groupBox)
        self.pinRefArrayAdd_pushButton.setObjectName("pinRefArrayAdd_pushButton")
        self.pinRefArray_verticalLayout_2.addWidget(self.pinRefArrayAdd_pushButton)
        self.pinRefArrayRemove_pushButton = QtWidgets.QPushButton(self.pinRefArray_groupBox)
        self.pinRefArrayRemove_pushButton.setObjectName("pinRefArrayRemove_pushButton")
        self.pinRefArray_verticalLayout_2.addWidget(self.pinRefArrayRemove_pushButton)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.pinRefArray_verticalLayout_2.addItem(spacerItem2)
        self.pinRefArray_horizontalLayout.addLayout(self.pinRefArray_verticalLayout_2)
        self.gridLayout_4.addLayout(self.pinRefArray_horizontalLayout, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.pinRefArray_groupBox, 3, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QObject.connect(self.ikfk_slider, QtCore.SIGNAL("sliderMoved(int)"), self.ikfk_spinBox.setValue)
        QtCore.QObject.connect(self.ikfk_spinBox, QtCore.SIGNAL("valueChanged(int)"), self.ikfk_slider.setValue)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtWidgets.QApplication.translate("Form", "Form", None, -1))
        self.ikfk_label.setText(QtWidgets.QApplication.translate("Form", "IK/FK Blend", None, -1))
        self.maxStretch_label.setText(QtWidgets.QApplication.translate("Form", "Max Stretch", None, -1))
        self.divisions_label.setText(QtWidgets.QApplication.translate("Form", "Divisions", None, -1))
        self.leafJoints_checkBox.setText(QtWidgets.QApplication.translate("Form", "Leaf Joints", None, -1))
        self.mirrorMid_checkBox.setToolTip(QtWidgets.QApplication.translate("Form", "This option set the axis of the mid CTL (elbow) and the up vector control to move in a mirror behaviour ", None, -1))
        self.mirrorMid_checkBox.setStatusTip(QtWidgets.QApplication.translate("Form", "This option set the axis of the mid CTL (elbow) and the up vector control to move in a mirror behaviour ", None, -1))
        self.mirrorMid_checkBox.setWhatsThis(QtWidgets.QApplication.translate("Form", "This option set the axis of the mid CTL (elbow) and the up vector control to move in a mirror behaviour ", None, -1))
        self.mirrorMid_checkBox.setText(QtWidgets.QApplication.translate("Form", "Mirror Mid Ctl and UPV  axis", None, -1))
        self.TPoseRest_checkBox.setToolTip(QtWidgets.QApplication.translate("Form", "<html><head/><body><p>If checked, the Rest pose for controls will be in T Pose</p></body></html>", None, -1))
        self.TPoseRest_checkBox.setStatusTip(QtWidgets.QApplication.translate("Form", "This option set the axis of the mid CTL (elbow) and the up vector control to move in a mirror behaviour ", None, -1))
        self.TPoseRest_checkBox.setWhatsThis(QtWidgets.QApplication.translate("Form", "This option set the axis of the mid CTL (elbow) and the up vector control to move in a mirror behaviour ", None, -1))
        self.TPoseRest_checkBox.setText(QtWidgets.QApplication.translate("Form", "Rest T Pose", None, -1))
        self.squashStretchProfile_pushButton.setText(QtWidgets.QApplication.translate("Form", "Squash and Stretch Profile", None, -1))
        self.ikRefArray_groupBox.setTitle(QtWidgets.QApplication.translate("Form", "IK Reference Array", None, -1))
        self.ikRefArray_copyRef_pushButton.setText(QtWidgets.QApplication.translate("Form", "Copy from UpV Ref", None, -1))
        self.ikRefArrayAdd_pushButton.setText(QtWidgets.QApplication.translate("Form", "<<", None, -1))
        self.ikRefArrayRemove_pushButton.setText(QtWidgets.QApplication.translate("Form", ">>", None, -1))
        self.upvRefArray_groupBox.setTitle(QtWidgets.QApplication.translate("Form", "UpV Reference Array", None, -1))
        self.upvRefArray_copyRef_pushButton.setText(QtWidgets.QApplication.translate("Form", "Copy from IK Ref", None, -1))
        self.upvRefArrayAdd_pushButton.setText(QtWidgets.QApplication.translate("Form", "<<", None, -1))
        self.upvRefArrayRemove_pushButton.setText(QtWidgets.QApplication.translate("Form", ">>", None, -1))
        self.pinRefArray_groupBox.setTitle(QtWidgets.QApplication.translate("Form", "Pin Elbow Reference Array", None, -1))
        self.pinRefArray_copyRef_pushButton.setText(QtWidgets.QApplication.translate("Form", "Copy from IK Ref", None, -1))
        self.pinRefArrayAdd_pushButton.setText(QtWidgets.QApplication.translate("Form", "<<", None, -1))
        self.pinRefArrayRemove_pushButton.setText(QtWidgets.QApplication.translate("Form", ">>", None, -1))

