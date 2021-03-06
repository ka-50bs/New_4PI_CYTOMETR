# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'settingsWindow.ui'
#
# Created by: PyQt5 UI code generator 5.15.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_SettingsWindow(object):
    def setupUi(self, SettingsWindow):
        SettingsWindow.setObjectName("SettingsWindow")
        SettingsWindow.resize(400, 393)
        self.gridLayoutWidget = QtWidgets.QWidget(SettingsWindow)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 371))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.portMonitorPushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.portMonitorPushButton.setObjectName("portMonitorPushButton")
        self.gridLayout.addWidget(self.portMonitorPushButton, 4, 3, 1, 1)
        self.setParamsPushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.setParamsPushButton.setObjectName("setParamsPushButton")
        self.gridLayout.addWidget(self.setParamsPushButton, 3, 3, 1, 1)
        self.showStatusLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.showStatusLabel.setText("")
        self.showStatusLabel.setObjectName("showStatusLabel")
        self.gridLayout.addWidget(self.showStatusLabel, 3, 2, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 1, 3, 1, 1)
        self.gainLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.gainLabel.setObjectName("gainLabel")
        self.gridLayout.addWidget(self.gainLabel, 0, 0, 1, 1)
        self.triggerLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.triggerLabel.setObjectName("triggerLabel")
        self.gridLayout.addWidget(self.triggerLabel, 1, 0, 1, 1)
        self.gainLineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget)
        self.gainLineEdit.setObjectName("GainLineEdit")
        self.gridLayout.addWidget(self.gainLineEdit, 0, 2, 1, 1)
        self.triggerLowLineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget)
        self.triggerLowLineEdit.setObjectName("triggerLowLineEdit")
        self.gridLayout.addWidget(self.triggerLowLineEdit, 1, 2, 1, 1)
        self.portNameLineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget)
        self.portNameLineEdit.setObjectName("portNameLineEdit")
        self.gridLayout.addWidget(self.portNameLineEdit, 2, 2, 1, 1)
        self.portNameLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.portNameLabel.setObjectName("portNameLabel")
        self.gridLayout.addWidget(self.portNameLabel, 2, 0, 1, 1)
        self.connectPushButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.connectPushButton.setObjectName("connectPushButton")
        self.gridLayout.addWidget(self.connectPushButton, 2, 3, 1, 1)

        self.retranslateUi(SettingsWindow)
        QtCore.QMetaObject.connectSlotsByName(SettingsWindow)

    def retranslateUi(self, SettingsWindow):
        _translate = QtCore.QCoreApplication.translate
        SettingsWindow.setWindowTitle(_translate("SettingsWindow", "Settings"))
        self.portMonitorPushButton.setText(_translate("SettingsWindow", "open port monitor"))
        self.setParamsPushButton.setText(_translate("SettingsWindow", "Set parametrs"))
        self.label_3.setText(_translate("SettingsWindow", "0 - 4095"))
        self.gainLabel.setText(_translate("SettingsWindow", "Gain"))
        self.triggerLabel.setText(_translate("SettingsWindow", "Trigger level"))
        self.portNameLabel.setText(_translate("SettingsWindow", "port name"))
        self.connectPushButton.setText(_translate("SettingsWindow", "Connect"))
