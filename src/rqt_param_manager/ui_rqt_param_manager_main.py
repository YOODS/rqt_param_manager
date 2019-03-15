# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 
# 'rqt_param_manager_main.ui'
#
# Created by: PyQt5 UI code generator 5.12
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from monitor_table import MonitorTable


class Ui_rqt_param_manager_main(object):
    def setupUi(self, rqt_param_manager_main):
        rqt_param_manager_main.setObjectName("rqt_param_manager_main")
        rqt_param_manager_main.resize(400, 300)
        self.verticalLayout = QtWidgets.QVBoxLayout(rqt_param_manager_main)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pnlBody = QtWidgets.QWidget(rqt_param_manager_main)
        self.pnlBody.setObjectName("pnlBody")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.pnlBody)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.tblConfigItems = MonitorTable(self.pnlBody)
        self.tblConfigItems.setObjectName("tblConfigItems")
        self.tblConfigItems.setColumnCount(0)
        self.tblConfigItems.setRowCount(0)
        self.verticalLayout_2.addWidget(self.tblConfigItems)
        self.verticalLayout.addWidget(self.pnlBody)
        self.pnlFooter = QtWidgets.QWidget(rqt_param_manager_main)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pnlFooter.sizePolicy().hasHeightForWidth())
        self.pnlFooter.setSizePolicy(sizePolicy)
        self.pnlFooter.setObjectName("pnlFooter")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.pnlFooter)
        self.horizontalLayout.setContentsMargins(0, 20, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.btnSave = QtWidgets.QPushButton(self.pnlFooter)
        self.btnSave.setObjectName("btnSave")
        self.horizontalLayout.addWidget(self.btnSave)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.btnClose = QtWidgets.QPushButton(self.pnlFooter)
        self.btnClose.setObjectName("btnClose")
        self.horizontalLayout.addWidget(self.btnClose)
        self.verticalLayout.addWidget(self.pnlFooter)

        self.retranslateUi(rqt_param_manager_main)
        QtCore.QMetaObject.connectSlotsByName(rqt_param_manager_main)

    def retranslateUi(self, rqt_param_manager_main):
        _translate = QtCore.QCoreApplication.translate
        rqt_param_manager_main.setWindowTitle(_translate("rqt_param_manager_main", ""))
        self.btnSave.setText(_translate("rqt_param_manager_main", "保存"))
        self.btnClose.setText(_translate("rqt_param_manager_main", "閉じる"))
