# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'interface.ui'
#
# Created: Fri Jan 30 22:15:46 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(800, 600)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.horizontalLayout.addWidget(self.label)
        self.lineEditAuthor = QtGui.QLineEdit(self.centralwidget)
        self.lineEditAuthor.setObjectName(_fromUtf8("lineEditAuthor"))
        self.horizontalLayout.addWidget(self.lineEditAuthor)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.lineEditStatename = QtGui.QLineEdit(self.centralwidget)
        self.lineEditStatename.setObjectName(_fromUtf8("lineEditStatename"))
        self.horizontalLayout_2.addWidget(self.lineEditStatename)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.horizontalLayout_4.addWidget(self.label_3)
        self.includesPlainText = QtGui.QPlainTextEdit(self.centralwidget)
        self.includesPlainText.setObjectName(_fromUtf8("includesPlainText"))
        self.horizontalLayout_4.addWidget(self.includesPlainText)
        self.verticalLayout_2.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_5.addWidget(self.label_5)
        self.flagsPlainText = QtGui.QPlainTextEdit(self.centralwidget)
        self.flagsPlainText.setObjectName(_fromUtf8("flagsPlainText"))
        self.horizontalLayout_5.addWidget(self.flagsPlainText)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.setModelsDirectoryButton = QtGui.QPushButton(self.centralwidget)
        self.setModelsDirectoryButton.setObjectName(_fromUtf8("setModelsDirectoryButton"))
        self.verticalLayout.addWidget(self.setModelsDirectoryButton)
        self.setDirectoryButton = QtGui.QPushButton(self.centralwidget)
        self.setDirectoryButton.setObjectName(_fromUtf8("setDirectoryButton"))
        self.verticalLayout.addWidget(self.setDirectoryButton)
        self.verticalLayout_3.addLayout(self.verticalLayout)
        self.createFilesButton = QtGui.QPushButton(self.centralwidget)
        self.createFilesButton.setObjectName(_fromUtf8("createFilesButton"))
        self.verticalLayout_3.addWidget(self.createFilesButton)
        self.horizontalLayout_3.addLayout(self.verticalLayout_3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 30))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFichier = QtGui.QMenu(self.menubar)
        self.menuFichier.setObjectName(_fromUtf8("menuFichier"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.actionQuitter = QtGui.QAction(MainWindow)
        self.actionQuitter.setObjectName(_fromUtf8("actionQuitter"))
        self.menuFichier.addAction(self.actionQuitter)
        self.menubar.addAction(self.menuFichier.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Créateur de state", None))
        self.label.setText(_translate("MainWindow", "Auteur : ", None))
        self.label_2.setText(_translate("MainWindow", "nom de l\'état : ", None))
        self.label_3.setToolTip(_translate("MainWindow", "<html><head/><body><p>chemins relatifs vers les fichiers à inclures, un fichier par ligne.</p><p>ex:</p><p>../params.h</p><p>lib_motor.h</p></body></html>", None))
        self.label_3.setText(_translate("MainWindow", "includes : ", None))
        self.label_5.setToolTip(_translate("MainWindow", "<html><head/><body><p>Un état par ligne. ex:</p><p>E_MOTOR</p><p>E_RADAR</p></body></html>", None))
        self.label_5.setText(_translate("MainWindow", "Flags : ", None))
        self.setModelsDirectoryButton.setText(_translate("MainWindow", "Dossier de modèles", None))
        self.setDirectoryButton.setText(_translate("MainWindow", "Dossier de destination", None))
        self.createFilesButton.setText(_translate("MainWindow", "Créer les fichiers", None))
        self.menuFichier.setTitle(_translate("MainWindow", "Fichier", None))
        self.actionQuitter.setText(_translate("MainWindow", "Quitter", None))

