__author__ = 'fabien'
import interface
import forger
from PyQt4 import QtGui
import sys

main_app = QtGui.QApplication(sys.argv)
fenetre = QtGui.QMainWindow()
appli = forger.Forger(fenetre)
appli.setupUi(fenetre)
appli.built()
fenetre.show()
main_app.exec_()
sys.exit(0)