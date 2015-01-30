from PyQt4 import QtGui
from interface import Ui_MainWindow
import os
import time

class Forger(Ui_MainWindow):
    def __init__(self,MainWindow):
        super(Forger,self).__init__()
        self.MainWindow = MainWindow
        self.author = ''
        self.statename = ''
        self.sources=['models/state_model.cpp','models/state_model.h']
        self.destination = './'
        self.table = str.maketrans(' éèêà@çöôïî<>^','_eeeaacooii___')

    def built(self):
        self.createFilesButton.clicked.connect(self.create_files)
        self.setDirectoryButton.clicked.connect(self.select_model_directory)

    def set_author(self):
        author = self.lineEditAuthor.text().lower()
        author = author.translate(self.table)
        author = author.strip('_')
        self.author = author.capitalize()

    def set_state_name(self):
        statename = self.lineEditStatename.text().lower()
        statename = statename.translate(self.table)
        statename = statename.strip('_')
        self.statename = statename.capitalize()

    def create_files(self):
        self.set_author()
        self.set_state_name()
        date = time.strftime("%Y %B %d")
        for file in os.listdir('models/'):
            source = 'models/' + file
        #for source in self.sources:
            fs = open(source,'r')
            _,ext = os.path.splitext(source)
            destination = self.destination + 'state_' + self.statename + ext
            fd = open(destination, 'w')

            for line in fs:
                line = line.replace('%STATENAME%', self.statename)
                line = line.replace('%STATENAME_UPPER%', self.statename.upper())
                line = line.replace('%AUTHOR%', self.author)
                line = line.replace('%DATE%', date)

                fd.write(line)

        fs.close()
        fd.close()

    def select_model_directory(self):
        #TODO cet fonction, la sélection, ...
        directoryselecter = QtGui.QFileDialog(None, 'Open file', '/home/fabien')
        #directoryselecter.setFileMode(QtGui.QFileDialog.Directory)
        #directoryselecter.setNameFilter("All C++ files (*.cpp *.cc *.C *.cxx *.c++)")
        directory = directoryselecter.getOpenFileName()
        print(directory)
