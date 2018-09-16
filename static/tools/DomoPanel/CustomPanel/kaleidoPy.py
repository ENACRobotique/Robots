#! /usr/bin/python
# -*- coding: utf-8 -*-
import IHM.MatrixGui as IHM
from tkinter import Tk
from sys import argv
import os

RESULT_PATH="src/image.h"

class kaleidoGui(IHM.MatrixGui):

	def __init__(self,master):
		IHM.MatrixGui.__init__(self,master)
		master.title("Kaleido GUI")

	def write_matrix(self,new_fichier):
		with open(new_fichier, 'w') as fichier:
			fichier.write("//Fonction généré par kaleidoPy\n")
			fichier.write("void draw_image(int offset){{\n".format(".".join(new_fichier.split(".")[:-1]) ) )

			for x in range(len(self.matrix)):
				for y in range(len(self.matrix[0])):
					r,g,b=self.matrix[x][y][:3]
					r>>=4+1#reduce the intensity
					g>>=4+1
					b>>=4+1
					if (r,g,b)!=(0,0,0):
						fichier.write("\tdrawPixel2({},{},{},{},{},offset);\n".
												format(x,y,r,g,b) )

			fichier.write("}\n")
			if(self.AnimationVar.get()):
				fichier.write("#define ANIMATION\n")


	def upload(self):
		self.write_matrix(RESULT_PATH)
		os.system("make upload")

if __name__=="__main__":
	root = Tk()
	root.withdraw()
	my_gui = kaleidoGui(root)
	if len(argv)>1:
		my_gui.import_image(argv[1])
		my_gui.upload()
		exit(0)
	
	root.update()
	root.deiconify()
	root.mainloop()
		

