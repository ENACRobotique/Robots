#! /usr/bin/python
# -*- coding: utf-8 -*-
import Image
from sys import argv

def import_image(fichier):
	try:
		im = Image.open(fichier)
	except e:
		print("Erreur : ",e)
		return 0;
	return im.resize( (32,16) ,Image.BICUBIC)

def write_sprit(new_fichier,data):
	with open(new_fichier, 'w') as fichier:
		fichier.write("//Fonction généré par kaleidoPy\n")
		fichier.write("void draw_{}(int offset){{\n".format(".".join(new_fichier.split(".")[:-1]) ) )
		n=0
		for i in data:
			r,g,b=i
			r>>=4+1#reduce the intensity
			g>>=4+1
			b>>=4+1
			if (r,g,b)!=(0,0,0):
				fichier.write("\tdrawPixel2({},{},{},{},{},offset);\n".
										format(n%32,n/32,r,g,b) )
			n+=1
		fichier.write("}\n")

def create_sprit(fichier,plot=False):
	im=import_image(fichier)
	if im==0:return
	if plot:im.resize( (32*20,16*20) ).show()
	
	new_fichier=".".join(fichier.split(".")[:-1])+".c"
	write_sprit(new_fichier,im.getdata())

PLOT=True
if __name__=="__main__":
	if len(argv)<=1:
		print("Il faut donner des images en arguments!")
		exit(0)
	for i in argv[1:]:
		create_sprit(i,PLOT)
