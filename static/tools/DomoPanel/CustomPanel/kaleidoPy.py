#! /usr/bin/python
# -*- coding: utf-8 -*-
import Image
from sys import argv
import os

def import_image(fichier):
	try:
		im = Image.open(fichier)
	except e:
		print("Erreur : ",e)
		return 0;
	return im.resize( (32,16) ,Image.BICUBIC)

def write_matrix(new_fichier,data):
	with open(new_fichier, 'w') as fichier:
		fichier.write("//Fonction généré par kaleidoPy\n")
		fichier.write("void draw_image(int offset){{\n".format(".".join(new_fichier.split(".")[:-1]) ) )
		n=0
		for i in data:
			r,g,b=i[:3]
			r>>=4+1#reduce the intensity
			g>>=4+1
			b>>=4+1
			if (r,g,b)!=(0,0,0):
				fichier.write("\tdrawPixel2({},{},{},{},{},offset);\n".
										format(n%32,n/32,r,g,b) )
			n+=1
		fichier.write("}\n")

def create_matrix(fichier,plot=False):
	im=import_image(fichier)
	if im==0:return
	if plot:im.resize( (32*20,16*20) ).show()
	
	#new_fichier="src/"+".".join(fichier.split(".")[:-1])+".h"
	new_fichier="src/image.h"
	write_matrix(new_fichier,im.getdata())

def upload_matrix():
	os.system("make upload")

PLOT=True
if __name__=="__main__":
	if len(argv)<=1:
		print("Il faut donner une image en argument!")
		exit(0)
	for i in argv[1:]:
		create_matrix(i,PLOT)
		response=raw_input("Voulez-vous flashez l'image?(y/N) ")
		if('Y' in response.upper()):
			upload_matrix()

