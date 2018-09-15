#! /usr/bin/python
from tkinter import *
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter.colorchooser import askcolor
import Image
import numpy as np

LED_PIXELS_SIZE=15
CURRENT_COLOR_TEXT="Crayon:({:03},{:03},{:03})"
CURRENT_PIXEL_TEXT="Pixel:({:03},{:03},{:03})"

class MatrixGui:
	def __init__(self, master):

		self.matrix=[[(0,0,0) for y in range(16)]for x in range(32)]
		self.currentColor=(0,0,0)

		self.master = master
		master.title("Matrix GUI")

		#Load & Save image
		self.loadSaveFrame=Frame(master)
		self.loadSaveFrame.pack()

		self.loadButton = Button(self.loadSaveFrame, text="Load", command=self.load)
		self.loadButton.pack(side=LEFT)

		self.saveButton = Button(self.loadSaveFrame, text="Save", command=self.save)
		self.saveButton.pack(side=LEFT)
		#Matrix Canvas (Image plot & modification)
		self.canvasMatrix = Canvas(master, width =LED_PIXELS_SIZE*32,
											  height=LED_PIXELS_SIZE*16)
		
		self.drawMatrix()
		self.canvasMatrix.bind("<B1-Motion>", self.penCallback)
		self.canvasMatrix.bind("<Motion>", self.moveMouseOverPixel)
		self.canvasMatrix.pack()

		#Pen Information
		self.PenOptionFrame=Frame(master)
		self.PenOptionFrame.pack()

		self.colorButton=Button(self.PenOptionFrame,text='Select Color', command=self.setColor)
		self.colorButton.pack(side=LEFT,padx=50)

		self.currentPenColorVar=StringVar()
		self.currentPenColorVar.set(CURRENT_COLOR_TEXT.format(0,0,0) )
		self.currentPenColorLabel=Label(self.PenOptionFrame, textvariable=self.currentPenColorVar)
		self.currentPenColorLabel.pack(side=LEFT,padx=50)

		self.currentPixelColorVar=StringVar()
		self.currentPixelColorVar.set(CURRENT_PIXEL_TEXT.format(0,0,0) )
		self.currentPixelColorLabel=Label(self.PenOptionFrame, textvariable=self.currentPixelColorVar)
		self.currentPixelColorLabel.pack(side=LEFT,padx=50)

		#Upload option
		self.uploadButton = Button(master, text="Upload", command=self.upload)
		self.uploadButton.pack()

	#Upload, load, save functions
	def load(self):
		image_file=askopenfilename(initialdir = ".",title = "Select Image",
			filetypes = (("Images files","*.jpg *.png"),("all files","*.*")))
		self.import_image(image_file)
		self.drawMatrix()

	def save(self):
		image_file=asksaveasfilename(initialdir = ".",title = "Select Image",
			filetypes = (("Images files","*.jpg *.png"),("all files","*.*")))

		np_matrix=np.array(self.matrix,dtype=np.uint8).transpose((1,0,2))#inverse x and y

		new_image=Image.fromarray(np_matrix,mode='RGB')
		new_image.save(image_file, 'PNG',dpi=[32,16])

	def upload(self):
		print("Upload!")

	def import_image(self,fichier):
		if type(fichier)==tuple or fichier=="":
			return
		try:
			im = Image.open(fichier)
		except Exception as e:
			print("Erreur : ",e)
			return
		image_data=im.resize( (32,16) ,Image.BICUBIC).getdata()
		self.matrix=[[(0,0,0)for y in range(16)]for x in range(32)]
		for i in range(len(image_data)):
			self.matrix[i%32][i/32]=image_data[i][:3]

	#Pen functions
	def setColor(self):
		self.currentColor= askcolor()[0]
		self.currentPenColorVar.set(CURRENT_COLOR_TEXT.format(
			self.currentColor[0],
			self.currentColor[1],
			self.currentColor[2]))

	def getXYmatrixFromMouse(self,event):
		if event.x<0 or event.y<0:
			return -1,-1
		x=event.x/LED_PIXELS_SIZE
		y=event.y/LED_PIXELS_SIZE
		if x>=32 or y>=16:
			return -1,-1
		return x,y

	def penCallback(self,event):
		x,y=self.getXYmatrixFromMouse(event)
		if x<0 or y<0:return
		self.matrix[x][y]=self.currentColor
		self.drawMatrix()

	def moveMouseOverPixel(self,event):
		x,y=self.getXYmatrixFromMouse(event)
		if x<0 or y<0:return
		self.currentPixelColorVar.set(CURRENT_PIXEL_TEXT.format(
			self.matrix[x][y][0],
			self.matrix[x][y][1],
			self.matrix[x][y][2]))

	#draw functions
	def getFillColor(self,x,y):
		return "#"+"".join(["{:02x}".format(i) for i in self.matrix [x][y]])

	def drawMatrix(self):
		self.canvasMatrix.delete("all")
		for x in range(32):
			for y in range(16):
				color=self.getFillColor(x,y)
				self.canvasMatrix.create_rectangle(
					LED_PIXELS_SIZE*x,		#x min
					LED_PIXELS_SIZE*y,		#y min
					LED_PIXELS_SIZE*(x+1),	#x max
					LED_PIXELS_SIZE*(y+1),	#y max
					fill=color,outline="")

if __name__=="__main__":
	root = Tk()
	my_gui = MatrixGui(root)
	root.mainloop()

