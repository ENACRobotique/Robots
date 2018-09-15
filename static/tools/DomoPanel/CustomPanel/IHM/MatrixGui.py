#! /usr/bin/python
from tkinter import *
from tkinter.colorchooser import askcolor
import Image

LED_PIXELS_SIZE=15


class MatrixGui:
	def __init__(self, master):

		self.matrix=[[(0,0,0)for y in range(16)]for x in range(32)]
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
		self.canvasMatrix.pack()

		#Pen Information
		self.PenOptionFrame=Frame(master)
		self.PenOptionFrame.pack()

		self.colorButton=Button(self.PenOptionFrame,text='Select Color', command=self.getColor)
		self.colorButton.pack(side=LEFT,padx=50)

		self.currentPenColorLabel=Label(self.PenOptionFrame, text="Couleur crayon!")
		self.currentPenColorLabel.pack(side=LEFT,padx=50)

		self.currentPixelColorLabel=Label(self.PenOptionFrame, text="Couleur actuelle!")
		self.currentPixelColorLabel.pack(side=LEFT,padx=50)

		#Upload option
		self.uploadButton = Button(master, text="Upload", command=self.upload)
		self.uploadButton.pack()

	def load(self):
		image_file=filedialog.askopenfilename(initialdir = ".",title = "Select Image",
			filetypes = (("Images files","*.jpg *.png"),("all files","*.*")))
		self.import_image(image_file)
		self.drawMatrix()

	def save(self):
		print("Save image")

	def upload(self):
		print("Upload!")

	def import_image(self,fichier):
		try:
			im = Image.open(fichier)
		except Exception as e:
			print("Erreur : ",e)
			return 0
		image_data=im.resize( (32,16) ,Image.BICUBIC).getdata()
		self.matrix=[[(0,0,0)for y in range(16)]for x in range(32)]
		for i in range(len(image_data)):
			self.matrix[i%32][i/32]=image_data[i][:3]

	def getColor(self):
		self.currentColor= askcolor()[0]
	def penCallback(self,event):
		if event.x<0 or event.y<0:
			return
		x=event.x/LED_PIXELS_SIZE
		y=event.y/LED_PIXELS_SIZE
		if x>=32 or y>=16:
			return
		self.matrix[x][y]=self.currentColor
		self.drawMatrix()


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

