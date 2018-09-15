#! /usr/bin/python
from tkinter import Tk, Button, Canvas, filedialog
import Image

LED_PIXELS_SIZE=15


class MatrixGui:
	def __init__(self, master):

		self.matrix=[[(0,0,0)for y in range(16)]for x in range(32)]

		self.master = master
		master.title("Matrix GUI")

		self.loadButton = Button(master, text="Load Image", command=self.load)
		self.loadButton.pack()

		self.canvasMatrix = Canvas(master, width =LED_PIXELS_SIZE*32,
											  height=LED_PIXELS_SIZE*16)
		
		self.drawMatrix()
		self.canvasMatrix.bind("<Button-1>", self.penCallback)
		self.canvasMatrix.pack()

		self.uploadButton = Button(master, text="Upload", command=self.upload)
		self.uploadButton.pack()

	def load(self):
		image_file=filedialog.askopenfilename(initialdir = ".",title = "Select Image",
			filetypes = (("Images files","*.jpg *.png"),("all files","*.*")))
		self.import_image(image_file)
		self.drawMatrix()

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

	def penCallback(self,event):
		x=event.x/LED_PIXELS_SIZE
		y=event.y/LED_PIXELS_SIZE
		self.matrix[x][y]=(0,0,0)
		self.drawMatrix()


	def getFillColor(self,x,y):
		return "#"+"".join(["{:02x}".format(i) for i in self.matrix [x][y]])
	def drawMatrix(self):
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

