# Make sure this package is installed:
# sudo apt-get install python-imaging-tk

from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

from Tkinter import *
from PIL import Image, ImageTk # Python Imaging Library
import cv2, numpy as np

class cameraGUI(object):
	slider1Value = 0
	slider2Value = 0
	slider3Value = 0
	first_view = True

	def __init__(self, master):
		self.master = master
		master.title('Blob Camera')

		self.updateCamView()

		# Create the buttons
		self.res_btn = Button(master, text="Reset Camera",
							  command=self.reset_cam)
		self.res_btn.pack(side="bottom", fill="both", expand="yes", padx="10",
					 pady="10")
		self.cal_btn = Button(master, text="Calibration",
							  command=self.calibrate_cam)
		self.cal_btn.pack(side="bottom", fill="both", expand="yes", padx="10",
						  pady="10")
		self.leds_btn = Button(master, text="Illuminate eBugs",
							   command=self.illuminate_ebugs)
		self.leds_btn.pack(side="bottom", fill="both", expand="yes", padx="10",
						   pady="10")

		# Create the sliders
		self.slider_1 = Scale(master, from_=0, to=256, orient="horizontal")
		self.slider_1.bind("<ButtonRelease-1>", self.updateSlider1Value)
		self.slider_1.pack(side="bottom", fill="both", expand="yes", padx="10",
					  pady="10")

		self.slider_2 = Scale(master, from_=0, to=256, orient="horizontal")
		self.slider_2.bind("<ButtonRelease-1>", self.updateSlider2Value)
		self.slider_2.pack(side="bottom", fill="both", expand="yes", padx="10",
					  pady="10")

		self.slider_3 = Scale(master, from_=0, to=256, orient="horizontal")
		self.slider_3.bind("<ButtonRelease-1>", self.updateSlider3Value)
		self.slider_3.pack(side="bottom", fill="both", expand="yes", padx="10",
					  pady="10")

	def reset_cam(self):
		print('You pressed the reset_cam button.')

	def calibrate_cam(self):
		print('You pressed the calibrate_cam button.')

	def illuminate_ebugs(self):
		print('You pressed the illuminate_ebugs button.')

	def updateSlider1Value(self, event):
		self.slider1Value = self.slider_1.get()
		self.updateCamView()

	def updateSlider2Value(self, event):
		self.slider2Value = self.slider_2.get()
		self.updateCamView()

	def updateSlider3Value(self, event):
		self.slider3Value = self.slider_3.get()
		self.updateCamView()

	def updateCamView(self):
		self.img = np.zeros((512, 640, 3), np.uint8) # Create the blank view
		cv2.putText(self.img, '%0d' % self.slider1Value, (0, 50),
					cv2.FONT_HERSHEY_PLAIN, 4, (0, 0, 255), 1, cv2.LINE_AA)
		cv2.putText(self.img, '%0d' % self.slider2Value, (150, 50),
					cv2.FONT_HERSHEY_PLAIN, 4, (0, 0, 255), 1, cv2.LINE_AA)
		cv2.putText(self.img, '%0d' % self.slider3Value, (300, 50),
					cv2.FONT_HERSHEY_PLAIN, 4, (0, 0, 255), 1, cv2.LINE_AA)
		self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
		self.img = Image.fromarray(self.img)
		self.img = ImageTk.PhotoImage(self.img)

		if self.first_view:
			self.cam_view = Label(image=self.img)
			self.cam_view.image = self.img
			self.cam_view.pack(side="left", padx=10, pady=10)
			self.first_view = False
		else:
			self.cam_view.configure(image=self.img)
			self.cam_view.image = self.img
#end class cameraGUI

if __name__ == '__main__':
	root = Tk()
	cam_view = cameraGUI(root)
	root.mainloop()
