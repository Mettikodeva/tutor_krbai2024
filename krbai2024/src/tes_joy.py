#!/usr/bin/python

import Tkinter
from Tkinter import *
import tkMessageBox

top = Tkinter.Tk()

class Joystick:
	speed = 0
	direction = 0
	B_show = Tkinter.Button(top, text = "Show")
	def __init__(self,parent):
		B_up = Tkinter.Button(top, text = u'\u25b2')
		B_up.grid(row=1,column=2)
		B_up.bind("<Button-1>",self.upMessage)
		B_left = Tkinter.Button(top, text = u'\u25c0')
		B_left.grid(row=2,column=1)
		B_left.bind("<Button-1>",self.leftMessage)
		B_down = Tkinter.Button(top, text = u'\u25bc')
		B_down.grid(row=2,column=2)
		B_down.bind("<Button-1>",self.downMessage)
		B_right = Tkinter.Button(top, text = u'\u25b6')
		B_right.grid(row=2,column=3)
		B_right.bind("<Button-1>",self.rightMessage)
		
		self.B_show.grid(row=4,column=2,rowspan=3)
		self.B_show.bind("<Button-1>",self.showMessagePress)
		self.B_show.bind("<ButtonRelease-1>", self.showMessageRelease)
		
		# Button to pop up
	def upMessage(self,event):
		if self.speed < 1:
			self.speed += 0.1
   
	def leftMessage(self,event):
		if self.direction > -1:
			self.direction -= 0.1
   
	def downMessage(self,event):
		if self.speed > -1:
			self.speed -= 0.1
	   
	def rightMessage(self,event):
		if self.direction < 1:
			self.direction += 0.1
	   
	def showMessagePress(self,event):
		self.B_show.bind("<Motion>",self.showMotion)
		#print( "Speed:"+str(self.speed)+" Direction:"+str(self.direction))
		#tkMessageBox.showinfo( "Hello Python", "Speed:"+str(self.speed)+" Direction:"+str(self.direction))

	def showMessageRelease(self,event):
		self.B_show.unbind("<Motion>")

	def showMotion(self,event):
		print( "Mouse position: (%s %s)" % (event.x, event.y))
		print( "Speed:"+str(self.speed)+" Direction:"+str(self.direction))

myJoystick = Joystick(top)


#B_up.pack()
#B.place(bordermode=OUTSIDE, relheight=0.2, relwidth=0.5)

#B.grid(row=2,column=2)
top.mainloop()