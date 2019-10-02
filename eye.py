#!/usr/bin/env python
import pygame
import pygame.camera
import pygame.image
from pygame.locals import *
from PIL import Image
from PIL import ImageChops
from PIL import ImageEnhance
from PIL import ImageMath
from PIL import ImageFilter
from PIL.ImageFilter import CONTOUR
import numpy as np
import scipy 
from scipy import misc
from scipy import ndimage
from scipy.misc import toimage
import time
import matplotlib.pyplot as plt
from glob import glob;
from time import time 
import pandas as pd
import os, sys

class Eye: # class of image processing

	def __init__(self, camera=1):
		self.camera = camera # white cable is camera 1
		self.size = (800,600)
		pygame.init()
		pygame.camera.init()
		camlist=pygame.camera.list_cameras()
		self.cam=pygame.camera.Camera(camlist[0],self.size)
		# create a display surface - standard pygame stuff
		self.display = pygame.display.set_mode(self.size, 0)
		self.imgSurface = pygame.surface.Surface(self.size, 0, self.display)
		self.cam.start()
		self.source=None
		# rearrange the field again by considering real x-y coordinate system
		if self.camera == 1:
			self.y=442.0
		elif self.camera == 2:
			self.y=450.0
		print "Now, camera is working.."

	def get_things(self):
		# Define H S V
		H,S,V=0,1,2
		# Capture Image using pre-Surface to make it faster
		captured=self.cam.get_image(self.imgSurface)
		# Convert to string
		array = pygame.image.tostring(captured,"RGB")
		# Convert to Numpy Array
		pil_image=Image.frombytes("RGB",(800,600),array)
		# Camera Dependent Crop
		if self.camera == 1:
			cap_cropped=pil_image.rotate(-0.5).crop((110,112,671,554))
		elif self.camera == 2:
			cap_cropped=pil_image.rotate(0.7).crop((81,78,648,528))
		# Convert Image to HSV
		cap_cropped.convert("HSV")
		# Split
		self.source = cap_cropped.split()

	def get_ball_centr(self):
		self.get_things() # get splitted images
		# get center of mass
		self.ball_centr=self.colorCenter("orange")
		return self.ball_centr

	def get_robot1_centr(self):
		self.get_things() # get splitted images
		self.blue_centr=self.colorCenter("blue")
		self.get_things()
		# get center of mass
		self.yellow_centr=self.colorCenter("yellow")
		return [self.blue_centr,self.yellow_centr]

	def get_robot2_centr(self):
		self.get_things()
		self.colorCenter("red")
		self.get_things()
		self.colorCenter("green")

	def get_ball_and_robot1_centr(self):
		return 0

	def get_all_centr(self):
		return 0


	def colorCenter(self,color):
		# [[H_low,H_up],[S_low,S_up],[V_low,V_up]]
		filterVal=[[0,0],[0,0],[0,0]]
		# define filter boundaries
		# define which images will be inverted
		if color == "orange":
			filterVal=[[50,180],[20,110],[3,150]]
			filterEros=[[15,15],[10,10]]
			inv_0=False
			inv_1=False
			inv_2=False
		elif color == "blue":
			filterVal=[[10,110],[55,125],[30,80]]
			filterEros=[[5,5],[12,12]]
			inv_0=False
			inv_1=False
			inv_2=True
		elif color == "green":
			filterVal=[[0,0],[0,0],[0,0]]
			filterEros=[[3,3],[3,3]]
			inv_0=False
			inv_1=False
			inv_2=False
		elif color == "yellow":
			filterVal=[[100,250],[10,130],[70,150]]
			filterEros=[[15,15],[8,8]]
			inv_0=False
			inv_1=True
			inv_2=True
		elif color == "red":
			filterVal=[[0,0],[0,0],[0,0]]
			filterEros=[[3,3],[3,3]]
			inv_0=False
			inv_1=False
			inv_2=False


		source = list(map(lambda layer,limits:layer.point(lambda i:limits[0]<i<limits[1] and 225),self.source,filterVal))
		if inv_0==True: source[0]=ImageChops.invert(source[0])
		if inv_1==True: source[1]=ImageChops.invert(source[1])
		if inv_2==True: source[2]=ImageChops.invert(source[2])

		color_img = ImageMath.eval("a & b & c", a=source[0]\
											  , b=source[1]\
											  , c=source[2])
		color_img = ndimage.binary_closing(color_img,\
						structure=np.ones(filterEros[0])).astype(np.int)
		color_img = ndimage.binary_erosion(color_img,\
						structure=np.ones(filterEros[1])).astype(np.int)


		# Centroid Calculation
		position = ndimage.measurements.center_of_mass(color_img)
		
		#color_img=toimage(color_img)
		#color_img.show()
		
		# centroid function gives [nan,nan] in the case of no any object detection
		# we will change them with [0,0]
		array=list(position)
		if str(array[0])=='nan':
			corrected_array=[0,0]
		else: # rearrange the field axes
			a=self.y-array[0]
			b=array[1]
			corrected_array=[b,a]
		return corrected_array