#!/usr/bin/env python

import numpy as np
import math

N_ARUCOS=28

class ArucoInfo():
	def __init__(self, arucoId, arucox, arucoy, arucoAlfa, arucox_world, arucoy_world, arucoAlfa_world):
		self.id=arucoId
		self.x=arucox
		self.y=arucoy
		self.alfa=arucoAlfa
		self.x_world=arucox_world
		self.y_world=arucoy_world
		self.alfa_world=arucoAlfa_world

	def get_id(self):
		return self.id
	def get_x(self):
		return self.x
	def get_y(self):
		return self.y
	def get_alfa(self):
		return self.alfa
	def get_measurement(self):
		return self.x, self.y, self.alfa
	def get_pose_world(self):
		return self.x_world, self.y_world, self.alfa_world
	def __repr__(self):
		return "marker id:%d x:%.4f y:%.4f alfa:%.4f"%(self.get_id(),self.get_x(),self.get_y(),self.get_alfa())

	


class ArucoList():
	def __init__(self, size=N_ARUCOS):
		self.aruco_list=[None]*size
		self.size=size

	def insert_marker(self, aruco_id, x, y, alfa, x_world, y_world, alfa_world):
		new_aruco=ArucoInfo(aruco_id, x, y, alfa, x_world, y_world, alfa_world)
		self.aruco_list[aruco_id]=new_aruco
		#print(self.aruco_list)
	
	def __str__(self):
		for i in self.aruco_list:
			print (i)

	def cleanList(self):
		self.aruco_list=[None]*self.size

	def get_size(self):
		return self.size

	def get_list(self):
		return self.aruco_list