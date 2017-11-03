#!/usr/bin/env python

import numpy as np
import math

class ArucoInfo():
	def __init__(self, arucoId, arucox, arucoy, arucoAlfa):
		self.id=arucoId
		self.x=arucox
		self.y=arucoy
		self.alfa=arucoAlfa
		arucoPose=[self.x, self.y, self.alfa]
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
	def __repr__(self):
		return "id:%d x:%f y:%f alfa:%f"%(self.get_id(),self.get_x(),self.get_y(),self.get_alfa())

	


class ArucoList():
	def __init__(self, size=16):
		self.aruco_list=[None]*size
		self.size=size

	def insert_marker(self, aruco_id, x, y, alfa):
		new_aruco=ArucoInfo(aruco_id, x, y, alfa);
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



