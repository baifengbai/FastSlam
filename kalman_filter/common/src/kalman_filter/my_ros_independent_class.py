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
	def __repr__(self):
		return "id:%d x:%f y:%f alfa:%f"%(self.get_id(),self.get_x(),self.get_y(),self.get_alfa())


class ArucoList():
	def __init__(self, size=16):
		self.aruco_list=[None]*size
		self.size=size

	def insert_marker(self, aruco_id, x, y, alfa):
		new_aruco=ArucoInfo(aruco_id, x, y, alfa);
		self.aruco_list[aruco_id]=new_aruco
		print(self.aruco_list)
	def __str__(self):
		for i in self.aruco_list:
			print (i)




'''def ekfupdate(state, measurement, pose, expectedValue, cov):

	#H matrix
	alfaPose=state[2]
	h=np.matrix([[math.cos(alfaPose), math.sin(alfaPose), 0], [-math.sin(alfaPose), math.cos(alfaPose), 0], [0, 0, 1]])

	#Motion model
	motionModel=state

	#Observation model
	measureModel=-pose+h.dot(state)
	measureCov=???

	#Prediction step
	predExpectedValue=expectedValue
	predCov=cov 

	#Update step
	kalmanGain=predCov.dot(h.transpose()).dot(np.linalg.inv(h.dot(predCov).dot(h.transpose()).dot(measureCov)))
	updateExpectedValue=predExpectedValue+kalmanGain.dot(measurement-measureModel)
	updateCov=(np.identity(3)-kalmanGain.dot(h)).dot(predCov)'''