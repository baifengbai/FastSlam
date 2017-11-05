#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import rospy
import math
import tf
#from particle_filter.my_ros_independent_class import my_generic_sum_function
#from kalman_filter.my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import *

#Number of particles - 1000
N_PARTICLES = 1000
numbp = np.zeros((N_PARTICLES,3))
numbp[:,0] = np.random.uniform(-6.5,6.5,N_PARTICLES) #6.5 dimensions of room
numbp[:,1] = np.random.uniform(-6.5,6.5,N_PARTICLES)
numbp[:,2] = np.random.uniform(0,2*math.pi,N_PARTICLES)
print(numbp)

#Motion Model
class ParticleFilter():

	def __init__(self):
		self.listener = tf.TransformListener()
		self.particle_list = [Particle() for i in range(N_PARTICLES)]
		


class Particle():

	def __init__(self,x=np.random.uniform(-6.5,6.5,1),y=np.random.uniform(-6.5,6.5,1),alfap=np.random.uniform(0,2*math.pi,1),w=1/N_PARTICLES):
		self.x = x
		self.y = y
		self.alfap = alfap
		self.w = w #weights



		(robot_position, robot_orientation)=self.listener.lookupTransform("/odom", "/base_link", now)



def main():
	rospy.init_node('particle_filter_node', anonymous=False)

	particle_filter_executor = ParticleFilter()

if __name__ == '__main__':
	main()

