#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from scipy.stats import multivariate_normal as mvn
import rospy
import math
import random
import tf
#from particle_filter.my_ros_independent_class import my_generic_sum_function
#from kalman_filter.my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import *
from fast_slam_ros.kalman_filter import KalmanFilter
from my_ros_independent_class import ArucoList
import copy

N_ARUCOS=28

#Number of particles - 1000
N_PARTICLES = 50
'''numbp = np.zeros((N_PARTICLES,3))
numbp[:,0] = np.random.uniform(-6.5,6.5,N_PARTICLES) #6.5 dimensions of room
numbp[:,1] = np.random.uniform(-6.5,6.5,N_PARTICLES)
numbp[:,2] = np.random.uniform(0,2*math.pi,N_PARTICLES)'''
#print(numbp)

#Motion Model
class ParticleFilter():

	def __init__(self):
		self.listener = tf.TransformListener()
		self.particles_publisher=rospy.Publisher('particles_publisher', PoseArray, queue_size=10)
		self.odom_prev=(0,0,0)
		self.particle_list = [Particle(self.odom_prev[0],self.odom_prev[1], self.odom_prev[2]) for i in range(N_PARTICLES)]


	def particle_filter_iteration(self, aruco_flag, aruco_msg, odom_pose):
		motion_model = (odom_pose[0]-self.odom_prev[0], odom_pose[1]-self.odom_prev[1], odom_pose[2]-self.odom_prev[2])
		#print(robot_position)
		self.odom_prev=(odom_pose[0], odom_pose[1], odom_pose[2])

		total_w=0
		for particle in self.particle_list:
			particle.particle_prediction(motion_model, aruco_flag, aruco_msg)
			if aruco_flag == True:
				particle.particle_update()
			total_w=total_w+particle.w
		if total_w>0:
			for particle in self.particle_list:
				particle.w=particle.w*pow(total_w,-1)
		
		#self.particle_publisher()

		self.resample()

		self.particle_publisher()


	def resample(self):
		new_list=[None]*N_PARTICLES
		r=random.random()*(pow(N_PARTICLES,-1))
		c=self.particle_list[0].w
		i=0
		for m in range(N_PARTICLES):
			u=r+m*pow(N_PARTICLES,-1)
			while u>c and i<m:
				i=i+1
				c=c+self.particle_list[i].w
			new_list[m]=self.particle_list[i].copy_particle()
			#print(i)
		self.particle_list=new_list

			
	def particle_publisher(self):
		#creating PoseArray object for publication
		pose_array=PoseArray()
		pose_array.header.stamp=rospy.Time.now()
		pose_array.header.frame_id="/odom"
		
		#creating a pose in the poses[] list for every aruco position being estimated
		for i in self.particle_list:
			if i!=None:
				aux_pose=Pose()
				aux_pose.position.x=i.x
				aux_pose.position.y=i.y
				aux_pose.position.z=0.275
				(ax,ay,az,aw) = tf.transformations.quaternion_from_euler(0,0,i.alfap)
				aux_pose.orientation.x=ax
				aux_pose.orientation.y=ay
				aux_pose.orientation.z=az
				aux_pose.orientation.w=aw
				pose_array.poses.append(aux_pose)

		self.particles_publisher.publish(pose_array)


		


class Particle():

	def __init__(self,x=np.random.uniform(-6.5,6.5,1),y=np.random.uniform(-6.5,6.5,1),alfap=np.random.uniform(0,2*math.pi,1),w=math.pow(N_PARTICLES,-1)):
		self.x = x
		self.y = y
		self.alfap = alfap
		self.w = w #weights
		self.kf = KalmanFilter([self.x,self.y,self.alfap])

	def particle_prediction(self, motion_model, aruco_flag, aruco_msg):
		self.x = self.x+motion_model[0]+np.random.normal(0,0.01)
		self.y = self.y+motion_model[1]+np.random.normal(0,0.01)
		self.alfap = self.alfap+motion_model[2]+np.random.normal(0,0.01)
		if aruco_flag == True:
			self.kf.start_perception(aruco_msg, [self.x,self.y,self.alfap])


	#mal
	def particle_update(self):
		for i in range(0,N_ARUCOS):
			if self.kf.arucos.aruco_list[i]!=None:
				measurement=self.kf.arucos.aruco_list[i]
				estimator=self.kf.markers_estimation[i]
				likelihood=mvn.pdf((measurement.x_world,measurement.y_world), (estimator.x,estimator.y), estimator.covariance)
				self.w=likelihood
				

	def copy_particle(self):
		new_p=Particle(self.x,self.y,self.alfap,self.w)
		new_p.kf=copy.copy(self.kf)
		#new_p.kf=KalmanFilter([self.x,self.y,self.alfap])
		return new_p




def main():
	rospy.init_node('particle_filter_node', anonymous=False)

	particle_filter_executor = ParticleFilter()

if __name__ == '__main__':
	main()

