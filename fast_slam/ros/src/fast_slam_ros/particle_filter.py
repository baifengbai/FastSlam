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

import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D


N_ARUCOS=28

#Number of particles - 1000
N_PARTICLES = 100
'''numbp = np.zeros((N_PARTICLES,3))
numbp[:,0] = np.random.uniform(-6.5,6.5,N_PARTICLES) #6.5 dimensions of room
numbp[:,1] = np.random.uniform(-6.5,6.5,N_PARTICLES)
numbp[:,2] = np.random.uniform(0,2*math.pi,N_PARTICLES)'''
#print(numbp)

#Motion Model
class ParticleFilter():

	def __init__(self, cam_transformation):
		self.listener = tf.TransformListener()
		self.particles_publisher=rospy.Publisher('particles_publisher', PoseArray, queue_size=10)
		self.ekf_publisher=rospy.Publisher('marker_estimations', PoseArray, queue_size=10)
		self.odom_prev=(0,0,0)
		self.cam_transformation=cam_transformation
		self.particle_list = [Particle(self.cam_transformation,self.odom_prev[0],self.odom_prev[1], self.odom_prev[2]) for i in range(N_PARTICLES)]
		plt.figure()
		plt.ion()
		#plt.show()

	def particle_filter_iteration(self, aruco_flag, aruco_msg, odom_pose):
		motion_model = (odom_pose[0]-self.odom_prev[0], odom_pose[1]-self.odom_prev[1], odom_pose[2]-self.odom_prev[2])
		#print(robot_position)
		self.odom_prev=(odom_pose[0], odom_pose[1], odom_pose[2])
		weights=[None]*N_PARTICLES
		particle_x=[None]*N_PARTICLES
		particle_y=[None]*N_PARTICLES
		particle_orientation=[None]*N_PARTICLES
		erro=[None]*N_PARTICLES
		total_w=0
		i=0
		for particle in self.particle_list:
			particle.particle_prediction(motion_model, aruco_flag, aruco_msg)
			if aruco_flag == True:
				particle.particle_update()
			particle_x[i]=particle.x
			particle_y[i]=particle.y
			particle_orientation[i]=particle.alfap
			erro[i]=pow(pow(particle.x,2)+pow(particle.y,2),0.5)
			total_w=total_w+particle.w
			i=i+1
		#particles weights normalization 
		if total_w>0:
			i=0
			for particle in self.particle_list:
				particle.w=particle.w*pow(total_w,-1)
				weights[i]=particle.w
				i=i+1
		
		#self.particle_publisher()

		self.resample()

		self.particle_publisher()

		
		plt.clf()
		plt.plot(erro,weights,'o')
		plt.draw()
		plt.xlabel('erro')
		plt.ylabel('weight')
		plt.pause(0.001)
		
		
		


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
		marker_array=PoseArray()
		marker_array.header.stamp=rospy.Time.now()
		marker_array.header.frame_id="/odom"
		
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
				(markers, size)=i.kf.markers_publisher()
				marker_array.poses=marker_array.poses+markers
				#print("particle pose x:%f y:%f"%(aux_pose.position.x,aux_pose.position.y))
		self.particles_publisher.publish(pose_array)
		if size>0:
			self.ekf_publisher.publish(marker_array)



		


class Particle():

	def __init__(self,cam_transformation,x=np.random.uniform(-6.5,6.5,1),y=np.random.uniform(-6.5,6.5,1),alfap=np.random.uniform(0,2*math.pi,1),w=math.pow(N_PARTICLES,-1)):
		self.x = x
		self.y = y
		self.alfap = alfap
		self.w = w #weights
		self.cam_transformation=cam_transformation
		self.kf = KalmanFilter([self.x,self.y,self.alfap],cam_transformation)

	def particle_prediction(self, motion_model, aruco_flag, aruco_msg):
		#self.x = self.x+motion_model[0]
		#self.y = self.y+motion_model[1]
		#self.alfap = self.alfap+motion_model[2]
		self.x = self.x+motion_model[0]+np.random.normal(0,0.1)
		self.y = self.y+motion_model[1]+np.random.normal(0,0.1)
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
		new_p=Particle(self.cam_transformation,self.x,self.y,self.alfap)
		new_p.kf=self.kf.kalman_copy()
		#new_p.kf=KalmanFilter([self.x,self.y,self.alfap])
		return new_p




def main():
	rospy.init_node('particle_filter_node', anonymous=False)

	particle_filter_executor = ParticleFilter()

if __name__ == '__main__':
	main()

