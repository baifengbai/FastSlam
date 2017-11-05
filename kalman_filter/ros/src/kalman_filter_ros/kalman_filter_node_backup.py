#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import numpy
import rospy
import math
import tf
from kalman_filter.my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray	
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from geometry_msgs.msg import *



class MarkerEstimation():

	def __init__(self,aid,x,y,alpha=0):
		self.id=aid
		self.x=x
		self.y=y
		self.orientation=alpha
		self.covariance=numpy.identity(3)*0.000001

	def __str__(self):
		return "id:%d x:%f y:%f alfa:%f"%(self.get_id(),self.get_state()[0,0],self.get_state()[0,1],self.get_state()[0,2])

	def get_state(self):
		return np.matrix([self.x, self.y, self.orientation])

	def get_cov(self):
		return self.covariance

	def get_id(self):
		return self.id

	def set_state(self, new_state):
		self.x=new_state[0,0]
		self.y=new_state[1,0]
		self.orientation=new_state[2,0]

	def set_cov(self, new_cov):
		self.covariance=new_cov

	def ekfupdate(self, measurement, pose, pose_world):


		state=self.get_state()
		state=state.T
		cov=self.get_cov()
		pose=np.matrix(pose)
		pose=pose.T
		measurement=np.matrix(measurement)
		measurement=measurement.T
		measurement[2,0]=0
		pose_world=np.matrix(pose_world)
		pose_world=pose_world.T

		#H matrix
		alfaPose=pose[2,0]
		h=np.matrix([[math.cos(alfaPose), math.sin(alfaPose), 0], [-math.sin(alfaPose), math.cos(alfaPose), 0], [0, 0, 1]])

		#Motion model
		motionModel=state

		#Observation model
		measureModel=pose+h.dot(pose_world)
		measureModel[2,0]=0
		print("Pose world: ", end="")
		print(pose_world)
		print("Measure Model: ", end="")
		print(measureModel)
		#measureCov=np.identity(3)*0.1
		#measureCov=np.matrix([[1.44433477e-04, 2.37789852e-04, -1.14394555e-03],[2.37789852e-04, 3.06948739e-03, 1.39377945e-02],[-1.14394555e-03, 1.39377945e-02, 3.90728455e+00]])
		measureCov=np.matrix([[1.44433477e-04, 0, 0],[0, 3.06948739e-03, 0],[0, 0, 3.90728455e+00]])

		#Prediction step
		predExpectedValue=state
		predCov=cov 

		#Update step
		kalmanGain=predCov.dot(h.transpose()).dot(np.linalg.inv(h.dot(predCov).dot(h.transpose())+(measureCov)))
		updateExpectedValue=predExpectedValue+kalmanGain.dot(measurement-measureModel)
		updateCov=(np.identity(3)-kalmanGain.dot(h)).dot(predCov)

		#Set values
		self.set_state(updateExpectedValue)
		self.set_cov(updateCov)
		#print(updateExpectedValue)
		#print(updateCov)

class KalmanFilter():

	def __init__(self):
		self.aruco_msg = None
		self.aruco_received = False
		self.aruco_list=ArucoList()
		self.markers_estimation=[None]*self.aruco_list.get_size()
		self.listener=tf.TransformListener()
		self.marker_publisher=rospy.Publisher('marker_estimations', PoseArray, queue_size=10)

		rospy.loginfo('Initializing kalman filter node')


		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)


	def aruco_callback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True
		rospy.loginfo('%d Aruco(s) detected!'%(len(self.aruco_msg.markers)))

	def start_perception(self):
		while not rospy.is_shutdown():
			if self.aruco_received==True:
				self.aruco_received=False
				self.aruco_list.cleanList()
				self.create_detection_list()
				self.start_kalman_filter()

	def start_kalman_filter(self):
		for i in self.aruco_list.get_list():
			if i!=None:
				if self.markers_estimation[i.get_id()]==None:
					self.markers_estimation[i.get_id()]=MarkerEstimation(i.get_id(),i.get_pose_world()[0], i.get_pose_world()[1])
				else:
					now=rospy.Time()
					#self.listener.waitForTransform("/base_link", "/odom", now, rospy.Duration(1.0))
					(robot_position, robot_orientation)=self.listener.lookupTransform("/base_link", "/odom", now)
					(robot_alfa, robot_beta, robot_gama)=euler_from_quaternion(robot_orientation)
					robot_pose=(robot_position[0], robot_position[1], robot_gama)
					self.markers_estimation[i.get_id()].ekfupdate(i.get_measurement(), robot_pose, i.get_pose_world())
					print("Robot pose: ", end="")
					print(robot_pose, end="")
					print("  | Measurement: ", end="")
					print(i.get_measurement(), end="")
					print("  | State: ", end="")
					print(i.get_pose_world())
					self.markers_publisher()

	def create_detection_list(self):

		for i in self.aruco_msg.markers:
			object_pose_in= PoseStamped()
			#object_pose_in.header.stamp=rospy.Time.now()
			object_pose_in.header.frame_id="/camera_rgb_optical_frame"
			object_pose_in.pose=i.pose.pose
			aruco_id=i.id
			#now=rospy.Time.now()
			#self.listener.waitForTransform("/odom", "/camera_rgb_optical_frame", now, rospy.Duration(1.0))
			object_pose_bl=self.listener.transformPose("/odom",object_pose_in)
			object_pose_cam=self.listener.transformPose("/camera_rgb_frame", object_pose_in)
			x=object_pose_cam.pose.position.x
			y=object_pose_cam.pose.position.y
			(roll,pitch,yaw) = euler_from_quaternion([i.pose.pose.orientation.x, i.pose.pose.orientation.y, i.pose.pose.orientation.z, i.pose.pose.orientation.w])		
			
			self.aruco_list.insert_marker(aruco_id,x,y,yaw, object_pose_bl.pose.position.x, object_pose_bl.pose.position.y, 0)
			#print ("World: X=%f | Y=%f | Roll=%f | Pitch=%f | Yaw=%f"%(object_pose_bl.pose.position.x, object_pose_bl.pose.position.y, roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi))
			#print ("Camer: X=%f | Y=%f | Roll=%f | Pitch=%f | Yaw=%f"%(x, y, roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi))
	
	def markers_publisher(self):
		pose_array=PoseArray()
		pose_array.header.stamp=rospy.Time.now()
		pose_array.header.frame_id="/odom"
		counter=0
		for i in self.markers_estimation:
			if i!=None:
				print(i)
				mpose=i.get_state()
				aux_pose=Pose()
				aux_pose.position.x=mpose[0,0]
				aux_pose.position.y=mpose[0,1]
				aux_pose.position.z=0.275
				aux_pose.orientation.x=0
				aux_pose.orientation.y=0
				aux_pose.orientation.z=0
				aux_pose.orientation.w=0
				pose_array.poses.append(aux_pose)
			else:
				print("markers_estimation[%d] is empty"%(counter))
			counter=counter+1
		self.marker_publisher.publish(pose_array)


def publish_map():
	map_publisher=rospy.Publisher('map_arucos', PoseArray, queue_size=10)
	map_array=PoseArray()
	map_array.header.stamp=rospy.Time.now()
	map_array.header.frame_id="/odom"
	



def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)

	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_perception()

if __name__ == '__main__':
	main()