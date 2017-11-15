#!/usr/bin/env python
import numpy as np
import rospy
import math
import tf
from kalman_filter.my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import *

#Covariance matrix R that represents the covariance of the Gaussian noise of observations
COVARIANCE_MATRIX=np.matrix([[1.48377597e-01, 2.37789852e-04],[2.37789852e-04, 1.47362967e-01]])
#COVARIANCE_MATRIX=np.matrix([[1.44433477e-04, 2.37789852e-04],[2.37789852e-04, 3.06948739e-03]])

class MarkerEstimation():

	#alpha is not being used (we donot consider the orientation of the marker)
	def __init__(self,aid,x,y,alpha=0):
		self.id=aid
		self.x=x
		self.y=y
		self.orientation=alpha
		self.covariance=COVARIANCE_MATRIX

	def __str__(self):
		return "markers_estimation[%d]: x:%.4f y:%.4f"%(self.id,self.x,self.y)

	def get_position(self):
		return np.matrix([self.x, self.y]).T

	def get_cov(self):
		return self.covariance

	def get_id(self):
		return self.id

	def update_position(self, new_state):
		self.x=new_state[0,0]
		self.y=new_state[1,0]

	def update_cov(self, new_cov):
		self.covariance=new_cov


	def ekf_update(self, measurement, robot_pose):


		marker_position=self.get_position()
		marker_cov=self.get_cov()

		#to put the robot pose in column format
		robot_pose=np.matrix(robot_pose)
		robot_pose=robot_pose.T

		#to put the measurement in column format
		measurement=np.matrix(measurement)
		measurement=measurement.T
		#ignore measured orientation of aruco 
		measurement=np.delete(measurement, (2), axis=0)

		print("Robot pose:%s"%(robot_pose.T))

		#H matrix
		alfaPose=robot_pose[2,0]
		robot_position=np.delete(robot_pose, (2), axis=0)
		h=np.matrix([[math.cos(alfaPose), math.sin(alfaPose)], [-math.sin(alfaPose), math.cos(alfaPose)]])

		#Motion model (dummy step only to represent motion our model)
		marker_position=marker_position

		#Observation/Measurement model
		measureModel=h.dot(marker_position-robot_position)
		
		print("Measure Model:%s"%(measureModel.T))

		#Prediction step
		predExpectedValue=marker_position
		predCov=marker_cov 

		#Update step
		kalmanGain=predCov.dot(h.transpose()).dot(np.linalg.inv(h.dot(predCov).dot(h.transpose())+(COVARIANCE_MATRIX)))
		updateExpectedValue=predExpectedValue+kalmanGain.dot(measurement-measureModel)
		updateCov=(np.identity(2)-kalmanGain.dot(h)).dot(predCov)

		#Update the Covariance and the Expected value of aruco
		self.update_position(updateExpectedValue)
		self.update_cov(updateCov)

class KalmanFilter():

	def __init__(self):
		#msg received from aruco subscriber
		self.aruco_msg = None
		#flag of aruco_ros subscriber
		self.aruco_received = False
		self.aruco_list=ArucoList()
		self.markers_estimation=[None]*self.aruco_list.get_size()
		self.listener=tf.TransformListener()

		rospy.loginfo('Initializing kalman filter node')
		#Subscriber of aruco publisher topic with arucos observations
		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
		#Publisher of arucos position estimation
		self.marker_publisher=rospy.Publisher('marker_estimations', PoseArray, queue_size=10)

		self.cov_publisher=rospy.Publisher('marker_cov', PoseWithCovarianceStamped, queue_size=10)

	def aruco_callback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True
		rospy.loginfo('%d Aruco(s) detected!'%(len(self.aruco_msg.markers)))


	#run a kalman filter for each of the markers being observed
	def start_kalman_filter(self):
		for i in self.aruco_list.get_list():
			if i!=None: #for all arucos being observed
				if self.markers_estimation[i.get_id()]==None:
					#if it's the first sighting of that aruco its position is initialized with the first observation values
					self.markers_estimation[i.get_id()]=MarkerEstimation(i.get_id(),i.get_measurement()[0], i.get_measurement()[1])
				else:
					#if there's already an estimate for that aruco's position, a kalman filter update is performed
					#robot pose in the world frame:
					(robot_position, robot_orientation)=self.listener.lookupTransform("/odom", "/base_link", rospy.Time())

					(robot_role, robot_pich, robot_yaw)=tf.transformations.euler_from_quaternion(robot_orientation)
					robot_pose=(robot_position[0]+np.random.normal(0,0.5), robot_position[1]+np.random.normal(0,0.5), robot_yaw)

					#kalman filter update
					self.markers_estimation[i.get_id()].ekf_update(i.get_measurement(), robot_pose)
					print("Measurement: %s"%(i))
					print(self.markers_estimation[i.get_id()].get_cov())

		self.markers_publisher()

	def create_detection_list(self):
		#stores every aruco being observed in an ArucoList:
		for i in self.aruco_msg.markers:
			#creating a PoseStamped object to store the aruco pose:
			aruco_pose_in= PoseStamped()
			#the reference frame is the camera optical frame
			aruco_pose_in.header.frame_id="/camera_rgb_optical_frame"
			aruco_pose_in.pose=i.pose.pose
			aruco_id=i.id

			#transforms the aruco pose in the optical frame to the "standard" camera frame
			object_pose_cam=self.listener.transformPose("/camera_rgb_frame", aruco_pose_in)
			x=object_pose_cam.pose.position.x
			y=object_pose_cam.pose.position.y
			(roll,pitch,yaw) = tf.transformations.euler_from_quaternion([object_pose_cam.pose.orientation.x, object_pose_cam.pose.orientation.y, object_pose_cam.pose.orientation.z, object_pose_cam.pose.orientation.w])		
			
			#stores the aruco in the list
			self.aruco_list.insert_marker(aruco_id,x,y,yaw)
	
	def markers_publisher(self):
		#creating PoseArray object for publication
		pose_array=PoseArray()
		pose_array.header.stamp=rospy.Time.now()
		pose_array.header.frame_id="/odom"
		
		#creating a pose in the poses[] list for every aruco position being estimated
		for i in self.markers_estimation:
			if i!=None:
				print(i)
				mpose=i.get_position()
				aux_pose=Pose()
				aux_pose.position.x=mpose[0,0]
				aux_pose.position.y=mpose[1,0]
				aux_pose.position.z=0.275
				aux_pose.orientation.x=0
				aux_pose.orientation.y=-0.707
				aux_pose.orientation.z=0
				aux_pose.orientation.w=0.707
				pose_array.poses.append(aux_pose)

		self.marker_publisher.publish(pose_array)

		if self.markers_estimation[0]!=None:
			covposest=PoseWithCovarianceStamped()
			covposest.header.stamp=rospy.Time.now()
			covposest.header.frame_id="/odom"
			covpose=PoseWithCovariance()
			mmpose=self.markers_estimation[0].get_position()
			aaux_pose=Pose()
			aaux_pose.position.x=mmpose[0,0]
			aaux_pose.position.y=mmpose[1,0]
			aaux_pose.position.z=0.275
			aaux_pose.orientation.x=0
			aaux_pose.orientation.y=-0.707
			aaux_pose.orientation.z=0
			aaux_pose.orientation.w=0.707
			covpose.pose=aaux_pose
			covmatrix=self.markers_estimation[0].get_cov()
			zerosm=np.zeros((6,6))
			zerosm[0,0]=covmatrix[0,0]
			zerosm[0,1]=covmatrix[0,1]
			zerosm[1,0]=covmatrix[1,0]
			zerosm[1,1]=covmatrix[1,1]
			zerosm=zerosm.flatten()
			zerosm=zerosm.tolist()
			#print(covmatrix)
			print(zerosm)
			#print(covmatrix+zerosp)
			covpose.covariance=zerosm
			covposest.pose=covpose

			self.cov_publisher.publish(covposest)


	def start_perception(self):
		while not rospy.is_shutdown():
			if self.aruco_received==True:
				#set the flag to false
				self.aruco_received=False
				#reset observations list
				self.aruco_list.cleanList()
				self.create_detection_list()
				self.start_kalman_filter()


def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)

	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_perception()

if __name__ == '__main__':
	main()