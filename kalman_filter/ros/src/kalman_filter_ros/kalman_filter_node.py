#!/usr/bin/env python
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

	def __init__(self,x,y,alpha=0):
		self.id
		self.x=x
		self.y=y
		self.orientation=alpha
		self.covariance=numpy.identity(3)

class KalmanFilter():

	def __init__(self):
		self.aruco_msg = None
		self.aruco_received = False
		self.aruco_list=ArucoList()
		self.markers_estimation=[None]*self.aruco_list.get_size()
		self.listener=tf.TransformListener()

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
		

	def create_detection_list(self):

		for i in self.aruco_msg.markers:
			object_pose_in= PoseStamped()
			object_pose_in.header.stamp=rospy.Time.now()
			object_pose_in.header.frame_id="/camera_rgb_optical_frame"
			object_pose_in.pose=i.pose.pose
			aruco_id=i.id
			now=rospy.Time.now()
			self.listener.waitForTransform("/odom", "/camera_rgb_optical_frame", now, rospy.Duration(1.0))
			object_pose_bl=self.listener.transformPose("/odom",object_pose_in)
			x=object_pose_bl.pose.position.x
			y=object_pose_bl.pose.position.y
			(roll,pitch,yaw) = euler_from_quaternion([object_pose_bl.pose.orientation.x, object_pose_bl.pose.orientation.y, object_pose_bl.pose.orientation.z, object_pose_bl.pose.orientation.w])		
			

			self.aruco_list.insert_marker(aruco_id,x,y,yaw)
			print ("\n X=%f | Y=%f | Roll=%f | Pitch=%f | Yaw=%f \n"%(x, y, roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi))


def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)
	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_perception()

if __name__ == '__main__':
	main()