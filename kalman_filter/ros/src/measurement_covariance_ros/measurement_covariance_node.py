import numpy as np
import math
from aruco_msgs.msg import MarkerArray	
import rospy
import tf
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion

class Covariance():

	def __init__(self,x=0.075,y=1,alpha=0, id=0):
		'''
		function to get the covariance of observations
		by default it works for aruco_0	
		'''
		#create a column vector with real position
		self.real_vector=np.array([x,y,alpha])
		self.real_vector.shape = (3,1)
		#matrix with all the observations
		self.obsr_vectors=np.array([])
		#msg received by aruco subscriber
		self.aruco_msg=None
		#flag for subscriber
		self.aruco_received=False
		#aruco id
		self.id=0

		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
		self.listener = tf.TransformListener()

	def start_perception(self):
			while not rospy.is_shutdown():
				if self.aruco_received==True:
					self.aruco_received=False
					self.add_detection()

	def add_detection(self):
		for i in self.aruco_msg.markers:
			if i!= None:
				if self.id==i.id:

					object_pose_in= PoseStamped()
					object_pose_in.header.stamp=rospy.Time()
					object_pose_in.header.frame_id="/camera_rgb_optical_frame"
					object_pose_in.pose=i.pose.pose
					object_pose_bl=self.listener.transformPose("/odom",object_pose_in)
					
					x=object_pose_bl.pose.position.x
					y=object_pose_bl.pose.position.y
					(roll,pitch,yaw) = tf.transformations.euler_from_quaternion([object_pose_bl.pose.orientation.x, object_pose_bl.pose.orientation.y, object_pose_bl.pose.orientation.z, object_pose_bl.pose.orientation.w])		
					#add the new observation to the matrix of observations
					if self.obsr_vectors.size==0:
						self.obsr_vectors=np.array([x,y,yaw])
						self.obsr_vectors.shape = (3,1)
					else:
						np.c_[self.obsr_vectors,[x,y,yaw]] 


	def aruco_callback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True

	def get_obsr_vectores(self):
		return self.obsr_vectors

def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('measurement_covariance_node', anonymous=False)
	cov= Covariance()
	cov.start_perception()
	cov = np.cov(cov.get_obsr_vectors())
	rospy.set_param('measurement_covariance', cov.tolist())

if __name__ == '__main__':
	main()