#!/usr/bin/env python
import fast_slam_ros.kalman_filter
from fast_slam_ros.particle_filter import ParticleFilter
import rospy
from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import *
import tf

N_ARUCOS=28
SAMPLING_FREQUENCY=100


class FastSlam():

	def __init__(self):
		self.particle_filter_executor = ParticleFilter()
		#msg received from aruco subscriber
		self.aruco_msg = None
		#flag of aruco_ros subscriber
		self.aruco_received = False
		#Subscriber of aruco publisher topic with arucos observations
		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
		rospy.Subscriber('/RosAria/pose', Odometry, self.pose_callback)
		self.odom_pose=(0,0,0)
		self.odom_msg=None
		self.odom_flag=False
		self.br=tf.TransformBroadcaster()
		


	def aruco_callback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True

	def start_slam(self):
		r = rospy.Rate(SAMPLING_FREQUENCY)
		t=rospy.get_time()
		n_iter=0
		while not rospy.is_shutdown():
			print(rospy.get_time()-t)
			print(n_iter)
			t=rospy.get_time()
			if self.odom_flag==True:
				odom_position=self.odom_msg.pose.pose.position
				(roll, pitch, yaw)=tf.transformations.euler_from_quaternion((self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w))
				self.odom_pose=(odom_position.x, odom_position.y, yaw)
				self.br.sendTransform((self.odom_pose[0], self.odom_pose[1], 0), tf.transformations.quaternion_from_euler(0,0,self.odom_pose[2]), rospy.Time.now(),"base_link","odom")
				self.odom_flag=False
			self.particle_filter_executor.particle_filter_iteration(self.aruco_received, self.aruco_msg, self.odom_pose)
			n_iter=n_iter+1
			r.sleep()

	def pose_callback(self,msg):
		self.odom_flag=True
		self.odom_msg=msg

			


def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('fast_slam', anonymous=False)

	fast_slam_executor = FastSlam()
	fast_slam_executor.start_slam()


if __name__ == '__main__':
	main()