#!/usr/bin/env python
import fast_slam_ros.kalman_filter
from fast_slam_ros.particle_filter import ParticleFilter
import rospy
from aruco_msgs.msg import MarkerArray

N_ARUCOS=16
SAMPLING_FREQUENCY=0.5


class FastSlam():

	def __init__(self):
		self.particle_filter_executor = ParticleFilter()
		#msg received from aruco subscriber
		self.aruco_msg = None
		#flag of aruco_ros subscriber
		self.aruco_received = False
		#Subscriber of aruco publisher topic with arucos observations
		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.aruco_callback)
		


	def aruco_callback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True

	def start_slam(self):
		r = rospy.Rate(SAMPLING_FREQUENCY)
		t=rospy.get_time()
		while not rospy.is_shutdown():
			print(rospy.get_time()-t)
			t=rospy.get_time()
			self.particle_filter_executor.particle_filter_iteration(self.aruco_received, self.aruco_msg)
			r.sleep()

			


def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('fast_slam', anonymous=False)

	fast_slam_executor = FastSlam()
	fast_slam_executor.start_slam()


if __name__ == '__main__':
	main()