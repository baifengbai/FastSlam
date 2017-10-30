#!/usr/bin/env python
import rospy
from kalman_filter.my_ros_independent_class import ArucoList
from MarkerArray.msg import MarkerArray
from tf.transformations import euler_from_quaternion

class KalmanFilter():

	def __init__(self):

		self.aruco_msg = None
        self.aruco_received = False
        self.aruco_list=ArucoList()


		rospy.loginfo('Initializing kalman filter node')


		rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.arucoCallback)


	def arucoCallback(self,msg):
		self.aruco_msg=msg
		self.aruco_received=True
		rospy.loginfo('Aruco(s) detected!')

	def start_kalman_filter(self):
		while not rospy.is_shutdown():
			if self.aruco_received==True
				#lower flag
				self.aruco_received=False
				self.get_pose_arucos()
	def get_pose arucos(self):
		for i in self.aruco_msg.markers
			aruco_id=i.id
			(x,y,z)=i.pose.position
			(roll,pitch,yaw) = euler_from_quaternion([self.aruco_msg.pose.orientation.x, self.aruco_msg.pose.orientation.y, self.aruco_msg.pose.orientation.z, self.aruco_msg.pose.orientation.w])		
			self.aruco_list.insert_marker(aruco_id,x,y,-roll)

def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node',log_level=rospy.INFO , anonymous=False)
	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_kalman_filter()

if __name__ == '__main__':
	main()