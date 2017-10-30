#!/usr/bin/env python
import rospy
from kalman_filter.my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray	
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
		rospy.loginfo('%d Aruco(s) detected!'%(len(self.aruco_msg.markers)))

	def start_kalman_filter(self):
		while not rospy.is_shutdown():
			if self.aruco_received==True:
				print("Esta true")
				self.aruco_received=False
				self.get_pose_arucos()

	def get_pose_arucos(self):
		for i in self.aruco_msg.markers:
			aruco_id=i.id
			x=i.pose.pose.position.z
			y=-i.pose.pose.position.x
			print("bora inserir na lista x:%d y:%d"%(x,y))
			(roll,pitch,yaw) = euler_from_quaternion([i.pose.pose.orientation.x, i.pose.pose.orientation.y, i.pose.pose.orientation.z, i.pose.pose.orientation.w])		
			
			self.aruco_list.insert_marker(aruco_id,x,y,-roll)

def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)
	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_kalman_filter()

if __name__ == '__main__':
	main()