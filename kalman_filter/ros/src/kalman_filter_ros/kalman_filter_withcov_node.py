#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import *
from kalman_filter_ros.kalman_filter_ros import MarkerEstimation
from kalman_filter_ros.kalman_filter_ros import KalmanFilter

class VectorWithCovariances():

	def vector_with_cov(self):
		return np.matrix([self.x, self.y, self.covariance([[0],[0]]), self.covariance([[1],[1]])]).T

def main():

	rospy.init_node('kalman_filter_withcov_node', anonymous=False)

if __name__ == '__main__':
	main()
