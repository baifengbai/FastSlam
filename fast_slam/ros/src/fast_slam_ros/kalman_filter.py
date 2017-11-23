#!/usr/bin/env python
import numpy as np
import rospy
import math
import tf
from my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import *
import copy

N_ARUCOS=28
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

	def copy_marker(self):
		new_m=MarkerEstimation(self.id, self.x, self.y)
		new_m.covariance=self.covariance
		return new_m

	def get_position(self):
		#print(self.x)
		#print(self.y)
		return np.matrix([self.x, self.y]).T

	def get_cov(self):
		return self.covariance

	def get_id(self):
		return self.id

	def update_position(self, new_state):
		#print(new_state)
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

		#print("Robot pose:%s"%(robot_pose.T))

		#H matrix
		alfaPose=robot_pose[2,0]
		robot_position=np.delete(robot_pose, (2), axis=0)
		h=np.matrix([[math.cos(alfaPose), math.sin(alfaPose)], [-math.sin(alfaPose), math.cos(alfaPose)]])

		#Motion model (dummy step only to represent motion our model)
		marker_position=marker_position

		#Observation/Measurement model
		measureModel=self.measurement_model(robot_pose,marker_position)
		#print("Measure Model:%s"%(measureModel.T))

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
	
	def measurement_model(self,robot_pose,marker_position):
		#Observation/Measurement model

		#H matrix
		alfaPose=robot_pose[2,0]
		robot_position=np.delete(robot_pose, (2), axis=0)
		h=np.matrix([[math.cos(alfaPose), math.sin(alfaPose)], [-math.sin(alfaPose), math.cos(alfaPose)]])

		return h.dot(marker_position-robot_position)



class KalmanFilter():

	def __init__(self, pose):
		#msg received from aruco subscriber
		self.aruco_msg = None
		#flag of aruco_ros subscriber
		self.aruco_received = False
		self.arucos=ArucoList()
		self.markers_estimation=[None]*N_ARUCOS
		#self.listener=tf.TransformListener()
		self.particle_pose=pose

		#rospy.loginfo('Initializing kalman filter node')
		#Publisher of arucos position estimation
		#self.marker_publisher=rospy.Publisher('marker_estimations', PoseArray, queue_size=10)

		self.cov_publisher=rospy.Publisher('marker_cov', PoseWithCovarianceStamped, queue_size=10)


	def kalman_copy(self):
		new_k=KalmanFilter(self.particle_pose)
		#new_k.arucos.aruco_list=list(self.arucos.aruco_list)
		new_k.arucos.size=self.arucos.size
		for i in range(N_ARUCOS):
			if self.markers_estimation[i]!=None:
				new_k.markers_estimation[i]=self.markers_estimation[i].copy_marker()
		return new_k


	#run a kalman filter for each of the markers being observed
	def start_kalman_filter(self):
		for i in self.arucos.get_list():
			if i!=None: #for all arucos being observed
				if self.markers_estimation[i.get_id()]==None:
					#if it's the first sighting of that aruco its position is initialized
					self.markers_estimation[i.get_id()]=MarkerEstimation(i.get_id(),i.get_pose_world()[0], i.get_pose_world()[1])
					#print("First_time(world_frame): [%f,%f]"%(i.get_pose_world()[0],i.get_pose_world()[1]))
					#print("First_time(camera_frame): [%f,%f]"%(i.x,i.y))
				else:
					#robot pose in the world frame:
					#(robot_position, robot_orientation)=self.listener.lookupTransform("/odom", "/base_link", rospy.Time())

					#(robot_role, robot_pich, robot_yaw)=tf.transformations.euler_from_quaternion(robot_orientation)

					#robot_pose=(robot_position[0]+np.random.normal(0,0.5), robot_position[1]+np.random.normal(0,0.5), robot_yaw)


					#robot_pose=(robot_position[0], robot_position[1], robot_yaw)
					#if there's already an estimate for that aruco's position, a kalman filter update is performed

					#kalman filter update
					self.markers_estimation[i.get_id()].ekf_update(i.get_measurement(), self.particle_pose)
					#print("Measurement(world_frame): [%f,%f]"%(i.x_world,i.y_world))
					#print("Measurement(camera_frame): [%f,%f]"%(i.x,i.y))
					#print("------------------------")
					#print(self.markers_estimation[i.get_id()].get_cov())

		#self.markers_publisher()

	def create_detection_list(self):
		#stores every aruco being observed in an ArucoList:
		for i in self.aruco_msg.markers:
			aruco_id=i.id
			#ignore the observations of arucos that are not considered
			if aruco_id<N_ARUCOS:
				#creating a PoseStamped object to store the aruco pose:
				aruco_pose_in= PoseStamped()
				#the reference frame is the camera optical frame
				aruco_pose_in.header.frame_id="/camera_rgb_optical_frame"
				aruco_pose_in.pose=i.pose.pose


				alfaPose=self.particle_pose[2]
				robot_position=np.matrix(self.particle_pose).T
				robot_position=np.delete(robot_position, (2), axis=0)

				#transforms the aruco pose in the optical frame to the "standard" camera frame
				#object_pose_cam=self.listener.transformPose("/base_link", aruco_pose_in)
				object_pose_cam=aruco_pose_in

				marker_position=np.matrix([object_pose_cam.pose.position.x,object_pose_cam.pose.position.y]).T
				h=np.matrix([[math.cos(alfaPose), -math.sin(alfaPose)], [math.sin(alfaPose), math.cos(alfaPose)]])

				#print("robot_pose:[%f,%f,%f]"%(self.particle_pose[0],self.particle_pose[1],self.particle_pose[2]))

				#print("marker_pose:[%f,%f]"%(object_pose_cam.pose.position.x,object_pose_cam.pose.position.y))

				object_pose_world= h.dot(marker_position)+robot_position

				#stores the aruco in the list
				self.arucos.insert_marker(aruco_id,object_pose_cam.pose.position.x,object_pose_cam.pose.position.y,0,object_pose_world[0,0],object_pose_world[1,0],0)
				#rospy.loginfo('Aruco %d  detected!'%(aruco_id))

	def markers_publisher(self):
		#creating PoseArray object for publication
		#pose_array=PoseArray()
		pose_array=[Pose()]*0
		size=0
		#pose_array.header.stamp=rospy.Time.now()
		#pose_array.header.frame_id="/odom"
		
		#creating a pose in the poses[] list for every aruco position being estimated
		for i in self.markers_estimation:
			if i!=None:
				#print(i)
				mpose=i.get_position()
				aux_pose=Pose()
				aux_pose.position.x=mpose[0,0]
				aux_pose.position.y=mpose[1,0]
				aux_pose.position.z=0.275
				aux_pose.orientation.x=0
				aux_pose.orientation.y=-0.707
				aux_pose.orientation.z=0
				aux_pose.orientation.w=0.707
				#pose_array.poses.append(aux_pose)
				pose_array.append(aux_pose)
				size=size+1

		#self.marker_publisher.publish(pose_array)
		return pose_array, size

		'''if self.markers_estimation[0]!=None:
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
			#print(zerosm)
			#print(covmatrix+zerosp)
			covpose.covariance=zerosm
			covposest.pose=covpose

			self.cov_publisher.publish(covposest)'''


	def start_perception(self, msg, pose):
				#reset observations list
				self.aruco_msg=msg

				self.particle_pose=pose
				self.arucos.cleanList()
				self.create_detection_list()
				self.start_kalman_filter()


def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)

	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_perception()

if __name__ == '__main__':
	main()