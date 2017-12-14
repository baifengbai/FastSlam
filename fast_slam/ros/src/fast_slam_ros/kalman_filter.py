#!/usr/bin/env python
import numpy as np
import rospy
import math
import tf
from my_ros_independent_class import ArucoList
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import *
import copy
import tf2_geometry_msgs

import matplotlib.pyplot as plt

MAP_TESTBED=[[3.2628,-1.292720],[-2.04076,-2.628],[2.241404,1.097244],[-1.287612,-3.342672],[-1.497996,1.064073],[0.922168,-1.763329],[-2.095216,0.483259],[2.996120,0.537353],[0.3016118 ,-1.776010],[-0.386236,1.204677],[0.613022,1.227358],[-2.036722,-0.896368],[3.250955,-2.338133],[-2.0512,-1.4596],[2.4315,-2.740110],[-0.55255,-3.1409],[-1.615080,0.687745],[-2.057349,-1.88958],[3.306175,-0.348601]]
MAP_ID=[23,30,31,34,37,38,39,41,43,44,47,49,50,51,53,55,57,60,61]

N_ARUCOS=100
#Covariance matrix R that represents the covariance of the Gaussian noise of observations
COVARIANCE_MATRIX=np.matrix([[1.48377597e-01, 2.37789852e-04],[2.37789852e-04, 1.47362967e-01]])*2
#COVARIANCE_MATRIX=np.matrix([[1.44433477e-04, 2.37789852e-04],[2.37789852e-04, 3.06948739e-03]])

class MarkerEstimation():

	#alpha is not being used (we donot consider the orientation of the marker)
	def __init__(self,aid,x,y,cov,alpha=0):
		self.id=aid
		self.x=x
		self.y=y
		self.orientation=alpha
		self.covariance=cov

	def __str__(self):
		return "markers_estimation[%d]: x:%.4f y:%.4f"%(self.id,self.x,self.y)

	def copy_marker(self):
		new_m=MarkerEstimation(self.id, self.x, self.y, self.covariance)
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

	def __init__(self, pose,cam_transformation):
		#msg received from aruco subscriber
		self.aruco_msg = None
		#flag of aruco_ros subscriber
		self.aruco_received = False
		self.arucos=ArucoList()
		self.markers_estimation=[None]*N_ARUCOS
		#self.listener=tf.TransformListener()
		self.particle_pose=pose
		self.cam_transformation=cam_transformation
		#rospy.loginfo('Initializing kalman filter node')
		#Publisher of arucos position estimation
		#self.marker_publisher=rospy.Publisher('marker_estimations', PoseArray, queue_size=10)

		#self.cov_publisher=rospy.Publisher('marker_cov', PoseWithCovarianceStamped, queue_size=10)

	def kalman_copy(self):
		new_k=KalmanFilter(self.particle_pose, self.cam_transformation)
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
					h=np.matrix([[math.cos(self.particle_pose[2]), math.sin(self.particle_pose[2])], [-math.sin(self.particle_pose[2]), math.cos(self.particle_pose[2])]])
					h_inv=np.linalg.inv(h)
					cov_robot_frame=h_inv.dot(COVARIANCE_MATRIX).dot(h_inv.transpose())
					self.markers_estimation[i.get_id()]=MarkerEstimation(i.get_id(),i.get_pose_world()[0], i.get_pose_world()[1], cov_robot_frame)
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
				object_pose_cam=tf2_geometry_msgs.do_transform_pose(aruco_pose_in, self.cam_transformation)
				#object_pose_cam=aruco_pose_in

				marker_position=np.matrix([object_pose_cam.pose.position.x,object_pose_cam.pose.position.y]).T
				h=np.matrix([[math.cos(alfaPose), -math.sin(alfaPose)], [math.sin(alfaPose), math.cos(alfaPose)]])

				#print("robot_pose:[%f,%f,%f]"%(self.particle_pose[0],self.particle_pose[1],self.particle_pose[2]))

				#print("marker_pose:[%f,%f]"%(object_pose_cam.pose.position.x,object_pose_cam.pose.position.y))

				object_pose_world= h.dot(marker_position)+robot_position

				#stores the aruco in the list
				self.arucos.insert_marker(aruco_id,object_pose_cam.pose.position.x,object_pose_cam.pose.position.y,0,object_pose_world[0,0],object_pose_world[1,0],0)
				#rospy.loginfo('Aruco %d  detected!'%(aruco_id))

	def markers_publisher(self,flag=False):
		#creating PoseArray object for publication
		#pose_array=PoseArray()
		pose_array=[Pose()]*0
		size=0
		#pose_array.header.stamp=rospy.Time.now()
		#pose_array.header.frame_id="/odom"
		
		#creating a pose in the poses[] list for every aruco position being estimated
		id=0;
		real_map=[]
		estimated_map=[]
		if flag:
			plt.ion()
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
				real_map.append(MAP_TESTBED[MAP_ID.index(id)])
				estimated_map.append([i.x , i.y])			
			id=id+1;


		#self.marker_publisher.publish(pose_array)
				#print("marker pose x:%f y:%f"%(aux_pose.position.x,aux_pose.position.y))
		real_map=np.array(real_map)
		estimated_map=np.array(estimated_map)
		print("------real_map-----")
		print(real_map)
		print("------estimated_map-----")
		print(estimated_map)
		if estimated_map.size !=0:
			_,procrustes_map,_= procrustes(real_map,estimated_map,False)
			if flag:
				plt.clf()
				line_real, =plt.plot(real_map[:,0], real_map[:,1], 'ro',label="real_map")
				line_estimated, =plt.plot(estimated_map[:,0], estimated_map[:,1], 'bo',label="estimated_map")
				line_procrusted, =plt.plot(procrustes_map[:,0], procrustes_map[:,1], 'co',label="procrustes_map")
				plt.legend(handles=[line_real, line_estimated, line_procrusted])
				plt.draw()
				plt.pause(0.001)
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





def procrustes(X, Y, scaling=True, reflection='best'):
    """
    A port of MATLAB's `procrustes` function to Numpy.

    Procrustes analysis determines a linear transformation (translation,
    reflection, orthogonal rotation and scaling) of the points in Y to best
    conform them to the points in matrix X, using the sum of squared errors
    as the goodness of fit criterion.

        d, Z, [tform] = procrustes(X, Y)

    Inputs:
    ------------
    X, Y    
        matrices of target and input coordinates. they must have equal
        numbers of  points (rows), but Y may have fewer dimensions
        (columns) than X.

    scaling 
        if False, the scaling component of the transformation is forced
        to 1

    reflection
        if 'best' (default), the transformation solution may or may not
        include a reflection component, depending on which fits the data
        best. setting reflection to True or False forces a solution with
        reflection or no reflection respectively.

    Outputs
    ------------
    d       
        the residual sum of squared errors, normalized according to a
        measure of the scale of X, ((X - X.mean(0))**2).sum()

    Z
        the matrix of transformed Y-values

    tform   
        a dict specifying the rotation, translation and scaling that
        maps X --> Y

    """

    n,m = X.shape
    ny,my = Y.shape

    muX = X.mean(0)
    muY = Y.mean(0)

    X0 = X - muX
    Y0 = Y - muY

    ssX = (X0**2.).sum()
    ssY = (Y0**2.).sum()

    # centred Frobenius norm
    normX = np.sqrt(ssX)
    normY = np.sqrt(ssY)

    # scale to equal (unit) norm
    X0 /= normX
    Y0 /= normY

    if my < m:
        Y0 = np.concatenate((Y0, np.zeros(n, m-my)),0)

    # optimum rotation matrix of Y
    A = np.dot(X0.T, Y0)
    U,s,Vt = np.linalg.svd(A,full_matrices=False)
    V = Vt.T
    T = np.dot(V, U.T)

    if reflection is not 'best':

        # does the current solution use a reflection?
        have_reflection = np.linalg.det(T) < 0

        # if that's not what was specified, force another reflection
        if reflection != have_reflection:
            V[:,-1] *= -1
            s[-1] *= -1
            T = np.dot(V, U.T)

    traceTA = s.sum()

    if scaling:

        # optimum scaling of Y
        b = traceTA * normX / normY

        # standarised distance between X and b*Y*T + c
        d = 1 - traceTA**2

        # transformed coords
        Z = normX*traceTA*np.dot(Y0, T) + muX

    else:
        b = 1
        d = 1 + ssY/ssX - 2 * traceTA * normY / normX
        Z = normY*np.dot(Y0, T) + muX

    # transformation matrix
    if my < m:
        T = T[:my,:]
    c = muX - b*np.dot(muY, T)

    #transformation values 
    tform = {'rotation':T, 'scale':b, 'translation':c}

    return d, Z, tform



def main():
	
	#inicialization of the node for the kalman filter
	rospy.init_node('kalman_filter_node', anonymous=False)

	kalman_filter_executor= KalmanFilter()
	kalman_filter_executor.start_perception()

if __name__ == '__main__':
	main()