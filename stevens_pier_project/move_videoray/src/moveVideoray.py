#!/usr/bin/env python

'''
	Nathaniel Goldfarb 
	8/16/15
	I&E scholar
	VideoRay research group
	
	This program moves the videoray model in rviz using 
	data from the /usbl_pose node
	based on "Using urdf with robot_state_publisher" tutorial
'''




import rospy
import roslib
import math
import tf
#import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, TransformStamped
from numpy import mean, std
import sys
import numpy as np
import random
from tf.transformations import euler_from_quaternion
import kalman_filter
import md_filter
import numpy as np


#//move the videoray using the data from the /pose_only node
def usbl_move(pos):

	broadcaster = tf.TransformBroadcaster()
	if usbl_move.start:
		dt = 2

		loc = np.matrix([[0],#v_x
						 [0],#v_y
						 [pos.pose.position.x],#x
						 [pos.pose.position.y]])#z
	
		A = np.matrix([[1,  0,  0,  0,],
				   	   [0,  1,  0,  0,],
				       [dt, 0,  1, 0,],
				       [0,  dt, 0,  1,]])
		B = np.matrix([0])
		C = np.eye(loc.shape[0])
		Q = np.eye(loc.shape[0])*0.5
		R = np.eye(loc.shape[0])*5000
		P = np.eye(loc.shape[0])
		U = np.matrix( [[0]])
		Z = np.matrix( [[0],[0],[0],[0]])
		
		usbl_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)
		usbl_move.md_filter = md_filter.md_filter(2, [1.75, 1.6], 10, [0, 1])
		usbl_move.start = 0


	if usbl_move.md_filter.update( [pos.pose.position.x,pos.pose.position.y] ):
		#update.ax.scatter(pos.position.x,pos.position.y,-1*current.position.z,color='b')
		current.position.x = pos.pose.position.x
		current.position.y = pos.pose.position.y
		#current.position.z = pos.pose.position.z
		#update('b')
		Z = np.matrix( [[0],[0],[pos.pose.position.x],[pos.pose.position.y] ])
		U = np.matrix( [[0]])
		usbl_move.kalman.move(U,Z)
		kalman_pos = usbl_move.kalman.getState()
		current.position.y = kalman_pos[2]
		current.position.x = kalman_pos[3]
	
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									rospy.Time.now(), "body", "odom" )
	



def pose_move(pos):

	#pos.position.z is in kPa, has to be convereted to depth
	# P  = P0 + pgz ----> pos.position.z = P0 + pg*z_real
	#z_real = -(1000*pos.position.z-101.325)/9.81 
	z_real =  -pos.position.z 
	toDegree = 180/math.pi
	current.position.z = z_real
	broadcaster = tf.TransformBroadcaster()
	#set up the Kalman Filter
	#tf.transformations.quaternion_from_euler()
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([-1*pos.orientation.y,
																  pos.orientation.x,
																-1*pos.orientation.z,
																   pos.orientation.w])
	
	roll = roll*toDegree
	pitch = pitch*toDegree
	yaw = yaw*toDegree
	s = 'The value of roll is ' + repr(roll) + ', and pitch is ' + repr(pitch) + ', yaw = ' + repr(yaw)
	print s

	if pose_move.start:
		dt = .02	
		pos = np.matrix([[0],  # v_x
						 [0],  # v_y
						 [0],  # v_z
						 [roll],  # x
						 [pitch],  # y
						 [yaw]])	  # z
		
		A = np.matrix([[1, 0, 0, 0, 0, 0, ],
					   [0, 1, 0, 0, 0, 0, ],
					   [0, 0, 1, 0, 0, 0, ],
					   [dt, 0, 0, 1, 0, 0, ],
					   [0, dt, 0, 0, 1, 0, ],
					   [0, 0, dt, 0, 0, 1, ]])
		B = np.matrix([0])
		C = np.eye(pos.shape[0])
		Q = np.eye(pos.shape[0]) * .5
		R = np.eye(pos.shape[0]) * 500
		P = np.eye(pos.shape[0])
		U = np.matrix([[0]])
		Z = np.matrix([[0], [0], [0], [0], [0], [0] ])

		pose_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,pos)
		
		pose_move.start = 0


	Z = np.matrix([[0], [0], [0], [roll], [pitch], [yaw]])
	U = np.matrix([[0]])
	pose_move.kalman.move(U, Z)
	pos = pose_move.kalman.getState()
		
	roll = pos[3]
	pitch = pos[4]
	yaw = pos[5]

	# quad = tf.transformations.quaternion_from_euler(roll/toDegree, pitch/toDegree, yaw/toDegree )
	quad = tf.transformations.quaternion_from_euler(0, 0, yaw/toDegree )
	
	current.orientation.x = quad[0]
	current.orientation.y = quad[1]
	current.orientation.z = quad[2]
	current.orientation.w = quad[3]

	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,
								current.orientation.w),
								rospy.Time.now(), "body", "odom" )



if __name__ == '__main__':
	#set up the node
	rospy.init_node('moveVideoray', anonymous=True)
	#make a broadcaster foir the tf frame
	broadcaster = tf.TransformBroadcaster()
	#make intilial values
	current = Pose()
	current.position.x = 0
	current.position.y = 0
	current.position.z = 0
	current.orientation.x = 0
	current.orientation.y = 0
	current.orientation.z = 0
	current.orientation.w = 0
	#send the tf frame
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z), 
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom" )

	#listen for information
	
	usbl_move.start = 1
	pose_move.start = 1
	#pub = rospy.Publisher("newPose", Pose)
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move);
	rospy.Subscriber("/pose_only", Pose, pose_move);
	rospy.spin()