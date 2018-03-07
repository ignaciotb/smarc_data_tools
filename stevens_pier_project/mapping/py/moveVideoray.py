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
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, TransformStamped, Transform
from std_msgs.msg import Bool, String
from nav_msgs.msg import Path
from numpy import mean, std
import sys
import numpy as np
import random
from tf.transformations import euler_from_quaternion, quaternion_matrix
import kalman_filter
import md_filter


# Callback that recieves iformation that a new position transformation is available
# based on the sonar data and ICP.
def sonar_move_callback(tmp):
	# print "SONAR"
	if sonar_move_callback.start:
		sonar_move_callback.RPos = np.eye(4)*25
		sonar_move_callback.RPose = np.eye(6)*25
		sonar_move_callback.start = 0

	# get the position the sonar scan estimates
	# try:
	# 	(trans,rot) = listener.lookupTransform('/odom_sonar', '/odom' , rospy.Time(0))
	# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 	print "returning"
	# 	return

	# print "listen: ", trans, rot
	# print "geomet: ", (tmp.translation.x,tmp.translation.y,tmp.translation.z), (tmp.rotation.x, tmp.rotation.y, tmp.rotation.z, tmp.rotation.w)
	# print rot, trans

	# Create objects for orientation and position updates
	pos = PoseStamped()
	pos.pose.position = tmp.translation
	pos.pose.orientation = tmp.rotation
	pose_move(pos.pose, sonar_move_callback.RPose, True)
	pos_move(pos, sonar_move_callback.RPos, True)


# Callback that recieves pose information from gyro and depth sensor
def pose_move_callback(pos):
	if pose_move_callback.start:
		pose_move_callback.R = np.eye(6)*25
		pose_move_callback.start = 0
	pose_move(pos, pose_move_callback.R, False)

# Callback that recieves position (x,y) from the usbl positioning system
def usbl_move_callback(pos):
	# print "USBL"
	if usbl_move_callback.start:
		usbl_move_callback.R = np.eye(4)*5
		usbl_move_callback.start = 0
		usbl_move_callback.md_filter = md_filter.md_filter(2, [2.0, 1.9], 10, [0, 1])
	# if usbl_move_callback.md_filter.update( [pos.pose.position.x,pos.pose.position.y] ):
	pos_move(pos, usbl_move_callback.R, False)


def pos_move(pos, R, correction):
	# Note pos and current position have x and y swapped.
	# -> pos.position.x corresponds to current.position.y

	# Check that the values aren't crazy
	if correction:
		max = 5
		# The pos is in world coordinates need to translate to kalman filter coordinates (x->y, y->x)
		a = quaternion_matrix([pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w])
		a[0,3] = pos.pose.position.x
		a[1,3] = pos.pose.position.y
		b = np.array([current.position.x, current.position.y, 0, 1])
		c = a.dot(b)
		pos.pose.position.x = c[1]
		pos.pose.position.y = c[0]
	else:
		max = 10

	if(abs(pos.pose.position.x - current.position.y) > max or abs(pos.pose.position.y - current.position.x) > max ):
		print "CRAZY POSITION"
		return


	broadcaster = tf.TransformBroadcaster()
	if pos_move.start:
		dt = 0.0

		loc = np.matrix([[0],#v_x
						 [0],#v_y
						 [pos.pose.position.x],#x
						 [pos.pose.position.y]])#y

		A = np.matrix([[1,  0,  0,  0,],
				   	   [0,  1,  0,  0,],
				       [dt, 0,  1, 0,],
				       [0,  dt, 0,  1,]])
		B = np.matrix([0])
		C = np.eye(loc.shape[0])
		Q = np.eye(loc.shape[0])*0.5
		# R = np.eye(loc.shape[0])*5000
		P = np.eye(loc.shape[0])
		U = np.matrix( [[0]])
		Z = np.matrix( [[0],[0],[0],[0]])

		pos_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,loc)
		pos_move.md_filter = md_filter.md_filter(2, [1.75, 1.6], 10, [0, 1])
		pos_move.path = Path()
		pos_move.path.header.frame_id="odom"
		pos_move.start = 0


	if not correction:
		pos.pose.position.x += pos_move.xd
		pos.pose.position.y += pos_move.yd



	# if pos_move.md_filter.update( [pos.pose.position.x,pos.pose.position.y] ) or correction or not correction:
	Z = np.matrix( [[0],[0],[pos.pose.position.x],[pos.pose.position.y] ])
	U = np.matrix( [[0]])
	pos_move.kalman.move(U,Z, R)
	kalman_pos = pos_move.kalman.getState()
	if correction:
		# We are subscribing to the inverse translation, thus the minus sign.
		pos_move.xd += kalman_pos.item(2) - current.position.y
		pos_move.yd += kalman_pos.item(3) - current.position.x
	# print "addition: ", pose_move.yaw_addition
	# print "pos:", "x:", current.position.x, "y:", current.position.y
	# print "mx: ", pos_move.xd
	# print "my: ", pos_move.yd
	# print "yaw:", pose_move.yaw
	current.position.y = kalman_pos.item(2)
	current.position.x = kalman_pos.item(3)

	# else:
	# 	print "NOT ACCEPTED"

	path_now = PoseStamped()
	path_now.pose.position.x = current.position.x
	path_now.pose.position.y = current.position.y
	path_now.pose.position.z = 0
	# from odom (parent) to body (child) 
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z),
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									rospy.Time.now(), "body", "odom" )
	# from buffer (parent) to odom (child)
	broadcaster.sendTransform((-pos_move.yd, -pos_move.xd, 0),
								tf.transformations.quaternion_from_euler(0, 0, -pose_move.yaw/180*math.pi),
								rospy.Time.now(),"odom","buffer")
	pos_move.path.poses.append(path_now)
	path.publish(pos_move.path)


	pose_kalman = PoseStamped();
	pose_kalman.pose.position.x = current.position.x 
	pose_kalman.pose.position.y = current.position.y
	pose_kalman.pose.position.z =  current.position.z
	pose_kalman.pose.orientation.x = current.orientation.x
	pose_kalman.pose.orientation.y = current.orientation.y
	pose_kalman.pose.orientation.z = current.orientation.z
	pose_kalman.pose.orientation.w = current.orientation.w
	pose_kalman.header.frame_id = "odom"
	pub_pose.publish(pose_kalman)


def pose_move(pos, R, correction):

	toDegree = 180/math.pi
	broadcaster = tf.TransformBroadcaster()
	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([-1*pos.orientation.y,
																  pos.orientation.x,
																-1*pos.orientation.z,
																   pos.orientation.w])

	roll = roll*toDegree
	pitch = pitch*toDegree
	yaw = yaw*toDegree

	if correction:
		if abs(yaw) > 45:
			print "CRAZY YAW"
			return
		roll = pose_move.kalman.getState().item(3)
		pitch = pose_move.kalman.getState().item(4)
		yaw = pose_move.kalman.getState().item(5) - yaw

	if pose_move.start:
		dt = .00
		position = np.matrix([[0],  # v_x
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
		C = np.eye(position.shape[0])
		Q = np.eye(position.shape[0]) * .5
		R = np.eye(position.shape[0]) * 50
		P = np.eye(position.shape[0])
		U = np.matrix([[0]])
		Z = np.matrix([[0], [0], [0], [0], [0], [0] ])

		pose_move.kalman = kalman_filter.kalman_filter(A,B,C,Q,P,R,position)
		pose_move.start = 0

	if not correction:
		yaw += pose_move.yaw
		current.position.z = -pos.position.z
		# current.position.z = 0

	# Z = np.matrix([[0], [0], [0], [roll], [pitch], [yaw+pose_move.yaw_addition]])
	Z = np.matrix([[0], [0], [0], [roll], [pitch], [yaw]])
	# Z = np.matrix([[0], [0], [0], [0], [0], [yaw]])
	U = np.matrix([[0]])

	tmp_yaw = pose_move.kalman.getState().item(5)

	pose_move.kalman.move(U, Z, R)
	pos = pose_move.kalman.getState()

	if correction:
		# print "yaw input:", yaw
		# print "yaw before:", tmp_yaw
		# print "yaw after:", pos[5]
		# print "yaw change:", yaw - tmp_yaw
		pose_move.yaw += pos.item(5) - tmp_yaw
		# print "yaw addition:", pose_move.yaw

	roll = pos[3]
	pitch = pos[4]
	yaw = pos[5]



	quad = tf.transformations.quaternion_from_euler(roll/toDegree, pitch/toDegree, yaw/toDegree )
	# quad = tf.transformations.quaternion_from_euler(0, 0, yaw/toDegree )

	current.orientation.x = quad[0]
	current.orientation.y = quad[1]
	current.orientation.z = quad[2]
	current.orientation.w = quad[3]

	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z),
								(current.orientation.x,current.orientation.y,current.orientation.z,
								current.orientation.w),
								rospy.Time.now(), "body", "odom" )
	broadcaster.sendTransform((-pos_move.yd,-pos_move.xd, 0),
								tf.transformations.quaternion_from_euler(0, 0, -pose_move.yaw/180*math.pi),
								rospy.Time.now(),"odom","buffer")


def save_path(name):
	print name.data
	f = open("/home/clarisse/catkin_ws/src/filteringprocess/mapping/savedData/" + name.data + "-path.txt", 'w')
	for pose in pos_move.path.poses:
		f.write(str(pose.pose.position.x) + " " + str(pose.pose.position.y) + "\n")
	print "hmmmmmmmmm"
	f.close()

if __name__ == '__main__':
	print "moveVideoray"
	#set up the node
	rospy.init_node('moveVideoray', anonymous=True)
	#make a broadcaster foir the tf frame
	broadcaster = tf.TransformBroadcaster()
	listener = tf.TransformListener()

	#make intilial values
	current = Pose()
	current.position.x = 0
	current.position.y = 0
	current.position.z = 0
	current.orientation.x = 0
	current.orientation.y = 0
	current.orientation.z = 0
	current.orientation.w = 0
	pose_move.yaw = 0
	pos_move.xd = 0
	pos_move.yd = 0
	#send the tf frame
	broadcaster.sendTransform( (current.position.x,current.position.y,current.position.z),
								(current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w),
									 rospy.Time.now(), "body", "odom" )

	#listen for information

	pos_move.start = 1
	pose_move.start = 1
	usbl_move_callback.start = 1
	sonar_move_callback.start = 1
	pose_move_callback.start = 1
	pose_move.yaw_addition = 0.0
	pub_pose = rospy.Publisher("/pose_after_kalman", PoseStamped, queue_size=1)
	pub = rospy.Publisher('update_buffer', Bool, queue_size=1)
	path = rospy.Publisher('path', Path, queue_size=1)
	rospy.Subscriber("/usbl_pose", PoseStamped, usbl_move_callback);
	rospy.Subscriber("/mapBuilder/trans_update", Transform, sonar_move_callback);
	rospy.Subscriber("/pose_only", Pose, pose_move_callback);
	rospy.Subscriber("/writeMapToTXT", String, save_path);
	rospy.spin()
	print "moveVideoray"
