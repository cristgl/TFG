#!/usr/bin/env python

import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import baxter_interface
import moveit_commander
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from std_msgs.msg import String


def dyn_wedge():

	pub = rospy.Publisher("finished", String, queue_size=10)

	# Initialize MoveIt scene
	p = PlanningSceneInterface("base")
	arms_group = MoveGroupInterface("both_arms", "base")
	rightarm_group = MoveGroupInterface("right_arm", "base")
	leftarm_group = MoveGroupInterface("left_arm", "base")
	
	# Create right arm instance
	right_arm = baxter_interface.limb.Limb('right')
	
	# Create right gripper instance
	right_gripper = baxter_interface.Gripper('right')
	right_gripper.calibrate()
	right_gripper.open()
	
	right_gripper.close()

	offset_zero_point=0.903
	table_size_x = 0.714655654394
	table_size_y = 1.05043717328
	table_size_z = 0.729766045265
	center_x = 0.457327827197
	center_y = 0.145765166941
	center_z = -0.538116977368	 
	table_distance_from_gripper = -offset_zero_point+table_size_z+0.0275/2
	j=0
	k=0

	# Initialize object list
	objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
	p.clear()
	p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
	p.waitForSync()
	# Move both arms to start state.

	# Initial pose
	rpos = PoseStamped()
	rpos.header.frame_id = "base"
	rpos.header.stamp = rospy.Time.now()
	rpos.pose.position.x = 0.555
	rpos.pose.position.y = 0.0
	rpos.pose.position.z = 0.206 #0.18
	rpos.pose.orientation.x = 1.0
	rpos.pose.orientation.y = 0.0
	rpos.pose.orientation.z = 0.0
	rpos.pose.orientation.w = 0.0

   	lpos = PoseStamped()
	lpos.header.frame_id = "base"
	lpos.header.stamp = rospy.Time.now()
	lpos.pose.position.x = 0.555
	lpos.pose.position.y = 0.65
	lpos.pose.position.z = 0.206#0.35
	lpos.pose.orientation.x = 1.0
	lpos.pose.orientation.y = 0.0
	lpos.pose.orientation.z = 0.0
	lpos.pose.orientation.w = 0.0

	rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	leftarm_group.moveToPose(lpos, "left_gripper", max_velocity_scaling_factor=1, plan_only=False)

	# Get the middle point between the two centroids
	locs = rospy.wait_for_message("clasificacion", PoseArray)

	if(len(locs.poses)!=0):

		punto_medio1 = PoseStamped()
		punto_medio1.header.frame_id = "base"
		punto_medio1.header.stamp = rospy.Time.now()
		punto_medio1.pose.position.x = locs.poses[0].position.x
		punto_medio1.pose.position.y = locs.poses[0].position.y
		punto_medio1.pose.position.z = -0.08
		punto_medio1.pose.orientation.x = locs.poses[0].orientation.x
		punto_medio1.pose.orientation.y = locs.poses[0].orientation.y
		punto_medio1.pose.orientation.z = locs.poses[0].orientation.z
		punto_medio1.pose.orientation.w = locs.poses[0].orientation.w

	else:
		sys.exit("No se encuentran los puntos")

	# Get the middle point again after a few seconds
	tiempo = 3
	time.sleep(tiempo)
	locs = rospy.wait_for_message("clasificacion", PoseArray)

	if(len(locs.poses)!=0):

		punto_medio2 = PoseStamped()
		punto_medio2.header.frame_id = "base"
		punto_medio2.header.stamp = rospy.Time.now()
		punto_medio2.pose.position.x = locs.poses[0].position.x
		punto_medio2.pose.position.y = locs.poses[0].position.y
		punto_medio2.pose.position.z = -0.08
		punto_medio2.pose.orientation.x = locs.poses[0].orientation.x
		punto_medio2.pose.orientation.y = locs.poses[0].orientation.y
		punto_medio2.pose.orientation.z = locs.poses[0].orientation.z
		punto_medio2.pose.orientation.w = locs.poses[0].orientation.w

	else:
		sys.exit("No se encuentran los puntos")

	# Calculate speed of objects
	vel = (punto_medio2.pose.position.y - punto_medio1.pose.position.y) / tiempo

	# Predict position within a certain time
	nuevo_tiempo = 2.1
	posicion = nuevo_tiempo * vel * 2

	if(posicion>=0):
		nueva_y = punto_medio2.pose.position.y - posicion
	else:
		nueva_y = punto_medio2.pose.position.y + posicion

	orientacion = (-1) * 1/punto_medio2.pose.orientation.x

	new_or = 0.26

	if orientacion > 0:
		new_or = -0.26

	# If there is time enough to sweep the objects
	if vel > -0.08:
		start = time.time()
		punto_medio2.pose.position.y = nueva_y - nueva_y/2
		punto_medio2.pose.position.x = punto_medio1.pose.position.x - 0.03
		punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, orientacion - new_or)
		rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
		end = time.time()

		tiempo = end - start
		while(nuevo_tiempo - tiempo >= 1):
			end = time.time()
			nuevo_tiempo = end - start
	else:
		punto_medio2.pose.position.y = nueva_y

	print(punto_medio2.pose.position.y)

	punto_medio2.pose.position.x = punto_medio1.pose.position.x
	punto_medio2.pose.position.y += 0.05 
	punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, orientacion)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	obj = punto_medio2.pose.position.y + 0.16
	neg = 1

	while punto_medio2.pose.position.y < obj:
		punto_medio2.pose.position.y += 0.03
		# Shake arm
		punto_medio2.pose.position.x += neg * 0.09
		rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
		neg = (-1)*neg

	time.sleep(2)

	rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	freemem = "free"
	pub.publish(freemem)


if __name__=='__main__':
	try:
		rospy.init_node('wdg', anonymous=True)
		dyn_wedge()
	except rospy.ROSInterruptException:
		pass
