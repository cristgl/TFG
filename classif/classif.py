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


def clasificar():

	adaptative = True
	pr_b = False
	precision = 0.7

	# Can free memory?
	pub = rospy.Publisher("finished", String, queue_size=10)

	# Initialize MoveIt! scene
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

	# Initial pos
	rpos = PoseStamped()
	rpos.header.frame_id = "base"
	rpos.header.stamp = rospy.Time.now()
	rpos.pose.position.x = 0.555
	rpos.pose.position.y = 0.0
	rpos.pose.position.z = 0.206
	rpos.pose.orientation.x = 1.0
	rpos.pose.orientation.y = 0.0
	rpos.pose.orientation.z = 0.0
	rpos.pose.orientation.w = 0.0

   	lpos = PoseStamped()
	lpos.header.frame_id = "base"
	lpos.header.stamp = rospy.Time.now()
	lpos.pose.position.x = 0.65
	lpos.pose.position.y = 0.6
	lpos.pose.position.z = 0.206
	lpos.pose.orientation.x = 1.0
	lpos.pose.orientation.y = 0.0
	lpos.pose.orientation.z = 0.0
	lpos.pose.orientation.w = 0.0


	while True:
		
		# Move to initial position
		rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
		leftarm_group.moveToPose(lpos, "left_gripper", max_velocity_scaling_factor=1, plan_only=False)

		# Get the middle point
		locs = PoseArray()
		locs = rospy.wait_for_message("clasificacion", PoseArray)

		if(len(locs.poses)!=0):
		   	
		   	punto_medio = PoseStamped()
			punto_medio.header.frame_id = "base"
			punto_medio.header.stamp = rospy.Time.now()
			punto_medio.pose.position.x = locs.poses[0].position.x
			punto_medio.pose.position.y = locs.poses[0].position.y
			punto_medio.pose.position.z = locs.poses[0].position.z
			punto_medio.pose.orientation.x = locs.poses[0].orientation.x
			punto_medio.pose.orientation.y = locs.poses[0].orientation.y
			punto_medio.pose.orientation.z = locs.poses[0].orientation.z
			punto_medio.pose.orientation.w = locs.poses[0].orientation.w
			
			alfa = 0.1

			# Two parameters: 
			# When pr_b == 1, the program will try to adapt the trajectory for getting the precision established in the parameter of the same name
			# When adaptative == 1, the program will try to get the best precision based on the precision of the last execution.

			if (pr_b and not adaptative):
				if precision >= 1:
					punto_medio.pose.position.x += 0.01
			
				else:
					punto_medio.pose.position.x -= alfa * (1 - precision)

			else:

				print("If it is the first time executing, write -1")
				precision_value = input()

				if precision_value != -1.0:
					punto_medio.pose.position.x += alfa * (1 - precision_value)
				elif precision_value == 1.0:
					adaptative = False
			
			# Get the normal orientation for separating the objects
			orient = (-1) * (1/punto_medio.pose.orientation.x)

			punto_medio.pose = rotate_pose_msg_by_euler_angles(punto_medio.pose, 0.0, 0.0, orient)
			rightarm_group.moveToPose(punto_medio, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

			orient = 1.57 - orient 
			punto_medio.pose = rotate_pose_msg_by_euler_angles(punto_medio.pose, 0.0, 0.0, orient)
			rightarm_group.moveToPose(punto_medio, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

			punto_medio.pose.position.x += 0.15
			rightarm_group.moveToPose(punto_medio, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
			time.sleep(1)
			punto_medio.pose.position.x -= 0.3
			rightarm_group.moveToPose(punto_medio, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
			time.sleep(1)

			rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

			# Free the memory when finished
			freemem = "free"
			if not adaptative:
				pub.publish(freemem)

		else:
			sys.exit("Cannot identify any object")

if __name__=='__main__':
	try:
		rospy.init_node('clsf', anonymous=True)
		clasificar()
	except rospy.ROSInterruptException:
		pass

# Copyright (C) 2018  Cristina Garrido

# This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
