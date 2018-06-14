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


def classify():

	adaptative = False
	pr_b = False
	precision = 0.7

	# Can free memory?
	pub = rospy.Publisher("finished", String, queue_size=10)

	# Inicializamos la escena de MoveIt
	p = PlanningSceneInterface("base")
	#group = moveit_commander.MoveGroupCommander("Baxter")
	#group.set_planner_id("RRTConnectkConfigDefault")
	arms_group = MoveGroupInterface("both_arms", "base")
	rightarm_group = MoveGroupInterface("right_arm", "base")
	leftarm_group = MoveGroupInterface("left_arm", "base")
	 
	# Creamos la instancia del brazo derecho
	right_arm = baxter_interface.limb.Limb('right')
	 
	# Creamos la instancia del gripper derecho
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

	# Inicializamos lista de objetos
	objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
	p.clear()
	p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
	p.waitForSync()
	# Move both arms to start state.

	# Posicion inicial de Baxter
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

	#rpos.pose = rotate_pose_msg_by_euler_angles(rpos.pose, 0.0, 0.0, 0.5)
	rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	leftarm_group.moveToPose(lpos, "left_gripper", max_velocity_scaling_factor=1, plan_only=False)

	#time.sleep(1)

	# Medimos el tiempo que invierte en realizar la tarea

	pr = cProfile.Profile()
	pr.enable()
	init = time.time()

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

		# Parametro que permita coger mas o menos objetos verdes y otro para robotica adaptativa
		if (pr_b and not adaptative):
			if precision >= 1:
				punto_medio.pose.position.x += 0.01
		
			else:
				punto_medio.pose.position.x -= alfa * (1 - precision)

		elif adaptative:

			print("Si es la primera ejecucion, escribe 1.0")
			precision_value = input()

			if precision_value != 1.0:
				punto_medio.pose.position.x += alfa * (1 - precision_value)
		
		#print("pm: "+ str(punto_medio.pose.orientation.x))

		orient = (-1) * (1/punto_medio.pose.orientation.x)
		print(orient)
		print(punto_medio.pose.orientation.x)
		print("**************************************************************************")

		#if orient > 45:
		#	orient = -1*(orient % 45)

		punto_medio.pose = rotate_pose_msg_by_euler_angles(punto_medio.pose, 0.0, 0.0, orient)#1.57)
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

		fin = time.time()
		mytime = fin - init
		pr.disable()
		sortby = 'cumulative'
		print("Deshabilitando profiler")
		ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)
		
		freemem = "free"
		pub.publish(freemem)

		with open('/home/buxter/ros_ws/src/classif/results-big.txt', 'a') as file:
			file.write(str(mytime))
			file.write("\n")
		file.close()

	else:
		sys.exit("No se encuentran los puntos")

if __name__=='__main__':
	try:
		rospy.init_node('clsf', anonymous=True)
		classify()
	except rospy.ROSInterruptException:
		pass
