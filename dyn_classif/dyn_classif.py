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


def dyn_cl():

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

	#rpos.pose = rotate_pose_msg_by_euler_angles(rpos.pose, 0.0, 0.0, 0.5)
	rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	leftarm_group.moveToPose(lpos, "left_gripper", max_velocity_scaling_factor=1, plan_only=False)

	#time.sleep(1)
	# Medimos el tiempo que invierte en realizar la tarea
	pr = cProfile.Profile()
	pr.enable()
	inicio = time.time()
	locs = PoseArray()
	locs = rospy.wait_for_message("clasificacion", PoseArray)

	if(len(locs.poses)!=0):

		punto_medio1 = PoseStamped()
		punto_medio1.header.frame_id = "base"
		punto_medio1.header.stamp = rospy.Time.now()
		punto_medio1.pose.position.x = locs.poses[0].position.x
		punto_medio1.pose.position.y = locs.poses[0].position.y
		punto_medio1.pose.position.z = -0.08#locs.poses[1].position.z
		punto_medio1.pose.orientation.x = locs.poses[0].orientation.x
		punto_medio1.pose.orientation.y = locs.poses[0].orientation.y
		punto_medio1.pose.orientation.z = locs.poses[0].orientation.z
		punto_medio1.pose.orientation.w = locs.poses[0].orientation.w
		#print("pB1: ", punto_medio1)

	else:
		sys.exit("No se encuentran los puntos")

	# Obtenemos por segunda vez la posicion del punto medio tras un tiempo
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
		#print("pB2: ", punto_medio2)

	else:
		sys.exit("No se encuentran los puntos")

	# Calculamos la velocidad
	
	print("P2 ",punto_medio2.pose.position.y)
	print("P1 ", punto_medio1.pose.position.y)
	vel = (punto_medio2.pose.position.y - punto_medio1.pose.position.y) / tiempo

	# Calculamos en que punto estara un tiempo mas tarde
	nuevo_tiempo = 2
	posicion = nuevo_tiempo*vel

	nueva_y = 0

	if(posicion>=0):
		nueva_y = punto_medio2.pose.position.y - posicion
	else:
		nueva_y = punto_medio2.pose.position.y + posicion

	print("Nueva y ", nueva_y)

	start = time.time()
	# if(nueva_y >= 0):
	#  	punto_medio2.pose.position.y = (-1) * nueva_y
	# else:
	#  	punto_medio2.pose.position.y = nueva_y

	punto_medio2.pose.position.y = nueva_y

	print("pm2 y ", punto_medio2.pose.position.y)

	punto_medio2.pose.position.x = punto_medio1.pose.position.x # Puede que el centro cambie y de una x distinta
	punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, -0.3927)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	fin = time.time()
	# Esperar hasta que lleguemos al nuevo_tiempo
	#while((fin - start) <= (nuevo_tiempo-2)):
	#	fin = time.time()

	# punto_medio2.pose.position.x -= 0.15
	# punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 0.7854)
	# rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	# punto_medio2.pose.position.x += 0.3
	# punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, -0.7854)
	# rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	punto_medio2.pose.position.x -= 0.15
	punto_medio2.pose.position.y += 0.1
	punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, -0.7854)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	end = time.time()

	tpo = (end - start) * 2
	posicion = tpo*vel
	nueva_y = 0

	if(posicion>=0):
		nueva_y = punto_medio2.pose.position.y - posicion
	else:
		nueva_y = punto_medio2.pose.position.y + posicion

	if(nueva_y >= 0):
		punto_medio2.pose.position.y = (-1) * nueva_y
	else:
		punto_medio2.pose.position.y = nueva_y

	
	punto_medio2.pose.position.x += 0.15
	punto_medio2.pose.position.y -= 0.35 # nueva_y
	punto_medio2.pose.position.z += 0.03
	punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 0.7854)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	punto_medio2.pose.position.z -= 0.03
	#punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 1)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)
	#time.sleep(2.5)

	punto_medio2.pose.position.x += 0.15
	punto_medio2.pose.position.y += 0.3
	punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 0.78)
	rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

###############
	# Calculamos la nueva posicion en un tiempo determinado
	# ultimo_tiempo = 5
	# ultima_posicion = ultimo_tiempo*vel

	# if(vel>=0):
	# 	nueva_y = punto_medio2.pose.position.y - ultima_posicion
	# else:
	# 	nueva_y = punto_medio2.pose.position.y + ultima_posicion
	# print("---------------------------------anterior posicion: ", punto_medio2.pose.position.y)
	# print("---------------------------------------------nueva posicion: ", nueva_y)

	# punto_medio2.pose.position.y = nueva_y
	# punto_medio2.pose.position.x = punto_medio1.pose.position.x # Puede que el centro cambie y de una x distinta
	# punto_medio2.pose.position.z -= 0.05
	# punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 0.7854)
	# rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	# punto_medio2.pose.position.x += 0.15
	# punto_medio2.pose.position.y += 0.1
	# punto_medio2.pose = rotate_pose_msg_by_euler_angles(punto_medio2.pose, 0.0, 0.0, 1.57)
	# rightarm_group.moveToPose(punto_medio2, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	fin = time.time()
	rightarm_group.moveToPose(rpos, "right_gripper", max_velocity_scaling_factor=1, plan_only=False)

	freemem = "free"
	pub.publish(freemem)

	pr.disable()
	sortby = 'cumulative'
	print("velocidad: ", vel)
	ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)
	mytime = fin - inicio
	file = open("results.txt", "w")
	file.write(str(vel) + ' ' + str(mytime))
	file.close()

if __name__=='__main__':
	try:
		rospy.init_node('dcl', anonymous=True)
		dyn_cl()
	except rospy.ROSInterruptException:
		pass
