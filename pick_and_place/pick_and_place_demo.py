#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

# pick and place
class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb
        self._hover_distance = hover_distance
        self._verbose = verbose
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
      
	# Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():

    rospy.init_node("pick_and_place_demo")

    limb = 'right'
    hover_distance = 0.15 

    starting_joint_angles = {#'left_w0': 0.6699952259595108,
                             #'left_w1': 1.030009435085784,
                             #'left_w2': -0.4999997247485215,
                             #'left_e0': -1.189968899785275,
                             #'left_e1': 1.9400238130755056,
                             #'left_s0': -0.08000397926829805,
                             #'left_s1': -0.9999781166910306,
                             'right_e0': 1.7238109084167481, 
                             'right_e1': 1.7169079948791506, 
                             'right_s0': 0.36930587426147465, 
                             'right_s1': -0.33249033539428713, 
                             'right_w0': -1.2160632682067871, 
                             'right_w1': 1.668587600115967, 
                             'right_w2': -1.810097327636719}
    pnp = PickAndPlace(limb, hover_distance)
   

	# Get the detected objects of the camera
    temp = rospy.wait_for_message("detected_objects", PoseArray)
    locs = temp.poses
    block_poses = list()

	# Get position and orientation
    for i in range(len(locs)):
        block_poses.append(Pose(position=Point(locs[i].position.x,locs[i].position.y,-0.8), orientation=Quaternion(locs[i].orientation.x,locs[i].orientation.y,locs[i].orientation.z, locs[i].orientation.w)))
        block_poses.append(Pose(position=Point(locs[i].position.x,locs[i].position.y+1,-0.53), orientation=Quaternion(locs[i].orientation.x,locs[i].orientation.y,locs[i].orientation.z, locs[i].orientation.w)))
   
    pnp.move_to_start(starting_joint_angles)
    idx = 0

	# are there objects on the table
    if len(block_poses) != 0:
        while not rospy.is_shutdown():
            pnp.pick(block_poses[idx])
            idx = (idx+1) % len(block_poses)
            pnp.place(block_poses[idx])
    else:
        rospy.logerr("Block_poses length error")
        return False
    return 0

if __name__ == '__main__':
    sys.exit(main())
