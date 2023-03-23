#!/usr/bin/env python3

##### IMPORTS #####
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import open_manipulator_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *

##### MOVEIT! SETUP #####

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")

##### POSES #####

INIT_POSE = [ 0.000,  0.000,  0.000,  0.000]
TUCK_POSE = [ 0.000, -1.700,  1.200,  0.700]

axisBounds = [[0,0],[0,0],[0,0],[0,0]]

def shutDownCallback():
    arm_group.go(INIT_POSE, wait=True)
    arm_group.go(TUCK_POSE, wait=True)
    arm_group.stop()
    print("Shutting Down...")

def main():
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_and_place", anonymous=True)
    rospy.on_shutdown(shutDownCallback)

    rate = rospy.Rate(100) #Hz

    arm_group.go(INIT_POSE, wait=True)
    arm_group.stop()

    counter = 0

    while not rospy.is_shutdown():
        joint_angles = arm_group.get_current_joint_values()
        for i in range(len(joint_angles)):
            joint_angles[i] += random.uniform(-0.2,0.2)

        if counter % 2000 == 0:
            joint_angles[random.randint(0, 3)] += random.uniform(-1,1)

        arm_group.go(joint_angles, wait=True)
        arm_group.stop()

        counter += 1

    rospy.spin()

if __name__=="__main__":
    main()