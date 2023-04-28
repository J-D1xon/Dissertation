#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import open_manipulator_msgs.msg

from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *


robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")

INIT_POSE = [ 0.000,  0.000,  0.000,  0.000]
TUCK_POSE = [ 0.000, -1.700,  1.200,  0.700]
WAVE_POSE = [ 0.000,  0.250, -0.250, -1.450]



def shutdown_callback():
    arm_group.go(INIT_POSE, wait=True)
    arm_group.go(TUCK_POSE, wait=True)
    arm_group.stop()
    print("Shutting down...")

def init():
    arm_group.go(INIT_POSE, wait=True)
    arm_group.stop()
    WAVE_POSE[0] = 0.4
    main()

def main():
    wave_speed = -1

    while not rospy.is_shutdown():

        arm_group.go(WAVE_POSE)
        arm_group.stop()
        WAVE_POSE[0] *= -1

        rospy.sleep(0.1)
    rospy.spin()




if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("wave", anonymous=True)
    rospy.on_shutdown(shutdown_callback)
    rospy.Rate(10) #Hz
    init()