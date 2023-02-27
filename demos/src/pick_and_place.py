#!/usr/bin/python3

##### IMPORTS #####
import sys
import copy
import rospy
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

group_name = "arm"

move_group = moveit_commander.MoveGroupCommander(group_name)

joint_goal = move_group.get_current_joint_values()

##### SET POSE SETUP #####

initPose = [0.000, 0.000,  0.000,  0.000]
pickPose = [0.000, 1.550, -0.300, -1.250]
movePose = [0.000, 1.000, -0.300, -1.000]

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size = 20
)

# function that sends a joint goal to the move group and executes a move
def moveToSetPose(pose):
    print

    # Move arm to a given pose
    for i in range(len(pose)):
        joint_goal[i] = pose[i]

    move_group.go(joint_goal, wait=True)
    move_group.stop()

def pickObject():
    print("Picking up object")
    # TODO: close gripper

    moveToSetPose(movePose)

def placeObject():
    print("Placing Object")
    moveToSetPose(pickPose)
    # TODO: open gripper
    control_gripper(-0.01)

def control_gripper(joint_value):
    rospy.wait_for_service("/open_manipulator/goal_tool_control")
    set_joint_position = rospy.ServiceProxy("/open_manipulator/goal_tool_control", SetJointPosition)
    msg = open_manipulator_msgs.msg.JointPosition()
    msg.joint_name.append("gripper")
    msg.position.append(joint_value)
    resp1 = set_joint_position("arm", msg, 5)
    rospy.sleep(1.5)

def shutDownCallback():
    print("\nMoving to initial position")
    moveToSetPose(initPose)
    print("shutting down...")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_and_place", anonymous=True)
    rospy.on_shutdown(shutDownCallback)
    counter = 0

    # set initial state to the init pose
    moveToSetPose(initPose)

    placeObject()

    ### main loop ###
    while not rospy.is_shutdown():

        if counter % 100 == 0:
            print("TODO")
        rospy.sleep(0.01)

    rospy.spin()

if __name__ == "__main__":
    main()