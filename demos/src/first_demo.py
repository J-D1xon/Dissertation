#!/usr/bin/python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("arm")

# =======================================================
# GETTING BASIC INFORMATION
# =======================================================

# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print("============ End effector link: %s" % eef_link)

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print(f"============ Available Planning Groups: {group_names}", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print("============ Printing robot state")
# print(robot.get_current_state())
# print("")

# =======================================================
# PLANNING A JOINT GOAL
# =======================================================
# We get the joint values from the group and change some of the values:
# joint_goal = move_group.get_current_joint_values()
# joint_goal[0] = 0
# joint_goal[1] = -tau / 8
# joint_goal[2] = 0
# joint_goal[3] = -tau / 4
# joint_goal[4] = 0
# joint_goal[5] = tau / 6  # 1/6 of a turn
# joint_goal[6] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
# move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
# move_group.stop()

# =======================================================
# PLANNING A POSE GOAL
# =======================================================
def moveArm(x,y,z):
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 1.0
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    move_group.set_pose_target(pose_goal)

    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets() 

def moveHome():
    moveArm(0.046, 0.0, 0.345)
    # rospy.sleep(2)

def moveInit():
    moveArm(0.194,0.0,0.304)

def shutDownCallback():
    moveArm(0.046, 0.0, 0.2)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("first_demo", anonymous=True)
    rospy.on_shutdown(shutDownCallback)

    while not rospy.is_shutdown():
        moveHome()
        rospy.sleep(0.5)
        moveInit()
        rospy.sleep(0.5)
    
    rospy.spin()



if __name__ == '__main__':
    main()