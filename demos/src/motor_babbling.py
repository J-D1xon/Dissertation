#!/usr/bin/env python3

##### IMPORTS #####
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import open_manipulator_msgs.msg

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

# used to visualise path with rviz
display_trajectory_publisher = rospy.Publisher(
    "arm_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size = 20
)

def shutDownCallback():
    arm_group.go(INIT_POSE, wait=True)
    arm_group.go(TUCK_POSE, wait=True)
    arm_group.stop()
    print("Shutting Down...")

def init():
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("babble", anonymous=True)
    rospy.on_shutdown(shutDownCallback)

    
    arm_group.go(INIT_POSE, wait=True)
    arm_group.stop()

    # closes the gripper
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    gripper_group.go([-0.01, -0.01], wait=True)
    gripper_group.stop()

def main():
    
    init()
    rate = rospy.Rate(10) #Hz

    thetaZ = 2.000 #radians in each direction from centre
    
    stepAngle = 0.100 # radians step taken for each sweep

    counter = 0

    while not rospy.is_shutdown():

        angleToVertical = (1.5 - stepAngle*counter) # radians of the boom from vertical

        if(angleToVertical > 0):

            arm_group.go([thetaZ,angleToVertical,-0.9,0],wait=True)
            arm_group.stop()

            thetaZ = -thetaZ

            counter += 1
        else:
            print("Motor Babble Complete")
        rate.sleep()

    rospy.spin()

if __name__=="__main__":
    main()