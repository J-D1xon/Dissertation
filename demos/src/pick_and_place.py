#!/usr/bin/python3

##### IMPORTS #####
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

arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

joint_goal = arm_group.get_current_joint_values()

# Threshold variable - used to reduce noise in joint angle readings
eps = 0.005

##### POSE SETUP #####

initPose = [0.000, 0.000,  0.000,  0.000]
pickPose = [0.000, 1.550, -0.300, -1.250]
placePoseL = [0.800, 1.550, -0.500, -1.050]
placePoseR = [-0.800, 1.550, -0.500, -1.050]
movePose = [0.000, 1.000, -0.300, -1.000]

# used to visualise pahs with rviz
display_trajectory_publisher = rospy.Publisher(
    "arm_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size = 20
)

# method that sends a joint goal to the move_group and executes a move if not already at the goal
def moveToPose(pose):
    # store the current joint angles
    joint_angles = arm_group.get_current_joint_values() 

    # used to keep track of joints that don't need to move
    sameJoints = 0

    for i in range(len(joint_angles)):
        # recognises a joint is already at the pose angle if its within a threshold 'eps' of the target pose (0.005 rad)
        if joint_angles[i]-eps < pose[i] and joint_angles[i]+eps > pose[i]:
            sameJoints += 1

    # if any of the joints require movement then plan a move, else do nothing
    if sameJoints < len(joint_angles):
        print("moving to new pose...")

        arm_group.go(pose, wait=True)
        arm_group.stop()
    else:
        print("already at the given pose")

# method to move the arm to pickPose and close the gripper
def pickObject():
    print("Picking up object")

    closeGripper()

    moveToPose(movePose)

# method to move the arm to placePoseL  and open the gripper
def placeObject(direction):

    if direction == "left":
        print("Placing object on the " + direction)
        moveToPose(placePoseL)

    elif direction == "right":
        print("Placing object on the " + direction)
        moveToPose(placePoseR)

    else:
        print(direction + " invalid. Must give direction: 'left'/'right'")
        return

    openGripper()

# method to move the arm to movePose - default state
def resetArm():
    moveToPose(movePose)

# method to move the arm to pickPose - resets the arm ready to grasp an object
def positionArm():
    moveToPose(pickPose)

# method to move the gripper arms to the closed position
def closeGripper():
    gripper_group.go([-0.01, -0.01], wait=True)
    gripper_group.stop()

# method to move the gripper arms to the open position
def openGripper():
    gripper_group.go([0.01, 0.01], wait=True)
    gripper_group.stop()

# shutdown callback returns the arm to the picking pose before ending
# so that the arm is as close to the ground as possible, preventing damage to the robot or the arm 
# should the arm experience a loss of torque and fall.
def shutDownCallback():
    positionArm()
    print("shutting down...")

def main():
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pick_and_place", anonymous=True)
    rospy.on_shutdown(shutDownCallback)

    # defines the rate the loop will execute
    rate = rospy.Rate(100) #Hz
    
    # used to keep track of the number of loops
    counter = 0

    # set initial state to movePose
    resetArm()

    ### main loop ###
    while not rospy.is_shutdown():

        if counter % 1000 == 0:
            positionArm()

        if counter % 1000 == 200:
            pickObject()

        if counter % 1000 == 400:
            if counter % 2000 < 1000:
                placeObject("left")
            else:
                placeObject("right")
        
        if counter % 1000 == 600:
            resetArm()
        
        counter += 1

        rate.sleep()

    # prevents the python file from ending once this point is reached 
    #(redundant since its below an infinite loop but still good practise to include)
    rospy.spin()

if __name__ == "__main__":
    main()