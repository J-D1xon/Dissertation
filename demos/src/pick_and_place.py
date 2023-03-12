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

# Puts the robot into joint space or task space planning 
# (must be set to False if working with a differently configured robot)
JOINT_PLANNING = True

##### JOINT SPACE POSE SETUP #####

jointPickPose = [0.000, 1.600, -0.500, -0.600]
jointPlacePoseL = [0.800, 1.550, -0.500, -1.050]
jointPlacePoseR = [-0.800, 1.550, -0.500, -1.050]
jointMovePose = [0.000, 0.600, 0.400, -1.000]

##### TASK SPACE POSE SETUP #####

taskTuckPose = [0.080, 0.000, 0.350, 0.000]
taskPickPose = [0.300, 0.000, 0.040, 0.000]
taskMovePose = [0.300, 0.000, 0.200, 0.000]

# used to visualise pahs with rviz
display_trajectory_publisher = rospy.Publisher(
    "arm_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size = 20
)

# method that sends a joint goal to the move_group and executes a planned move if not already at the goal
def moveToJointPose(pose):
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

# method that sends a pose goal to the move group and executes a planned move
def moveToTaskPose(pose):
    arm_group.clear_pose_targets()
    # gets the current task space position (x,y,z,w) - w = orientation of the end effector
    pose_goal = geometry_msgs.msg.Pose()

    # updates the pose_goal to the given pose 
    print(f"{pose[0]}, {pose[1]}, {pose[2]}")
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]

    # only adds the orientation to the plan if specified by the parameter
    if len(pose) > 3:
        pose_goal.orientation.w = pose[3]

    # updates the arm_group pose target
    arm_group.set_pose_target(pose_goal)

    # executes the planned move
    success = arm_group.go(wait=True)

    # stop() ensures that there is no residual movement and 
    # clear prevents the move group from unnecessarily planning to the current pose
    arm_group.stop()
    arm_group.clear_pose_targets()
    rospy.sleep(0.75)


# method to move the arm to jointPickPose and close the gripper
def pickObject():
    print("Picking up object")

    closeGripper()

    if JOINT_PLANNING:
        moveToJointPose(jointMovePose)
    else:
        moveToTaskPose(taskMovePose)


# method to move the arm to jointPlacePoseL  and open the gripper
def placeObject(direction):

    if direction == "left":
        print("Placing object on the " + direction)
        if JOINT_PLANNING:
            moveToJointPose(jointPlacePoseL)

    elif direction == "right":
        print("Placing object on the " + direction)
        if JOINT_PLANNING:
            moveToJointPose(jointPlacePoseR)

    else:
        print(direction + " invalid. Must give direction: 'left'/'right'")
        return

    openGripper()

# method to move the arm to jointMovePose - default state
def resetArm():
    if JOINT_PLANNING:
        moveToJointPose(jointMovePose)
    else:
        moveToTaskPose(taskTuckPose)

    

# method to move the arm to the position where ready to grasp an object
def positionArm():
    if JOINT_PLANNING:
        moveToJointPose(jointPickPose)
    else:
        moveToTaskPose(taskPickPose)

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

    # set initial state to jointMovePose
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