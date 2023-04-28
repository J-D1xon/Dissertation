#!/usr/bin/env python3

##### IMPORTS #####
import rospy
import numpy as np
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import open_manipulator_msgs.msg
from pathlib import Path

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from moveit_msgs.msg import Grasp

import cv2
from cv_bridge import CvBridge, CvBridgeError

from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *

from tf.transformations import euler_from_quaternion

##### MOVEIT! SETUP #####

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

eef_link = arm_group.get_end_effector_link()
touch_links = robot.get_link_names("gripper")

can_name = "can1"

##### POSES #####

INIT_POSE = [ 0.000,  0.000,  0.000,  0.000]
TUCK_POSE = [ 0.000, -1.700,  1.200,  0.700]
PICK_POSE = [ 0.050,  1.550, -0.500, -0.850]

##### MOVEIT METHODS #####

# method to move the gripper arms to the closed position
def close_gripper():
    gripper_group.go([-0.01, -0.01], wait=True)
    gripper_group.stop()

def grasp_gripper():
    gripper_group.go([-0.01, -0.01], wait=True)

# method to move the gripper arms to the open position
def open_gripper():
    gripper_group.go([0.01, 0.01], wait=True)
    gripper_group.stop()

# method that sends a joint goal to the move_group and executes a planned move if not already at the goal
def move_to_pose(pose):
    # store the current joint angles
    joint_angles = arm_group.get_current_joint_values() 

    # used to keep track of joints that don't need to move
    sameJoints = 0

    for i in range(len(joint_angles)):
        # recognises a joint is already at the pose angle if its within a threshold 'eps' of the target pose (0.005 rad)
        if joint_angles[i]-0.005 < pose[i] and joint_angles[i]+0.005 > pose[i]:
            sameJoints += 1

    # if any of the joints require movement then plan a move, else do nothing
    if sameJoints < len(joint_angles):
        print(f"moving to new joint pose: {pose}")

        arm_group.go(pose, wait=True)
        arm_group.stop()
    else:
        print("already at the given pose")

class MobilePicker:

    def shutDownCallback(self):
        print("Shutting down... ")
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.cmd_vel.publish(self.vel)

        move_to_pose(INIT_POSE)
        move_to_pose(TUCK_POSE)
        close_gripper()

    # callback function is triggered when a new frame is available
    # it manipulates the frame and returns the average position of cans in the frame
    def cam_callback(self, img_data):
        # only process a new fram when its needed for tracking
        if self.MODE == 1:
            # converst the image data to a cv image for manipulation
            try:
                cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            # gets the height and width for cropping to reduce image size
            height, width, _ = cv_img.shape

            # crops the image to half its original size
            cropped_img = cv_img[int(height/2):height, 0:width]

            # converts the image to greyscale to make the cans stand out more
            greyscale = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
            upper, lower = 60, 0

            # masks out the cans based on an upper and lower threshold
            self.mask = cv2.inRange(greyscale, lower, upper)

            # print(f"{np.amax(mask)}, {np.amin(mask)}")

            # returns the average position of a can along the width of the image
            if len(np.nonzero(self.mask)[1]) > 0:
                self.avg_mask = np.average(np.nonzero(self.mask)[1])
            else:
                self.avg_mask = -1
            #print(self.avg_mask)
            self.newFrame = True

    # callback function is triggered when a new odometry measurment is available
    # it provides positional data using dead-reckoning 
    def odom_callback(self, odom_data):
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (_, _, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = round(pos_x, 2)
        self.y = round(pos_y, 2)
        self.theta_z = round(yaw, 2) 

        if self.startup:
            self.startup = False

            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

            self.place_target = self.x0 - 0.25

    # calculates the percentage of the frame taken up by masked objects
    def can_coverage(self):
        return round(np.count_nonzero(self.mask) / (len(self.mask) * len(self.mask[1])), 2)

    # returns true when the robot is within 2cm from point (x, y) 
    def at_origin(self, x, y):
        x_threshold = (self.x < x + 0.02) and (self.x > x - 0.02)
        y_threshold = (self.y < y + 0.02) and (self.y > y - 0.02)

        return x_threshold and y_threshold

    # calculates the heading required to move in a straight line from the robot's position to (x, y)
    def calculate_heading(self, x, y):
        self.target_heading = round(-math.pi + math.atan2(self.y-y, self.x-x), 2)

        if self.target_heading < -math.pi:
            self.target_heading = round(math.pi - (abs(self.target_heading) - math.pi), 2)

    # MODE 1
    def check_for_base_move(self):
        if self.avg_mask > 0:
            self.angularAligned = False

            if self.avg_mask < 300:
                print("turning left quickly")
                self.vel.angular.z = self.FAST
                
            elif self.avg_mask < 340:
                print("turning left slowly")
                self.vel.angular.z = self.SLOW

            elif self.avg_mask > 380:
                print("turning right quickly")
                self.vel.angular.z = -self.FAST

            elif self.avg_mask > 340:
                print("turning right slowly")
                self.vel.angular.z = -self.SLOW

            else:
                print("no turn needed")
                self.vel.angular.z = 0
                self.angularAligned = True


            if self.can_coverage() < 0.15:
                print("moving forward quickly")
                self.vel.linear.x = self.FAST
                self.linearAligned = False
            elif self.can_coverage() < 0.22:
                print("moving forward slowly")
                self.vel.linear.x = self.SLOW
                self.linearAligned = False

            elif self.can_coverage() > 0.22:
                print("backing up")
                self.vel.linear.x = -self.SLOW
                self.linearAligned = False

            else:
                print("no linear move needed")
                self.vel.linear.x = 0
                self.linearAligned = True

        else:
            print("no cans in sight")
            self.vel.linear.x = 0
            self.vel.angular.z = 0.1
            self.linearAligned  = False
            self.angularAligned = False

        if self.linearAligned and self.angularAligned:
            print("switching to MODE 2 - pick up")
            print(self.avg_mask)
            self.MODE = 2
            self.linearAligned = False
            self.angularAligned = False

        self.cmd_vel.publish(self.vel)

    # MODE 2
    def pick_can(self):
        move_to_pose(INIT_POSE)

        open_gripper()

        move_to_pose(PICK_POSE)

        scene.attach_box(eef_link,can_name,touch_links=touch_links)

        grasp_gripper()

        move_to_pose(INIT_POSE)

        ## add a check to confirm grasp

        self.MODE = 3
        self.angularAligned = False

    # MODE 3
    def return_to_origin(self):
        # if more than 10cm from the origin, calculate a new heading 
        if ((self.x-(self.place_target))**2 + (self.y-self.y0)**2) > 0.1**2:
            self.calculate_heading(self.place_target, self.y0)
        else:
            print("within 10cm of (x0, y0)")

        print(f"{self.x}:{self.x0}, {self.y}:{self.y0} | {self.theta_z}:{self.target_heading}")
        
        # add angular velocity if not on the heading
        if self.target_heading - 0.01 > self.theta_z:
            self.vel.angular.z = self.FAST
            # self.angularAligned = False
        elif self.target_heading + 0.01 < self.theta_z:
            self.vel.angular.z = -self.FAST
            # self.angularAligned = False
        else:
            self.vel.angular.z = 0
            self.angularAligned = True
        
        # add linear velocity if aligned and not at the origin
        if not self.at_origin(self.place_target, self.y0):
            if self.angularAligned:
                self.vel.linear.x = self.FAST
            else:
                self.vel.linear.x = 0
        else:
            print("At the origin")
            self.vel.linear.x = 0
            self.angularAligned = False

            print("switching to MODE 4 - reorient and place")
            self.MODE = 4

        self.cmd_vel.publish(self.vel)

    #MODE 4
    def reorient_and_place(self):
        if not self.angularAligned:
            print(f"Aligning: {self.theta_z}")

            if self.theta_z < round(math.pi, 2) and self.theta_z > 0:
                self.vel.angular.z = self.FAST
            elif self.theta_z > -round(math.pi, 2) and self.theta_z < 0:
                self.vel.angular.z = -self.FAST
            else:
                self.angularAligned = True
                self.vel.angular.z = 0
                print("Aligned, placing can")

            self.cmd_vel.publish(self.vel)

        else:
        
            move_to_pose(PICK_POSE)

            open_gripper()

            move_to_pose(INIT_POSE)
            move_to_pose(TUCK_POSE)

            close_gripper()

            print("Placed object")
            self.angularAligned = False
            print("switching to MODE 5 - resetting")
            self.repose_pub.publish(self.repose)
            self.MODE = 5

    # MODE 5
    def reset_robot(self):
        if not self.angularAligned:
            print(f"Aligning: {self.theta_z}")

            if self.theta_z < 0:
                self.vel.angular.z = self.FAST
            elif self.theta_z > 0:
                self.vel.angular.z = -self.FAST
            else:
                self.angularAligned = True
                self.vel.angular.z = 0

            self.cmd_vel.publish(self.vel)

        else:
            print("Reset to start")
            self.angularAligned = False
            print("switching to MODE 1 - tracking")
            self.MODE = 1


    def __init__(self):
        self.rate = rospy.Rate(5) #Hz
        rospy.on_shutdown(self.shutDownCallback)
        self.startup = True

        self.MODE = 5 # 1 - track towards | 2 - pick up | 3 - take back to spawn | 4 - reorient and place | 5 - reset

        # setup for computer vision  
        self.cvbridge_interface = CvBridge()
        self.newFrame = False
        self.range = 0.21
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback, queue_size=1)
        #self.depth_sub = rospy.Subscriber("/camera/rgb/image__raw/compressedDepth", CompressedImage, self.depth_callback, queue_size=1)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.vel = Twist()
        self.FAST = 0.1
        self.SLOW = 0.025
        self.linearAligned  = False
        self.angularAligned = False

        self.nav_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        arm_group.go(TUCK_POSE,wait=True)
        arm_group.stop()

        self.x = 0
        self.y = 0
        self.x0 = 0
        self.y0 = 0
        self.theta_z = 0
        self.theta_z0 = 0

        self.repose_pub = rospy.Publisher("/repose_can", String, queue_size=0)
        self.repose = String()
        self.repose = "repose"

        self.place_target = 0

        self.target_heading = 0

        close_gripper()

        self.main()

    def main(self):
        print(f"starting in MODE {self.MODE}")
        while not rospy.is_shutdown():

            if self.MODE == 1:
                if self.newFrame:
                    self.newFrame = False

                    self.check_for_base_move()

            elif self.MODE == 2:
                self.pick_can()

            elif self.MODE == 3:
                self.return_to_origin()

            elif self.MODE == 4:
                self.reorient_and_place()

            elif self.MODE == 5:
                self.reset_robot()

                    
            self.rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mobile_pick_and_place", anonymous=True)
    
    node = MobilePicker()