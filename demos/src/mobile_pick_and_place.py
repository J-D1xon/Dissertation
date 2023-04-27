#!/usr/bin/env python3

##### IMPORTS #####
import rospy
import numpy as np
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
PICK_POSE = [ 0.000,  1.500, -0.500, -0.750]

class MobilePicker:

    def shutDownCallback(self):
        print("Shutting down... ")
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.cmd_vel.publish(self.vel)

        arm_group.go(INIT_POSE,wait=True)
        arm_group.go(TUCK_POSE,wait=True)
        arm_group.stop()
        gripper_group.go([-0.01, -0.01], wait=True)
        gripper_group.stop()


    # callback function is triggered when a new frame is available
    # it manipulates the frame and returns the average position of cans in the frame
    def cam_callback(self, img_data):
        
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

    # def depth_callback(self, depth_data):
    #     pass

    def can_coverage(self):
        return round(np.count_nonzero(self.mask) / (len(self.mask) * len(self.mask[1])), 2)

    def check_for_base_move(self):
        if self.avg_mask > 0:
            if self.avg_mask < 330:
                print("turning left")
                self.vel.angular.z = 0.05
                self.angularAligned = False

            elif self.avg_mask > 340:
                print("turning right")
                self.vel.angular.z = -0.05
                self.angularAligned = False

            else:
                print("no turn needed")
                self.vel.angular.z = 0
                self.angularAligned = True


            if self.can_coverage() < 0.20:
                    print("moving forward")
                    self.vel.linear.x = 0.02
                    self.linearAligned = False

            elif self.can_coverage() > 0.20:
                print("backing up")
                self.vel.linear.x = -0.02
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
            self.MODE = 2
            self.linearAligned = False
            self.angularAligned = False

        self.cmd_vel.publish(self.vel)

    def __init__(self):
        self.rate = rospy.Rate(5) #Hz
        rospy.on_shutdown(self.shutDownCallback)

        self.MODE = 1 # 1 - track towards | 2 - pick up | 3 - take back to spawn

        # setup for computer vision  
        self.cvbridge_interface = CvBridge()
        self.newFrame = False
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cam_callback, queue_size=1)
        #self.depth_sub = rospy.Subscriber("/camera/rgb/image__raw/compressedDepth", CompressedImage, self.depth_callback, queue_size=1)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=0)
        self.vel = Twist()
        self.linearAligned  = False
        self.angularAligned = False

        arm_group.go(TUCK_POSE,wait=True)
        arm_group.stop()

        gripper_group.go([-0.01, -0.01], wait=True)
        gripper_group.stop()
        self.main()

    def main(self):
        print("switching to MODE 1 - tracking")
        while not rospy.is_shutdown():

            if self.MODE == 1:
                if self.newFrame:
                    self.newFrame = False

                    self.check_for_base_move()

            elif self.MODE == 2:
                arm_group.go(INIT_POSE,wait=True)
                arm_group.stop()
                gripper_group.go([0.01, 0.01], wait=True)
                gripper_group.stop()

                scene.attach_box(eef_link,can_name,touch_links=touch_links)
                arm_group.go(PICK_POSE,wait=True)
                arm_group.stop()

                gripper_group.go([-0.01, -0.01], wait=True)
                gripper_group.stop()

                arm_group.go(INIT_POSE,wait=True)
                arm_group.stop()

                self.MODE = 3

                    
            self.rate.sleep()
        rospy.spin()


if __name__ == "__main__":
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mobile_pick_and_place", anonymous=True)
    
    node = MobilePicker()