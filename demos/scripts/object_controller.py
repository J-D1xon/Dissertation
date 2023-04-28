#!/usr/bin/env python3

# Imports
##########################
import rospy 
import math
import random
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import String

class Gazebo_Object():

    def repose_callback(self, data):
        if data.data == "repose":
            print("setting state position")
            
            self.state_msg.model_name = self.name
            self.state_msg.pose.position.x = 1.5
            self.state_msg.pose.position.y = 0.5
            self.state_msg.pose.position.z = 0

            self.state_msg.pose.orientation.x = 0
            self.state_msg.pose.orientation.y = 0
            self.state_msg.pose.orientation.z = 0
            self.state_msg.pose.orientation.w = 0

            self.new_state = True

    def __init__(self):
        self.gazebo_ns = "/gazebo" # Gazebo namespace

        rospy.init_node('object_controller', anonymous=True)
        
        rospy.wait_for_service(self.gazebo_ns + "/set_model_state")

        print("subscribing to /repose_can ... ")
        repose_sub = rospy.Subscriber("/repose_can", String, self.repose_callback, queue_size=1)

        #self.gazebo_ns = "/gazebo" # Use this namespace if launched with ./launch_full.sh
        self.name = "can1" # Object name
        self.new_state = False
        self.state_msg = ModelState()

        self.main()


    def main(self):
        # Initilise the node
        
        while not rospy.is_shutdown():
            if self.new_state:
                self.new_state = False

                try:
                    set_state = rospy.ServiceProxy(self.gazebo_ns + "/set_model_state", SetModelState)
                    resp = set_state( self.state_msg )
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

        rospy.spin()

if __name__ == "__main__":

    target_can = Gazebo_Object()
