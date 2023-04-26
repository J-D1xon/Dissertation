#!usr/bin/env python3

##### IMPORTS #####
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import open_manipulator_msgs.msg

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from moveit_commander.conversions import pose_to_list
from open_manipulator_msgs.srv import *

##### MOVEIT! SETUP #####

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

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

class MobilePicker:

    def odom_callback(self, topic_data:Odometry):
        self.pose = topic_data.pose.pose

    def __init__(self):

        self.nav_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)
        self.cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.
        return

    def main(self):
        while not rospy.is_shutdown():

        rospy.spin()


if __name__ == "__main__":
    # initilising the moveit_commander and ROS node, assigns the shutdown callback
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mobile_pick_and_place", anonymous=True)
    rospy.on_shutdown(shutDownCallback)

    node = MobilePicker()