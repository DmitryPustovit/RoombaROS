#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

x = 0
y = 0
yaw = 0

def callback_pose(data):
    global x
    global y
    global yaw
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    yaw = degrees(data.pose.pose.orientation.w)
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])

    # Read the current pose topic
rospy.init_node("navigation_snippet")
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
counter = 0
while True:
    counter+=1
    #if(counter == 10000):
    print("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)))




