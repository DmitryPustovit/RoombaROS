#!/usr/bin/env python
import rospy
import math
import sys
import requests
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


pub = rospy.Publisher('/mobile_base/commands/velocity',Twist, queue_size=1)
rospy.Subscriber('odom',Odometry,odometryCb)
rospy.init_node('run_py',anonymous=True)
rate = rospy.Rate(timeRate)
        
speed = 0.15
ap = "5"

def getCommand():
  #threading.Timer(0.4, getCommand).start()
  r = requests.get("https://turtle-ui.herokuapp.com/command", verify=False ).json()
  #print(r.content)
  if(r == "0"):
    twist.linear.x = 0
    twist.angular.z = 0
  elif(r == "1"):
    twist.linear.x = .15
    twist.angular.z = 0
  elif(r == "2"):
    twist.linear.x = -.15
    twist.angular.z = 0
  elif(r == "3"):
    twist.linear.x = 0
    twist.angular.z = .5
  elif(r == "4"):
    twist.linear.x = 0
    twist.angular.z = -.5
    
twist.linear.x = .15
twist.angular.z = 0
counter = 0
while True:
    counter+=1
    if(counter == 1000):
	getCommand()
	counter = 0
    print(twist.linear.x)
    pub.publish(twist)
