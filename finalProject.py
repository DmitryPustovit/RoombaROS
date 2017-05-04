#!/usr/bin/env python
import rospy
import math
import sys
import requests
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#Position Tracking
xPosCur = 0.0
yPosCur = 0.0
angCur = 0.0
#Velocities
dirVel = 0.2
dirVelMax = 0.33
angVel = 0.52
angVelMax = 0.52
timeRate = 60
twist = Twist()
#Checks to see if a string is a number




pub = rospy.Publisher('/mobile_base/commands/velocity',Twist, queue_size=1)
rospy.Subscriber('odom',Odometry,odometryCb)
rospy.init_node('run_py',anonymous=True)
rate = rospy.Rate(timeRate)
#printInfo()
#Main loopty loop
#while not rospy.is_shutdown():
#        s = raw_input('turtle :')
#        params = s.split(' ')
        
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
    #ap = r.content
    pub.publish(twist)
    #getCommand()
