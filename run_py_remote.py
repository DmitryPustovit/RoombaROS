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
def isNum(val):
    try:
       temp = float(val)
    except ValueError:
       return False
    return True
#Prints info
def printInfo():
    print("===================================================================")
    print('Simple commands:')
    print("'forward'   moves the turtle forward .33 meters")
    print("'backward'  moves the turtle backward .33 meters")
    print("'turnRight' turns the turtle 90 degrees to the right")
    print("'turnLeft'  turns the turtle 90 degrees to the left")
    print("'dance'     executes a series of smooth moves")
    print("===================================================================")
    print('Commands:')
    print("[val] is the value you wish to pass")
    print("'f <d | t> [val]' moves forward based on time (t) or distance (d)")
    print("'b <d | t> [val]' moves backward based on time (t) or distance (d)")
    print("'r <l | r> [val]' rotates left(l) or right(r)")
    print("'v <a | l> [val]' sets the angular(a) or linear(l) velocity)")
    print("Example: 'f t 1.5' moves the turtle forward for 1.5 seconds")
    print("===================================================================")
#Moves for the given time
def moveTime(time, back):
    time = (time * timeRate)
    vel = dirVel
    if (back):
        vel = -vel
    twist.linear.x = vel
    twist.angular.z = 0
    while (time > 0):
        pub.publish(twist)
        rate.sleep()
        time = time - 1
    twist.linear.x = 0
    pub.publish(twist)
#Moves the given distance
def moveDist(dist):
    if (dist > 0):
        twist.linear.x = dirVel
    else :
        twist.linear.x = -dirVel
    twist.angular.z = 0;
    dist = abs(dist)
    if (dist > 0.33):
        dist = .33
    distMoved = 0
    originX = xPosCur
    originY = yPosCur
    while (distMoved < dist):
        pub.publish(twist)
        distMoved = math.sqrt(math.pow(originX - xPosCur, 2) + math.pow(originY - yPosCur, 2))
        rate.sleep()
        print(distMoved)
    twist.linear.x = 0
    pub.publish(twist)
#Turns the number of degrees given
def turn(angle):
    if angle > 180:
        angle = 180
    elif angle < -180:
        angle = -180
    angDest = ((angCur - angle + 360) % 360)
    print(angCur)
    print(angDest)
    twist.linear.x = 0
    if angle > 0:
        twist.angular.z = -angVel
    else:
        twist.angular.z = angVel
    while (not(angCur < (angDest + 1) and angCur > (angDest - 1))):
        print(str(angCur) + " " + str(angDest))
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)
#Sets the angular velocity
def setAngVel(speed):
    global angVel
    if (speed < 0 or speed > angVelMax):
        return False
    else :
        angVel = speed
        print(angVel)
        return True
#Sets the directional velocity
def setDirVel(speed):
    global dirVel
    if (speed < 0 or speed > dirVelMax):
        return False
    else :
        dirVel = speed;
        return True
#Subscriber Function
def odometryCb(msg):
    global angCur
    global xPosCur
    global yPosCur
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    a,b,yaw = euler_from_quaternion(quaternion)
    yawDegrees = yaw * 180 / 3.15149
    if (yawDegrees < 0):
        angCur = yawDegrees + 360
    else:
        angCur = yawDegrees
    xPosCur = msg.pose.pose.position.x
    yPosCur = msg.pose.pose.position.y
#Parses input
def processInput(params):
    key = params[0]
    if (key == 'q'): #Used to quit
        sys.exit(0)
    elif (key == 'turnRight') :
        turn(90)
    elif (key == 'turnLeft') :
        turn(-90)
    elif (key == 'forward') :
        moveDist(.33)
    elif (key == 'backward') :
        moveDist(-.33)
    elif (key == 'dance') :
        print("Lets dance!")
        orgDVel = dirVel
        orgAVel = angVel
        setDirVel(.16)
        setAngVel(angVelMax)
        moveDist(.33)
        moveDist(-.33)
        for i in range(0,4):
            turn(-90)
            moveDist(-.25)
        turn(180)
        turn(180)
        setDirVel(orgDVel)
        setAngVel(orgAVel)
        print("That was fun!")
    elif (key == 'v'): #Sets Velocity
        if (len(params) != 3):
            print("Invalid usage!")
            printInfo()
        elif (not isNum(params[2])):
            print("Invalid number!")
            printInfo()
        elif (params[1] == 'a'):
            if (setAngVel(float(params[2]))):
                print("Angular velocity set!")
            else:
                print("Error, angular velocity not set!")
        elif (params[1] == 'l'):
            if (setDirVel(float(params[2]))):
                print("Linear velocity set!")
            else:
                print("Error, directional velocity not set!")
        else :
            print("Invalid flag")
            printInfo()
    else : #Movement commands
        if (len(params) != 3):
            print("Invalid usage!")
            printInfo()
        elif (not isNum(params[2])):
            print("Invalid number!")
            printInfo()
        elif (key == 'f' or key == 'b'): #Forwards or backwards
            back = False
            if (key == 'b'):
                back = True
            if (params[1] == 'd'):
                dist = float(params[2])
                if (back):
                    dist = -dist
                moveDist(dist)
            elif (params[1] == 't'):
                time = float(params[2])
                if (time > 2):
                    time = 2
                moveTime(time, back)
            elif (params[1] == 'o'): #Move forward (raw)
                  moveTime(params[2], False)
            elif (params[1] == 'p'): #Move back (raw)
                  moveTime(params[2], True)
            else :
                print("Invalid flag!")
                printInfo()
        elif (key == 's' or key == 'r'): #Spin or rotate (same thing different flag)
            num = float(params[2])
            if (params[1] == 'r'):
                  turn(num)
            elif (params[1] == 'l'):
                  turn(-num)
            elif (params[1] == 'o'): #(raw)
                  turn(num)
            elif (params[1] == 'p'): #(raw)
                  turn(num)
            else:
                print("Invalid flag!")
                printInfo()
        else:
            printInfo()
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
