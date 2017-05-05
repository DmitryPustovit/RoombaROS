#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import sys
import math
import copy
import os.path
import time
from pgm import pgm
from display import *

xG = 0
yG = 0
yaw = 0
timeRate = 60
twist = Twist()
j,p = 0+288, 0+240

#Calculates the distance between two points
def calcDistance(x1, y1, x2, y2):
    return int(round(math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2)), 0))

def callback_pose(data):
    global xG
    global yG
    global yaw
    xG = data.pose.pose.position.x
    yG = data.pose.pose.position.y
    yaw = data.pose.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
#-288, 240

step = 5

class Fire:
    limit = 1
    goal = [0,0]
    direct = [0, 45, 90 ,135, 180, 225, 270, 315]

    #Base board init
    def __init__(self, board, width, height):

        self.space = []
        self.height = height
        self.width = width

        #Clean the board
        for x in range(len(board)):
            if board[x] == 254:
                x,y = self.resolve(x)
                for xC,yC in [[1,0],[0,-1],[-1,0],[0,1]]:
                    xN = xC + x
                    yN = (y + yC)

                    if board[xN + yN * self.width] == 205:
                        count = 0
                        for xD,yD in [[1,0],[0,-1],[-1,0],[0,1]]:
                            if board[xN + xD + ((yN + yD) * self.width)] == 254:
                                count = count + 1

                        if count == 4:
                            board[xN + yN * self.width] = 206
                        elif count == 3:
                            board[xN + yN * self.width] = 206
                            whites = 0
                            for g in range(-6,6):
                                for h in range(-6,6):
                                    if board[xN + g + ((h + yN) * self.width)] == 254:
                                        whites = whites + 1
                            if whites > 60:
                                for g in range(-6,6):
                                    for h in range(-6,6):
                                        board[xN + g + ((h + yN) * self.width)] = 254

        for x in range(len(board)):
            if board[x] < 206:
                board[x] = 1
            else:
                board[x] = 0
                self.space.append(x)

        self.board = board
        self.tail = len(self.space) - 1
        self.lightAFire()

    #Loads a board from a file
    @classmethod
    def fromFile(cls, fileName):
        p = pgm(fileName)
        return cls(p.bytes, p.width, p.height)

    def isThere(self, x, y = None):
            if (x >= 0):
                if y == None:
                    return x < self.width * self.height
                else:
                    return x < self.width and y >= 0 and y < self.height
            return False

    def get(self, x, y = None):
        if y == None:
            return self.board[x]
        else:
		#CHANGED SECOND WIDTH TO HEIGHT CUS BRANDON HATES
            return self.board[x % self.width + y * self.width]


    def set(self,x,y,val):
        self.board[(x % self.width) + (y * self.width)] = val

    def resolve(self, i):
        return i % self.width , int(i / self.width)

    def ignore(self, p1):
        t = self.space[self.tail]
        self.space[self.tail] = -1
        self.space[p1] = t
        self.tail -= 1

    #Checks to see if the space is zero (rolls two checks into one)
    def shouldChangeSpace(self,x, y):
        if self.isThere(x,y):
            if self.get(x,y) != 0:
                return True
        return False

    #Burns the board (Note this can be easily changed to do both 4 and 8 point)
    def burn(self,i):
        x, y = self.resolve(i)
        if not self.isThere(x,y):
            return False

        if self.get(x,y) != 0:
            return False

        changed = False
        val = self.limit + 1
        for xC,yC in [[1,0],[0,-1],[-1,0],[0,1],[1,1],[-1,-1],[-1,1],[1,-1]]:
            if self.isThere(x + xC, y + yC):
                if self.get(x + xC, y + yC) <= self.limit and self.get(x + xC, y + yC) < val and self.get(x + xC, y + yC) > 0 :
                    val = self.get(x + xC, y + yC)

        if val > self.limit:
            return False

        if val:
            val += 1
            self.set(x,y,val)
            changed = True
        return changed

    #Runs the Brush Fire Algorithm
    def lightAFire(self):
        hasChanged = True
        self.limit = 1

        while hasChanged:
            hasChanged = False
            for i in range(len(self.space)):
                if i == -1:
                    break
                if self.burn(self.space[i]):
                    hasChanged = True
                    self.ignore(i)
            self.limit = self.limit + 1

    def setGoal(self, x, y):
        print(self.get(x,y))
        if self.isThere(x,y) and self.get(x,y) > 1:
            self.goal = [x,y]
            return True
        else:
            return False

    #Less then check
    @classmethod
    def coldCheck(cls,val1, val2):
        if (val2 == 0 or val1 < val2):
            return True
        return False

    #Greater then check
    @classmethod
    def hotCheck(cls,val1, val2):
        if (val2 == 0 or val1 > val2):
            return True
        return False

    #Gets the potential based on the function passed
    def getPotential(self,x,y,test):
        if not self.isThere(x,y):
            raise ValueError('A very specific bad thing has happened' + str(x) + '  ' + str(y))
        mainValue = 0
        direction = 4
        for xC,yC,ang in [[1,0,0],[1,1,1],[0,1,2],[-1,1,3],[-1,0,4],[-1,-1,5],[0,-1,6],[1,-1,7]]:
            if self.isThere(x + xC,y + yC):
                val = self.get(x + xC,y + yC)
                if (test(val, mainValue)):
                    mainValue = val
                    direction = ang
        return self.direct[direction]

    def getAttractive(self, x, y):
	print(str(x) + ' att '+ str(y))
	x = int(x)
	y = int(y)
        return self.getPotential(x,y,self.hotCheck)

    def getRepulsive(self, x, y):
	print(str(x) + ' rep '+ str(y))
        return self.getPotential(x,y,self.coldCheck)

    def getHeading(self, xC, yC, head):
	print (str(xC) + ' hea ' + str(yC))
        dist = calcDistance(xC, yC, self.goal[0], self.goal[1])
        cAtAngle = self.getAttractive(xC, yC)
        angle = math.radians(head)
        change = [step * math.cos(angle), step * math.sin(angle)]
	print("Angles : " + str(math.cos(angle)) + "   " + str(math.sin(angle)));
        nX, nY = xC + change[0], yC + change[1]
	print (str(change[0]) + ' hea  n' + str(change[1]))
	print (str(xC) + ' hea ' + str(yC))
        nAtAngle = (self.getAttractive(nX, nY))
        gAngle = math.atan2(xC - self.goal[0], yC - self.goal[1]) * (180/3.14159)
        influ = 1 - ((math.log(dist) + .000000000001) / 3)
        otherInflu = (1 - influ) / 2
        return cAtAngle * otherInflu + nAtAngle * otherInflu + gAngle * influ

#f = Fire.fromFile("roomba_hall.pgm")
f = Fire.fromFile("simulationMap.pgm")
#f = Fire.fromFile("thirdFloorMap.pgm")
print(f.setGoal(j,p))

#b = Board.fromFile("roomba_hall.pgm")
a = []
for x in f.board:
    a.append(abs((x-1) * 5))
a[j +  p * f.width] = 254
displaypgm(a,f.width, "class_out.png")
print("Debug Map Output Done!")


rospy.init_node("navigation_snippet")
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
pub = rospy.Publisher('/mobile_base/commands/velocity',Twist, queue_size=1)
rate = rospy.Rate(timeRate)
counter = 0
twist.linear.x = .15

wait = 0

while (wait < 2000):
     print("Current robot pose: x=" + str(xG) + "y=" + str(yG) + " yaw=" + str(degrees(yaw)))
     wait = wait+1;
     print("waiting");

while True:
     print("Current robot pose: x=" + str(xG) + "y=" + str(yG) + " yaw=" + str(degrees(yaw)))
     degree = degrees(yaw)
     if(degree < 0):
     	degree = 180 + (180 + degree) 

     uRot = f.getHeading(int((xG/0.05)+288), int((yG/0.05)+240), degree)
     if(degrees(yaw) > uRot):
	twist.angular.z = .5
     if(degrees(yaw) < uRot):
	twist.angular.z = -.5
     pub.publish(twist)
     #print(str(degrees(yaw)) + ' DEG ' + str(uRot));

	











