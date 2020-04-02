#!/usr/bin/env python

#standard imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty

import time
from myClasses import *
from basics import *
from math import *
import sympy as sp
import numpy as np

#max Linear vel = 0.22	Angular = 2.84

#________________________________________Global Variables__________________________________#
robotYaw = BotX = BotY = 0
freq = 100
laserData = [100]
iterations = 0
#_________________________________________Classes__________________________________________#


ppidOmega = PID(2.0,0,0,pi/2,-2.84,2.84) #orientation control
ppidFollowB = PID(0.01,0.1,0,0,-2.84,2.84)
#_______________________________________________Functions_______________________________________#

def initSystem():
	rospy.init_node('my_robot',anonymous='True')	

	rospy.Subscriber("/odom",Odometry,getXY)
	rospy.Subscriber("/imu",Imu,getYaw)
	rospy.Subscriber("/scan",LaserScan,getLaserData)

	startTime = time.time()
	resetOdom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
	while time.time() - startTime < 1.0:
		resetOdom.publish(Empty())
		
	print "__Odometry Reset__"

#______________________________________________Callbacks________________________________________#	
def getLaserData(laser):
	global laserData	
	laserData = laser.ranges #m
			

	
def getXY(PoseWithCovariance):	
	global BotX, BotY # My Axis
	BotX =  (PoseWithCovariance.pose.pose.position.x - 0)*100 #cm
	BotY =  (PoseWithCovariance.pose.pose.position.y - 0)*100  #cm


def getYaw(Quaternion):	
	global robotYaw	
	qx = Quaternion.orientation.x
	qy = Quaternion.orientation.y
	qz = Quaternion.orientation.z
	qw = Quaternion.orientation.w
	
	robotYaw = (atan2(2*(qz*qw + qy*qx),1- 2*(qz**2 + qy**2))) 

#___________________________________________ex__Main___________________

   #0 1 2 3 4 5 6 7 8 9
#0 [_ _ _ _ _ _ _ _ _ _] 
#1 [_ _ _ # _ _ _ # _ _]
#2 [_ _ # # _ _ _ # _ _]
#3 [_ _ # # _ _ _ # _ _]
#4 [_ _ # # # _ _ _ _ _]
#5 [_ _ # # # # _ _ _ _]
#6 [_ _ _ _ # # _ _ _ _]
#7 [_ _ _ _ # # _ _ _ _]
#8 [_ _ _ _ _ # _ # _ _]
#9 [_ _ _ _ _ # _ _ _ _]
#10[S _ _ _ _ # _ _ _ G] Start = (10,0) Goal(3,6)
Map = [				  
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,1,0,0,0,1,0,0],
	[0,0,0,1,0,0,0,1,0,0],
	[0,0,1,1,0,0,0,1,0,0],
	[0,0,1,1,0,0,0,0,0,0],
	[0,0,1,1,1,0,0,0,0,0],
	[0,0,1,1,1,1,0,0,0,0],
	[0,0,0,0,1,1,0,0,0,0],
	[0,0,0,0,1,1,0,1,0,0],
	[0,0,0,0,0,1,0,0,0,0],
	[0,0,0,0,0,1,0,0,0,0]
]


OPEN = []
OPEN_ = []

CLOSED = []
CLOSED_ = []

INCONSI = []
INCONSI_ = []

def extractIndicies(s):
	i = ""
	j = ""
	for k in range(len(s)):
		if s[k] != ' ' and s[k] != 's':
			i = i + s[k]
		elif s[k] == ' ':
			j = j + s[k:]
			break

	return int(i),int(j)
			
			
def InClosed(s):
	global CLOSED_

	for i in range(len(CLOSED_)):
		if s == CLOSED_[i]:
			return i
	return -1

def InOpen(s):
	global OPEN_

	for i in range(len(OPEN_)):
		if s == OPEN_[i]:
			return i
	return -1

def getSolution(xs,ys,index):
	global OPEN ,CLOSED, Map

	solution = []
	temp = Map

	
	goal = 	OPEN[index].name
	i, j = extractIndicies(goal)		
	solution.append(goal)	
	temp[i][j] = 8	
	
	
	backPointer = OPEN[index].backpointer
	solution.append(backPointer)
	i, j = extractIndicies(backPointer)		
	temp[i][j] = 8	

	while backPointer != "s"+str(xs)+" "+str(ys):
						
		index = InClosed(backPointer)		
		backPointer = CLOSED[index].backpointer
		solution.append(backPointer)	
		i, j = extractIndicies(backPointer)		
		temp[i][j] = 8	
		
		
		
	#print solution
	return temp
		
	
def checkGoal(s,xg,yg):
	if s == "s"+str(xg)+" "+str(yg):
		return 1
	return 0

def ARAStar(xs,ys,xg,yg,e):
	
	global Map, OPEN, OPEN_,CLOSED_, CLOSED, INCONSI, INCONSI_, iterations
	

	solutionExist = 0

	
	if iterations == 0:	

		startNode = node(xs,ys,0,0,xg,yg,"null",e) # i,j,val,g,xg,yg
		startNode.backPointer = startNode.name

		OPEN.append(startNode)
		OPEN_.append(startNode.name)
	else:
		while len(INCONSI) >= 1:
			OPEN.append(INCONSI.pop(0))
			OPEN_.append(INCONSI_.pop(0))

		for i in range(len(OPEN)):
			OPEN[i].updateF(e)
	
		iterations = 0

	while(len(OPEN) != 0):
	
		min = 1000
		index = -1	

		for i in range(len(OPEN)):
			if OPEN[i].f < min:
				min = OPEN[i].f
				index = i	
		
		if index == -1:
			break		
			
	
		OPEN[index].v = OPEN[index].g

		if OPEN[index].name == "s"+str(xg)+" "+str(yg):
			solutionExist = 1			
			print "Number of computations =",iterations
			break
		

		i = OPEN[index].i
		j = OPEN[index].j
		g = OPEN[index].g	
		preName = OPEN[index].name

		#EXPANSION

		if i+1 < len(Map) and i+1 >= 0: 
			s = "s"+ str(i+1) +" "+ str(j)
			op = InOpen(s)
			clo = InClosed(s)

			if clo == -1 and op == -1:				
				OPEN.append(node(i+1,j,Map[i+1][j],g,xg,yg,preName,e)) #DOWN
				OPEN_.append(s)
			elif op != -1:
				OPEN[op].update(Map[i+1][j],g,preName,e)
			elif clo != -1:
				INCONSI.append(CLOSED.pop(clo))
				INCONSI_.append(CLOSED_.pop(clo))
				
				
		if i-1 < len(Map) and i-1 >= 0:
			s = "s"+ str(i-1) +" "+ str(j)
			op = InOpen(s)
			clo = InClosed(s)

			if clo == -1 and op == -1:
				OPEN.append(node(i-1,j,Map[i-1][j],g,xg,yg,preName,e)) #UP
				OPEN_.append(s)
			elif op != -1:
				OPEN[op].update(Map[i-1][j],g,preName,e)
			elif clo != -1:
				INCONSI.append(CLOSED.pop(clo))
				INCONSI_.append(CLOSED_.pop(clo))
				
		if j-1 < len(Map[0]) and j-1 >= 0:	
			s = "s"+ str(i) +" "+ str(j-1)
			op = InOpen(s)
			clo = InClosed(s)		

			if clo == -1 and op == -1:					
				OPEN.append(node(i,j-1,Map[i][j-1],g,xg,yg,preName,e)) #LEFT
				OPEN_.append(s)
			elif op != -1:
				OPEN[op].update(Map[i][j-1],g,preName,e)
			elif clo != -1:
				INCONSI.append(CLOSED.pop(clo))
				INCONSI_.append(CLOSED_.pop(clo))
				
		if j+1 < len(Map[0]) and j+1 >= 0:
			s = "s"+ str(i) +" "+ str(j+1)
			op = InOpen(s)
			clo = InClosed(s)

			if clo == -1 and op == -1:		
				OPEN.append(node(i,j+1,Map[i][j+1],g,xg,yg,preName,e)) #RIGHT
				OPEN_.append(s)	
			elif op != -1:
				OPEN[op].update(Map[i][j+1],g,preName,e)
			elif clo != -1:
				INCONSI.append(CLOSED.pop(clo))
				INCONSI_.append(CLOSED_.pop(clo))
			

		OPEN[index].expanded = 1

		#EXPANDED NODE IN CLOSED
		CLOSED_.append(OPEN_.pop(index))		
		CLOSED.append(OPEN.pop(index))
		
		iterations = iterations + 1
	
	if solutionExist:
		print "open =",OPEN_
		print "inconsi =",INCONSI_
		return getSolution(xs,ys,index)
	print "No solution"
	return solutionExist

def actuate():
	global BotX, BotY, robotYaw, Map
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)				
		
	e = 2
	while e >= 1:	
		plotGrid(ARAStar(10,0,10,9,e))
		e = e - 1

	while not rospy.is_shutdown():
		"""linearV, angularV = evaluateGradientVector(BotX,BotY)		
		ppidOmega.required = angularV
		angularV = ppidOmega.pidControl(robotYaw)
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
		"""
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





