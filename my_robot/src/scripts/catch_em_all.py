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
from copy import deepcopy, copy
import random
#max Linear vel = 0.22	Angular = 2.84

#________________________________________Global Variables__________________________________#
robotYaw = BotX = BotY = 0
freq = 100
laserData = [100]
robotX = robotY = targetX = targetY = -1
prevTargetX = prevTargetY = -1
prevRobotX = []
prevRobotY = []
artificalObstaclesX = []
artificalObstaclesY = []
prev_artificalX = []
prev_artificalY = []
ignoreTarget = 0

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
#0 [_ _ _ # _ _ _ _ _ _] 
#1 [_ _ _ # _ _ _ # _ _]
#2 [_ _ # # _ _ _ # _ _]
#3 [_ _ # # _ _ G # _ _]
#4 [_ _ # # # _ _ _ _ _]
#5 [_ _ # # # # _ _ _ _]
#6 [_ _ _ _ # # _ _ _ _]
#7 [_ _ _ _ # # _ _ _ _]
#8 [_ _ _ _ _ # _ # _ _]
#9 [_ _ _ _ _ # _ _ _ _]
#10[S _ _ _ _ # _ _ _ _] Start = (10,0) Goal(3,6)
Map = [				  
	[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,0,0,1,0,1,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,1,1,1,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,0,0,1,0,1,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,0,0,1,0,1,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,1,1,1,1,1,0,1,0,0,1,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,1,1,1,1,0,1,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,1,1,1,1,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0]
]


OPEN = []
OPEN_ = []
CLOSED = []
CLOSED_ = []

Point_i = [1,-1,0,0,-1,-1,1,1] #8 point connectivity for catcher
Point_j = [0,0,1,-1,-1,1,1,-1] #8 point connectivity for catcher

Point_xg = [1,-1,0,0] #4 point connectivity for target
Point_yg = [0,0,1,-1] #4 point connectivity for target

def smallest_F(data,minimum):
	
	index = -1

	for i in range(len(data)):
		if data[i].f < minimum:
			minimum = data[i].f 
			index = i

	return index

def largestInArray(data,maximum):
	
	index = -1
	for i in range(len(data)):
		if data[i] > maximum:
			maximum = data[i]
			index = i

	return index

def smallestInArray(data,minimum):
	
	index = -1
	for i in range(len(data)):
		if data[i] < minimum:
			minimum = data[i]
			index = i

	return index
		
def notIn(s,data):
	
	for i in range(len(data)):
		if data[i] == s:
			return i
	return -1

def extractIndicies(s):
	i = ""
	j = ""
	space = []
	for k in range(len(s)):
		if s[k] == ' ':
			space.append(k)
	i = i + s[space[0]:space[1]]
	j = j + s[space[1]:]

	return int(i),int(j)

def lookForFreeSpace(A,B,X,Y,Map):
	global Point_i, Point_j

	x = []
	y = []
	#print "here1"
	for w in range(len(X)):
	#	print "here2"
		for k in range(len(Point_i)):
	#		print "here3"
			m = Point_i[k] + X[w]
			n = Point_j[k] + Y[w]
			
			if m < len(Map) and m >= 0 and m!=A and n!= B and n < len(Map[0]) and n >= 0:
	#			print "here4"				
				if Map[m][n] == 0:
					x.append(m)
					y.append(n)

	if len(x) == 0:
		return -1,-1
	minimum = dist(A,B,x[0],y[0])
	m = x[0]
	n = y[0]	
	for i in range(len(x)):
		if dist(A,B,x[i],y[i]) < minimum:
			minimum = dist(A,B,x[i],y[i])
			m = x[i]
			n = y[i]
	return x[i],y[i]
			
	

def  computePath(xs,ys,xg,yg,e,Map):

#______________________________________INITIALIZATIONS_________________________________________________________	
	global OPEN, CLOSED	
	global OPEN_, CLOSED_
	global Point_i, Point_j, Point_xg, Point_yg
	global robotX, robotY, targetX, targetY
	global prevTargetX, prevTargetY, prevRobotX, prevRobotY
	global artificalObstaclesX, artificalObstaclesY
	global prev_artificalX, prev_artificalY
	global ignoreTarget

	OPEN = []
	OPEN_ = []
	CLOSED = []
	CLOSED_ = []

	startNode = node(xs,ys,Map[xs][ys],0,xg,yg,"null",e)#i,j,val,g,xg,yg,backP,e
	startNode.backpointer = startNode.name
	OPEN.append(startNode)
	OPEN_.append(startNode.name)

	goalExpanded = 0 
	expansions = 0
	solutionExist = 0
	goalBackPointer = "null"

	targetMoved = 0
	index = -1
	blocked = 1
	
	neigbhours = 0
	obstacles = 0

	temp = deepcopy(Map)
	temp[xs][ys] = 2
	temp[xg][yg] = 3
#_________________________________________________________________________________________________
	while(len(OPEN)!=0 and goalExpanded == 0):

		

		index = smallest_F(OPEN,1000)    #finding node with smallest f value

		
		i = OPEN[index].i		# node INFO
		j = OPEN[index].j
		g = OPEN[index].g	
		name = OPEN[index].name
			
		CLOSED.append(OPEN.pop(index))
		CLOSED_.append(OPEN_.pop(index))	#pushing that node into CLOSED to expand its neigbhours
		
		if ignoreTarget == 0:
			if targetMoved:			#if target has moved replan everything

				temp[xs][ys] = 0
				temp[i][j] = 4	
			
				robotX = i
				robotY = j
			
				return "target moved"	

		


		if i != xs and j != ys:			#different color of start node
			temp[i][j] = 4
		
		plotGrid(temp,"draw")

		for k in range(len(Point_i)): #exploring node's neighbours
			m = Point_i[k] + i
			n = Point_j[k] + j
			
		
			if m < len(Map) and m >= 0 and n < len(Map[0]) and n >= 0:
				s = "s" + " " + str(m) + " " + str(n)
					
				cost = 1				
				if m!=0 and n!=0:
					cost = 1.4			#diagonal cost = 1.4

				openIndex = notIn(s,OPEN_)		# node already in open or closed ?
				closedIndex = notIn(s,CLOSED_) 
				
				if openIndex != -1:
					OPEN[openIndex].checkG(Map[m][n],g,name,e)	# if already in open see its best backpointer			
				
				elif closedIndex == -1:
					OPEN.append(node(m,n,Map[m][n],g+cost,xg,yg,name,e)) #if not in closed and open, create node and append in open
					OPEN_.append(s)
					expansions = expansions + 1


				if s == "s" + " " + str(xg) + " " + str(yg): #check if goal is found
					goalExpanded = 1			
					solutionExist = 1
					goalBackPointer = name

				if Map[m][n] == 0:
					blocked = 0
					temp[m][n] = 4
				
		
		if goalExpanded == 0 and ignoreTarget == 0:		
			
			while targetMoved == 0:			
				new_xg = xg + random.choice(Point_xg)
				new_yg = yg + random.choice(Point_yg)

				if new_xg < len(Map) and new_xg >= 0 and new_yg < len(Map[0]) and new_yg >= 0:
					if Map[new_xg][new_yg] == 0:
						temp[xg][yg] = 0
					
						prevTargetX = xg
						prevTargetY = yg

						targetX = new_xg
						targetY = new_yg

						temp[new_xg][new_yg] = 3
						targetMoved = 1

		elif goalExpanded == 1 and ignoreTarget == 1:
			goalExpanded = 0
			ignoreTarget = 0
			targetMoved = 1
			print "IgnoreTarget complete"

			

			artificalObstaclesX = copy(prev_artificalX)
			artificalObstaclesY = copy(prev_artificalY)

			artificalObstaclesX.append(robotX)
			artificalObstaclesY.append(robotY)

			robotX = targetX
			robotY = targetY

			for a in range(len(prev_artificalX)):
				Map[prev_artificalX[a]][prev_artificalY[a]] = 1

			targetX = prevTargetX
			targetY = prevTargetY
			targetMoved = 1

			prev_artificalX = []
			prev_artificalY = []
			
			for i in range(len(Map)):
				print (Map[i])
			
			return "target moved"
				
		
		
		if ignoreTarget == 0:
			prevRobotX.append(i)
			prevRobotY.append(j)	
			if len(prevRobotX) >= 3:
				for k in range(len(prevRobotX)-2):
					if prevRobotX[k+0] == prevRobotX[k+2] and prevRobotY[k+0] == prevRobotY[k+2]:
					
						Map[prevRobotX[k]][prevRobotY[k]] = 1

						artificalObstaclesX.append(prevRobotX[k])
						artificalObstaclesY.append(prevRobotY[k])
					
						prevRobotX = []
						prevRobotY = []					
					
		if blocked == 1 and ignoreTarget == 0:

			print "IgnoreTarget init"
			xg_,yg_ = lookForFreeSpace(i,j,artificalObstaclesX,artificalObstaclesY,Map)	
						
			if xg_ == -1 and yg_ == -1:
				return "null"
			
			ignoreTarget = 1		

			targetX = xg_
			targetY = yg_
	
			robotX = i
			robotY = j

			artificalObstaclesX.append(i)
			artificalObstaclesY.append(j)
			prev_artificalX = copy(artificalObstaclesX)
			prev_artificalY = copy(artificalObstaclesY)
			
			for a in range(len(artificalObstaclesX)):
				Map[artificalObstaclesX[a]][artificalObstaclesY[a]] = 0

			
			artificalObstaclesX = []
			artificalObstaclesY = []

			return "target moved"
		


			
			
			
	if solutionExist:
		print "Goal Found in Expansions = ",expansions
		return goalBackPointer
	else:
		print "sorry no solution"
		return "null" 
		
		
							
def getSolution(xs,ys,xg,yg,goalBackPointer,Map):
	
	global CLOSED_, CLOSED

	solved = deepcopy(Map)
	solution = []

	goal = "s"+" "+str(xg)+" "+str(yg)
	start = "s"+" "+str(xs)+" "+str(ys)

	solution.append(goal)	
	i, j = extractIndicies(goal)
	solved[i][j] = 3

	backP = goalBackPointer
	while(backP != "s"+" "+str(xs)+" "+str(ys)):
		
		solution.append(backP)
		i, j = extractIndicies(backP)
		solved[i][j] = 3	

		index = notIn(backP,CLOSED_)
		backP = CLOSED[index].backpointer
		
	solution.append(start)
	i, j = extractIndicies(start)
	solved[i][j] = 2
	
	return solved		
			
		
def A_Star(e,Map):
	
	global targetX, targetY, robotX, robotY
	targetX = 0
	targetY = 70
	robotX = 10
	robotY = 0

	targetCaught = 0

	while targetCaught == 0:
 
		goalBackPointer = computePath(robotX,robotY,targetX,targetY,e,Map)		
		if goalBackPointer != "target moved" and goalBackPointer != "null":
			targetCaught = 1
			solution = getSolution(robotX,robotY,targetX,targetY,goalBackPointer,Map)
			plotGrid(solution,"plot")	
		if goalBackPointer == "null":
			targetCaught = -1
			print "NO SOLUTION"
		
		

		

			
	

def actuate():
	global BotX, BotY, robotYaw, Map
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)				
	
	
	
	A_Star(1,Map)
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
