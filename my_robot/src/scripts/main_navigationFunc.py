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
x, y = sp.symbols('x y', real=True)
thetaF = f = x
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


def navigationFunc(xg,yg,k,ob): #ob[[x,y,r]]

	global thetaF, f

	x, y = sp.symbols('x y', real=True)
		
	B0 = -(distSP(x,y,ob[0][0],ob[0][1])**2.0) + ob[0][2]**2.0
	B1 =    distSP(x,y,ob[1][0],ob[1][1])**2.0 - ob[1][2]**2.0
	B2 =    distSP(x,y,ob[2][0],ob[2][1])**2.0 - ob[2][2]**2.0
	B3 =    distSP(x,y,ob[3][0],ob[3][1])**2.0 - ob[3][2]**2.0
	#B4 =    distSP(x,y,ob[4][0],ob[4][1])**2.0 - ob[4][2]**2.0			
		
	Bi = B0*B1*B2*B3#*B4
	yk = (distSP(x,y,xg,yg))**(2.0*k)

	phi = atan2(yg - ob[0][1],xg - ob[0][0])

	x_ = ob[0][2]*cos(phi+pi) + ob[0][0]
	y_ = ob[0][2]*sin(phi+pi) + ob[0][0]

	
	yk_Max = yk.evalf(subs={x: x_, y: y_})
	Bi_ = Bi.evalf(subs={x: x_, y: y_})

	lambDa = fabs(yk_Max/Bi_)*10**(-25.0)
	
	print yk.evalf(subs={x: 0, y: -100}), Bi.evalf(subs={x: 0, y: -100}) 

	f = 0.22*(yk/(yk+lambDa*Bi))**(1.0/k)	
	
	
	fi = -sp.diff(f,x)
	fj = -sp.diff(f,y)

	thetaF = sp.atan2(fj,fi)

	
def evaluateGradientVector(a,b):

	global thetaF, f

	x, y = sp.symbols('x y', real=True)

	
	thetaFinal = thetaF.evalf(subs={x: a, y: b})
	magFinal = 0.22#f.evalf(subs={x: a, y: b})
	
	if magFinal < 0.10 and magFinal > 0.0007:
		magFinal = 0.10
	
	
	print "angle = ",thetaFinal*180/pi," Mag = ", magFinal

	return  magFinal, thetaFinal

def actuate():
	global BotX, BotY, robotYaw
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)				
				
	QO = [[0.0,0.0,500.0],
	      [100.0,100.0,100.0],
	      [200.0,-100.0,100.0],
	      [330.0,70.0,100.0]]

	navigationFunc(448.0,160.0,10.0,QO)
	
	while not rospy.is_shutdown():
		linearV, angularV = evaluateGradientVector(BotX,BotY)		
		ppidOmega.required = angularV
		angularV = ppidOmega.pidControl(robotYaw)
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
		
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





