#!/usr/bin/env python

#standard imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty

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

#_________________________________________Classes__________________________________________#


ppidOmega = PID(2.0,0,0,pi/2,-2.84,2.84) #orientation control
ppidFollowB = PID(0.01,0.1,0,0,-2.84,2.84)
#_______________________________________________Functions_______________________________________#

def initSystem():
	rospy.init_node('my_robot',anonymous='True')	

	rospy.Subscriber("/odom",Odometry,getXY)
	rospy.Subscriber("/imu",Imu,getYaw)
	rospy.Subscriber("/scan",LaserScan,getLaserData)

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


def contourAttractive(x,y,xg,yg):	
	return X,Y,Z
	

def contourRepulsive(x,y,xg,yg,xo,yo):	
	return X,Y,Z	

def navigationFunc(xg,yg,k,ob): #ob[[x,y,r]]

	x, y = sp.symbols('x y', real=True)
		
	B0 = -(distSP(x,y,ob[0][0],ob[0][1])**2.0) + ob[0][2]**2.0
	B1 =    distSP(x,y,ob[1][0],ob[1][1])**2.0 - ob[1][2]**2.0
	B2 =    distSP(x,y,ob[2][0],ob[2][1])**2.0 - ob[2][2]**2.0
	#B3 =    distSP(x,y,ob[3][0],ob[3][1])**2.0 - ob[3][2]**2.0
	#B4 =    distSP(x,y,ob[4][0],ob[4][1])**2.0 - ob[4][2]**2.0			
		
	Bi = B0*B1*B2#*B3*B4

	
	yk = (distSP(xg,yg,x,y))**(2.0*k)

	f = 0.22*(yk/(yk+Bi))**(1.0/k)	
	
	fi = -sp.diff(f,x)
	fj = -sp.diff(f,y)

	return fi,fj,f


	print theta,mag
	return mag, theta
	
def evaluateGradientVector(a,b,fi,fj,f):

	global BotX,BotY
	x, y = sp.symbols('x y', real=True)

	theta = sp.atan2(fj,fi)
	
	
	
	thetaFinal = theta.evalf(subs={x: a, y: b})
	magFinal = f.evalf(subs={x: a, y: b})
	
	print thetaFinal*180/pi, magFinal

	return  magFinal, thetaFinal

def actuate():
	global BotX, BotY, robotYaw
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)				
				
	QO = [[0,0,500],
	      [100,100,100],
	      [200,-100,100]]

	fi, fj, f = navigationFunc(400,0,8,QO)
	
	while not rospy.is_shutdown():
		linearV, angularV = evaluateGradientVector(BotX,BotY,fi,fj,f)		
		
		ppidOmega.required = angularV
		angularV = ppidOmega.pidControl(robotYaw)
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
		
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





