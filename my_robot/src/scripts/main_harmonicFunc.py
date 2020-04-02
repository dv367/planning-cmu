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


def panel(x,y,xi,yi,L,lambDa_o):
	
	k = lambDa_o/(2.0*pi)
	
	normalV = k*(1.0/(x-xi))*((atan2(y-yi+L,x-xi))-(atan2(y-yi-L,x-xi)))
	tangentialV = k*log(((x-xi)**2+(y+-yi+L)**2)/((x-xi)**2+(y-yi-L)**2))
	
	print normalV,tangentialV  
	
def sinkingGoal(x,y,xg,yg):
	
	lambDa_g = 0.22*dist*2*pi
	k = -lambDa_g/(2.0*pi)
	
	distG = dist(x,y,xg,yg)**(2.0)

	vGx = k*(x-xg)/(distG)
	vGy = k*(y-yg)/(distG)

	return vGx,vGy	

def uniformField(x,y,xg,yg):
	
	U = 0.22
	alpha = atan2(y-yg,x-xg)

	vUx = U(-sin(alpha))
	vUy = U(cos(alpha))
	
	return vUx,vUy	
	
	
def harmonicFunc(x,y,xg,yg,lambDa_g):
	
	vGX, vGY = sinkingGoal(x,y,xg,yg,lambDa_g)
		
	return v,theta
	



def actuate():
	global BotX, BotY, robotYaw
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)				
				

	panel(50,50,0,0,100,1)	
	while not rospy.is_shutdown():
		linearV, angularV = harmonicFunc(BotX,BotY,-200,-200,10)		
		ppidOmega.required = angularV
		angularV = ppidOmega.pidControl(robotYaw)
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





