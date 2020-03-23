#!/usr/bin/env python

#standard imports
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty

from myClasses import PID
from basics import dist, distNP, angle, scale360, scale180
from math import pi, atan2, sqrt, cos, fabs, atan, sin, radians 
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

#max Linear vel = 0.22	Angular = 2.84

#________________________________________Global Variables__________________________________#
robotYaw = BotX = BotY =  0
freq = 10
laserData = [100]

#_________________________________________Classes__________________________________________#


ppidOmega = PID(2.0,0,0,pi/2,-2.84,2.84) 
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
	
	zeta = 0.22/dist(0,0,xg,yg)

	xx = np.linspace(x, xg, 30)
	yy = np.linspace(y, yg, 30)

	X, Y = np.meshgrid(xx, yy)
	Z = zeta*distNP(X, Y, xg,yg)

	#fig= plt.figure()
	#ax = plt.axes(projection='3d')
	#ax.plot_surface(X, Y, Z, rstride=1, cstride=1,cmap='viridis', edgecolor='none')	
	#fig, ax = plt.subplots(figsize=(6,6))
	#ax.contourf(X,Y,Z)	
	#ax.set_xlabel('x')
	#ax.set_ylabel('y')
	#ax.set_zlabel('z');
	#ax.set_title('surface');
	#plt.show()

def contourRepulsive(x,y,xg,yg,xo,yo):
	
	tol = 100
	r = 10 #cm
	eta = 22.0/(((1/tol) - (1/350.0))*(350.0)**2)
	

	xx = np.linspace(x, xg, 30)
	yy = np.linspace(y, yg, 30)
	
			

	X, Y = np.meshgrid(xx, yy)
	Z = distNP(X, Y, xo,yo)
	
	for i in range(0,len(Z)):
		for j in range(0,len(Z)):
			if Z[i][j] < tol and Z[i][j] > 0:
				Z[i][j] = eta*((1/tol) - (1/Z[i][j]))*(1/Z[i][j]**2)
			else:
				Z[i][j] = 0
	#fig= plt.figure()	
	#ax = plt.axes(projection='3d')
	#ax.contour3D(X, Y, Z, 50, cmap='binary')	
	#ax.plot_surface(X, Y, Z, rstride=1, cstride=1,cmap='viridis', edgecolor='none')	
	
	#fig, ax = plt.subplots(figsize=(6,6))	
	#ax.contourf(X,Y,Z)	
	#ax.set_xlabel('x')
	#ax.set_ylabel('y')
	#ax.set_zlabel('z');
	#ax.set_title('surface');
	#plt.show()


	

def attractivePotential(x,y,xg,yg):
	
	zeta = 0.22/dist(0,0,xg,yg)
	
	
	theta = angle(x,y,xg,yg)
	magnitudeA = zeta*dist(x,y,xg,yg)

	return magnitudeA, theta	

def repulsivePotential(x,y,xg,yg):
	global laserData

	tol = 1.0
	eta = 0.22/(((1/tol) - (1/3.5))*(3.5)**2)
	
	dq = min(laserData)
	
	phi = 0
	magnitudeR = 0
	
	if dq < tol:
		for i in range(0,len(laserData)):
			if dq == laserData[i]:
				phi = i*pi/180.0 - pi + robotYaw
				
					
		magnitudeR = fabs(eta*((1/tol) - (1/dq))*(1/dq**2))
	
	if magnitudeR > 0.22:
		magnitudeR = 0.22
	return magnitudeR, phi	
	
def quadraticPotential(x,y,xg,yg):

	

	mAtt, theta = attractivePotential(x,y,xg,yg)
	mRep, phi = repulsivePotential(x,y,xg,yg)


	vAtt = [mAtt*cos(theta),mAtt*sin(theta)]
	vRep = [mRep*cos(phi),mRep*sin(phi)]
	vFinal = [vAtt[0]+vRep[0],vAtt[1]+vRep[1]]
	
	magFinal = sqrt(vFinal[0]**2 + vFinal[1]**2)
	angFinal = atan2(vFinal[1],vFinal[0])
	
	if magFinal > 0.22:
		magFinal = 0.22
	
	if fabs(phi - angle(x,y,xg,yg)) <= radians(185) and fabs(phi - angle(x,y,xg,yg)) >= radians(175):
		print("________Local minima found_________")
		print("+ve vector = ",mAtt,theta*180/pi," -ve vector = ",mRep,phi*180/pi)
		print("resultant vector = ",magFinal,angFinal*180/pi)	
	else:
		print("Cool")

	
	


	return magFinal, angFinal


def actuate():
	global BotX, BotY, robotYaw
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)

	while not rospy.is_shutdown():
		linearV, angularV = quadraticPotential(BotX,BotY,500,500)
		
		ppidOmega.required = angularV
		angularV = ppidOmega.pidControl(robotYaw)	
		
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))

		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





