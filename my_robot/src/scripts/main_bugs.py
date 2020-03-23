#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from math import pi, atan2, sqrt, cos, time 
import matplotlib.pyplot as plt

#________________________________________Global Variables__________________________________#
robotYaw = prevYaw = BotX = BotY = angularV = linearV  = 0
stateFlag = 0
distOmin = distGmin = float('inf')
freq = 10
distObstacle = []
hitPoint = [0,0]
leavePoint = [0,0]
distFBoundary = 0
t1 = t2 = 0
XX = []
YY = []
#_________________________________________Classes__________________________________________#
class PID:
	kp = 0
	kd = 0
	ki = 0
	required = 0 
	error = 0 
	prevError = 0
	derivative = 0
	integral = 0
	output = 0
	minControl = 0
	maxControl = 0

	def __init__(self,kp,kd,ki,required,minControl,maxControl):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.required = required
		self.maxControl = maxControl
		self.minControl = minControl

	def pidControl(self,actual):
		 
		self.error = self.required - actual
		self.derivative = (self.error - self.prevError)*freq
		self.integral = self.integral + self.error
		self.prevError = self.error
		self.output = self.kp*self.error + self.kd*self.derivative + self.ki*self.integral
		
		if self.output > self.maxControl:
			self.output = self.maxControl
		elif self.output < self.minControl:
			self.output = self.minControl
		return self.output

ppidOmega = PID(0.05,0,0,179,-2.84,2.84) 
ppidFollowB = PID(0.01,0.1,0,0,-2.84,2.84)
#_______________________________________________Functions_______________________________________#

def initSystem():
	rospy.init_node('my_robot',anonymous='True')	

	rospy.Subscriber("/odom",Odometry,getXY)
	rospy.Subscriber("/imu",Imu,getYaw)
	rospy.Subscriber("/scan",LaserScan,getLaserData)

#______________________________________________Callbacks________________________________________#	
def getLaserData(laserData):
	global distObstacle	
	distObstacle = laserData.ranges
	

	
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
	
	robotYaw = (atan2(2*(qz*qw + qy*qx),1- 2*(qz**2 + qy**2))*180.0/pi) 
	
#_____________________________________________Basics______________________________________________
	

def pointDistance(x1,y1,x2,y2):
	return sqrt((x1-x2)**2 + (y1-y2)**2)

def calculateHeading(x1,y1,x2,y2):
	angle = atan2((y2-y1),(x2-x1))
	return angle*180.0/pi	

	
def getReqYaw():
	global angularV
	if abs(angularV) <= 0.001:
		ppidOmega.required = input("Enter yaw=")

def getGoal():
	global x1,y1,x2,y2
	x1 = input("Enter a start position X: ")
	y1 = input("Enter a start position Y: ")
	x2 = input("Enter goal position X: ")	
        y2 = input("Enter goal position Y: ")
	
	ppidOmega.required = calculateHeading(x1,y1,x2,y2) 

def checkObstacle():
	global distObstacle
	if len(distObstacle) == 0:
		return 0

	for i in range(0,45,1):
		if distObstacle[i] < 0.5: 
			return -1
	
	for i in range(315,359,1):
		if distObstacle[i] < 0.5: 
			return 1
	return 0

def followBoundary(safeDistance, mode):
	global distObstacle, angularV, linearV, distOmin
	
	ppidFollowB.required = safeDistance
	if len(distObstacle) > 0:
		if mode == "clockWise":
			distOmin = min(distObstacle[180:359])
			angularV = ppidFollowB.pidControl(distOmin*100)
		else:
			distOmin = min(distObstacle[0:180])
			angularV = -ppidFollowB.pidControl(distOmin*100)

def bug1(x1, y1, x2, y2, vel):
	global stateFlag, robotYaw, linearV, angularV, hitPoint, leavePoint, distObstacle, distGmin
	global distFBoundary, t1,t2, BotX, BotY
	if stateFlag == 0:
		print("Motion to Goal")
		angleGoal = calculateHeading(BotX,BotY,x2,y2)
		ppidOmega.required = angleGoal 
		angularV = ppidOmega.pidControl(robotYaw)
		linearV = vel

		if checkObstacle():
			print("Hit Point!!")
			stateFlag = 1
			ppidOmega.required = robotYaw + 90.0
			distFBoundary = 20

		if pointDistance(BotX,BotY,x2,y2) < 5:
			angularV = 0
			linearV = 0
			print("Goal Reached")
			stateFlag = -1
			return 0

	elif stateFlag == 1:		
		angularV = ppidOmega.pidControl(robotYaw)
		if abs(angularV) < 0.001:
			stateFlag = 2
			linearV = vel
			hitPoint[0]= BotX
			hitPoint[1]= BotY
			t1 = time()
			print("Hit Point!!")
			
	
	elif stateFlag == 2:
		print("Boundary Follow")
		followBoundary(distFBoundary,"clockWise")
		distGoal = pointDistance(BotX,BotY,x2,y2) 
		if distGoal < distGmin:
			distGmin = distGoal
			leavePoint[0] = BotX
			leavePoint[1] = BotY
			print("Checking for Optimal Leave Point")
			
		if (pointDistance(BotX,BotY,hitPoint[0],hitPoint[1]) < 30 and time()-t1>10):
			stateFlag = 3
			print("Hit Point reached again")
	
	elif stateFlag == 3:		
		
		followBoundary(distFBoundary,"clockWise")
		if (pointDistance(BotX,BotY,leavePoint[0],leavePoint[1])) < 30:
			stateFlag = 4
			print("I am free!!")
			angleGoal = calculateHeading(BotX,BotY,x2,y2)
			ppidOmega.required = angleGoal 

	elif stateFlag == 4:
		angularV = ppidOmega.pidControl(robotYaw)
		if abs(angularV) < 0.001:
			stateFlag = 0

	return 1
	
		
	
		
				
	
		
		

			
	
	
	
	
#max Linear vel = 0.22		
def actuate():
	global pub, linearV, angularV, XX, YY
		
	pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
	rate = rospy.Rate(freq)
	
	while not rospy.is_shutdown():
		if(bug1(0,0,600,100,0.12)):
			XX.append(BotX)
			YY.append(BotY)
		else:
			pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
			plt.xlabel('X')
			plt.ylabel('Y')			
			plt.plot(XX,YY,label='Path')
			plt.scatter(0,0,label='Start')
			plt.scatter(600,100,label='Goal')
			plt.legend()
			plt.show()
			
		pub.publish(Twist(Vector3(linearV,0,0),Vector3(0,0,angularV)))
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()





           	

	





