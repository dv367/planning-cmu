
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


freq = 100
BotX = 0
BotY = 0
printFlag = 0

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

circle1 = plt.Circle((100,100), 100, color='g', label='obstacles',fill=True)#, linestyle=':')
circle2 = plt.Circle((200,-100), 100, color='g', label='obstacles',fill=True)#, linestyle=':')
circle3 = plt.Circle((330,100), 100, color='g', label='obstacles',fill=True)#, linestyle='-')
circle4 = plt.Circle((0,0), 500, color='r', label='obstacles',fill=False, linestyle='-')


X = [0,448]
Y = [-100,160]

ax1.set_aspect(1)

ax1.add_artist(circle1)
ax1.add_artist(circle2)
ax1.add_artist(circle3)
ax1.add_artist(circle4)

label1 = ax1.annotate("Ob1", xy=(100, 100), fontsize=10)
label2 = ax1.annotate("Ob2", xy=(200,-100), fontsize=10)
label3 = ax1.annotate("Ob3", xy=(330, 100), fontsize=10)

plt.ylim(-800,800)
plt.xlim(-800,800)

def initSystem():
	rospy.init_node('envPlot',anonymous='True')	
	rospy.Subscriber("/odom",Odometry,getXY)

def getXY(PoseWithCovariance):	
	global BotX, BotY, printFlag # My Axis
	BotX =  (PoseWithCovariance.pose.pose.position.x - 0)*100 #cm
	BotY =  (PoseWithCovariance.pose.pose.position.y - 0)*100  #cm
	printFlag = 1


def actuate():
	global BotX, BotY
		
	rate = rospy.Rate(freq)
	plt.scatter(X,Y,color='r')
	plt.ion()

	
		

	while not rospy.is_shutdown():
		
		if printFlag:		
			plt.xlabel('X-axis(cm)')
	    		plt.ylabel('Y-axis(cm)')
		
			plt.scatter(BotX,BotY)
			plt.draw()
			plt.pause(0.05)
	
		rate.sleep()		

if __name__ == '__main__':
           initSystem()
	   actuate()
	   rospy.spin()
