
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

freq = 100
BotX = BotY = 0

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

circle1 = plt.Circle((100,130), 50, color='r', label='obstacles',fill=False, linestyle=':')
circle2 = plt.Circle((330,270), 50, color='r', label='obstacles',fill=False, linestyle=':')

ax1.set_aspect(1)

ax1.add_artist(circle1)
ax1.add_artist(circle2)


label1 = ax1.annotate("Ob1", xy=(100, 130), fontsize=10)
label2 = ax1.annotate("Ob2", xy=(330, 270), fontsize=10)

plt.ylim(-100,800)
plt.xlim(-100,800)

def initSystem():
	rospy.init_node('envPlot',anonymous='True')	
	rospy.Subscriber("/odom",Odometry,getXY)

def getXY(PoseWithCovariance):	
	global BotX, BotY # My Axis
	BotX =  (PoseWithCovariance.pose.pose.position.x - 0)*100 #cm
	BotY =  (PoseWithCovariance.pose.pose.position.y - 0)*100  #cm


def actuate():
	global BotX, BotY
		
	rate = rospy.Rate(freq)
	
	plt.ion()
	while not rospy.is_shutdown():
				
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
