
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


freq = 100
BotX = 0
BotY = 0
printFlag = 0

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

circle1 = plt.Circle((100,500), 50, color='g', label='obstacles',fill=True)#, linestyle=':')
circle2 = plt.Circle((150,100), 50, color='g', label='obstacles',fill=True)#, linestyle=':')
circle3 = plt.Circle((250,350), 50, color='g', label='obstacles',fill=True)#, linestyle='-')
circle4 = plt.Circle((400,250), 50, color='g', label='obstacles',fill=True)#, linestyle=':')
circle5 = plt.Circle((500,100), 50, color='g', label='obstacles',fill=True)#, linestyle=':')
circle6 = plt.Circle((500,500), 50, color='g', label='obstacles',fill=True)#, linestyle='-')
circle7 = plt.Circle((700,500), 50, color='g', label='obstacles',fill=True)#, linestyle=':')

#circle4 = plt.Circle((0,0), 500, color='r', label='obstacles',fill=False, linestyle='-')

xs = 0
ys = 0
xg = 150
yg = 800

X1 = [xs,xs,xg,xg,xs]
Y1 = [ys,yg,yg,ys,ys]

ax1.set_aspect(1)

ax1.add_artist(circle1)
ax1.add_artist(circle2)
ax1.add_artist(circle3)
ax1.add_artist(circle4)
ax1.add_artist(circle5)
ax1.add_artist(circle6)
ax1.add_artist(circle7)


#label1 = ax1.annotate("Ob1", xy=(100, 100), fontsize=10)
#label2 = ax1.annotate("Ob2", xy=(200,-100), fontsize=10)
#label3 = ax1.annotate("Ob3", xy=(330, 100), fontsize=10)

plt.ylim(0,800)
plt.xlim(0,800)

def initSystem():
	rospy.init_node('envPlot',anonymous='True')	
	rospy.Subscriber("/odom",Odometry,getXY)

def getXY(PoseWithCovariance):	
	global BotX, BotY, printFlag # My Axis
	BotX =  (PoseWithCovariance.pose.pose.position.x - 0)*100 #cm
	BotY =  (PoseWithCovariance.pose.pose.position.y - 0)*100  #cm
	printFlag = 1


def actuate():
	global BotX, BotY, printFlag
		
	rate = rospy.Rate(freq)
	#plt.scatter(X,Y,color='r')
	plt.ion()

	printFlag = 0
		
	plt.plot(X1,Y1,label="window")
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
