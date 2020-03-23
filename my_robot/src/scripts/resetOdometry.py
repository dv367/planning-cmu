import rospy
from std_msgs.msg import Empty
import time



if __name__ == '__main__':
	startTime = time.time()
	rospy.init_node('resetOdom')
	resetOdom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
	stri = "."	
	while time.time() - startTime < 1.0:
		resetOdom.publish(Empty())
		
	print "__Odometry Reset__"
