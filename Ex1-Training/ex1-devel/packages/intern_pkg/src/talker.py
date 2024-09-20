#!/usr/bin/env python3
# Adapted from Prof Robinette's EECE5560 class 
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
class Talker:
	def __init__(self):
		self.pub = rospy.Publisher('chatter', String, queue_size = 10)
		
	def talker(self): 
		hello_str = 'Hello World: ' + str(rospy.get_time())
		rospy.loginfo(hello_str)
		self.pub.publish(hello_str)
		
if __name__ == '__main__':
	try:
		rospy.init_node('talker', anonymous=True) 
		t = Talker() 
		rate = rospy.Rate(1) 
		while not rospy.is_shutdown(): 
			t.talker() 
			rate.sleep()

	except rospy.ROSInterruptException:
		pass