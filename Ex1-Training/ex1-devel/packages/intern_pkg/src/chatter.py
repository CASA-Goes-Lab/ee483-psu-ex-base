#!/usr/bin/env python3
# Adapted from Prof Robinette's EECE5560 class 
import rospy
from std_msgs.msg import String

class Listener:
	def __init__(self):
		rospy.Subscriber("chatter", String, self.callback)
		if rospy.has_param("mode"): # Check if parameter "mode" exists first
			self.foo = rospy.get_param("mode") #Get the values of "mode" and store it in self.foo
		else:
			self.foo = "Portuguese" # If there is no parameter "mode" we need an alternate plan
										
	def callback(self, msg): 
		if rospy.has_param("mode"): # Check if parameter "mode" exists first
			self.foo = rospy.get_param("mode") #Get the values of "mode" and store it in self.foo
			rospy.loginfo(self.foo)
		else:
			self.foo = "Portuguese" # If there is no parameter "mode" we need an alternate plan
		if self.foo == "Portuguese":
			rospy.loginfo(rospy.get_caller_id() + ": Eu escutei %s", msg.data) # This will print the message received
		else:
			rospy.loginfo(rospy.get_caller_id() + ": I heard %s", msg.data) # This will print the message received
			
if __name__ == '__main__':
	try:
		rospy.init_node('listener', anonymous=True) 
		Listener() 
		rospy.spin()
	except rospy.ROSInterruptException:
		pass