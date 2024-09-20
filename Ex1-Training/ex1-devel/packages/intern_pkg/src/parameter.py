#!/usr/bin/env python3

import rospy


if __name__=='__main__':
    try:
        rospy.init_node('conversion',anonymous=True)
        rate = rospy.Rate(1)
        rate.sleep()
        while rospy.has_param("mode"):
            rospy.sleep(4)
            rospy.loginfo("Changing parameter to Portuguese")
            rospy.set_param("mode", 'Portuguese')
            rospy.sleep(4)
            rospy.loginfo("Changing parameter to English")
            rospy.set_param("mode", 'English')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass