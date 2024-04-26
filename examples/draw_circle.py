#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("draw_circle")
    rospy.loginfo("Node started")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        msg = Twist()

        # No need to fill out everything, just the values we care about
        msg.linear.x = 2
        msg.angular.z = 1
        pub.publish(msg)

        rate.sleep()
