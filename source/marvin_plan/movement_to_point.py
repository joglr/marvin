#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
from math import atan2

x = 0.0
y = 0.0
theta = 0.0

def new_odom_callback(msg:Odometry):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

if __name__ == "__main__":
    rospy.init_node("move_to_point")
    rospy.loginfo("Node has been started")

    sub = rospy.Subscriber("/marvin/odom", Odometry, new_odom_callback)
    pub = rospy.Publisher("/marvin/cmd_vel", Twist, queue_size=5)

    speed = Twist()

    rate = rospy.Rate(4)

    goal = Point()
    goal.x = 5
    goal.y = 5
    
    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 1.0
        else:
            speed.linear.x = 2.0
            speed.angular.z = 0.0
        pub.publish(speed)

        rate.sleep()
    