#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist

prev_x = 0
def call_set_pen_service(r,g,b, width, off):
    try: 
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        response = set_pen(r,g,b,width, off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def pose_callback(pose: Pose):
    cmd = Twist()
    if pose.x > 9 or pose.x < 2 or pose.y > 9 or pose.y < 2:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.4
    else:
        cmd.linear.x = 5
        cmd.angular.z = 0

    pub.publish(cmd)
    
    global prev_x

    if pose.x >= 5.5 and prev_x <= 5.5:
        prev_x = pose.x
        rospy.loginfo("Set colour to red!")
        call_set_pen_service(255, 0, 0, 3, 0)

    elif pose.x <= 5.5 and prev_x >= 5.5:
        prev_x = pose.x
        rospy.loginfo("Set colour to green!")
        call_set_pen_service(0, 255, 0, 3, 0)

if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    rospy.wait_for_service("/turtle1/set_pen")
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Listener node has been started")
    rospy.spin()
