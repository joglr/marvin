#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

#add _callback to indicate that this is a callback
def pose_callback(msg: Pose):
    rospy.loginfo(f"{msg.x:.3f},{msg.y:.3f}")

if __name__ == '__main__':
    rospy.init_node("turtle_pose_subscriber")

    rospy.loginfo("Node has been started")
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.spin()

