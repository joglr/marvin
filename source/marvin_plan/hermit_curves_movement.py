#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point

def h1(t):
    return 2*t**3 - 3*t**2 + 1

def h2(t):
    return -2*t**3 + 3*t**2

def h3(t):
    return t**3 - 2*t**2 + t

def h4(t):
    return t**3 - t**2

def hx(t):
    return h1(t) * p0[0] + h2(t) * p1[0] + h3(t) * t0[0] + h4(t) *t1[0]

def hy(t):
    return h1(t) * p0[1] + h2(t) * p1[1] + h3(t) * t0[1] + h4(t) *t1[1]


def dh1(t):
    return 6*t**2 - 6*t

def dh2(t):
    return -6*t**2 + 6*t

def dh3(t):
    return 3*t**2 - 4*t + 1

def dh4(t):
    return 3*t**2 - 2*t

def dhx(t):
    return dh1(t) * p0[0] + dh2(t) * p1[0] + dh3(t) * t0[0] + dh4(t) * t1[0]

def dhy(t):
    return dh1(t) * p0[1] + dh2(t) * p1[1] + dh3(t) * t0[1] + dh4(t) * t1[1]


def linear_velocity(t):
    dx = dhx(t)
    dy = dhy(t)
    return 0.02 * math.sqrt(dx**2 + dy**2)

def angular_velocity(t, current_theta):
    target_theta = math.atan2(dhy(t), dhx(t))
    angle_diff = target_theta - current_theta
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    return 2.0 * angle_diff


def reached_end_point(x, y, end_point, threshold=0.3):
    return abs(x - end_point[0]) + abs(y - end_point[1]) < threshold

p0 = [0, 0]
p1 = [5, 5]
t0 = [1, 5]
t1 = [8, 4] 
dt = 0.01

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
    _, _, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == "__main__":
    rospy.init_node("hermite_curves_movement")
    rospy.loginfo("Node has been started")

    sub = rospy.Subscriber("/marvin/odom", Odometry, new_odom_callback)
    pub = rospy.Publisher("/marvin/cmd_vel", Twist, queue_size=5)

    rate = rospy.Rate(10)

    t = 0
    while not rospy.is_shutdown() and t <= 1:
        
        next_x, next_y = hx(t), hy(t)
        rospy.loginfo(f"Target point at t={t}: ({next_x}, {next_y})")

        while not (reached_end_point(x, y ,[next_x, next_y])):
            # inc_x = next_x - x
            # inc_y = next_y - y
            # angle_to_goal = math.atan2(inc_y, inc_x)
            # angle_diff = angle_to_goal - theta

            twist = Twist()
            twist.linear.x = linear_velocity(t)
            twist.angular.z = angular_velocity(t, theta)

            pub.publish(twist)
            rate.sleep()
        rospy.loginfo("t vaue: " + str(t))
        t += dt
     # Stop the robot when the final point is reached
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)



    
    