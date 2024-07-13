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

def hx(t, p0, p1, t0, t1):
    return h1(t) * p0[0] + h2(t) * p1[0] + h3(t) * t0[0] + h4(t) *t1[0]

def hy(t, p0, p1, t0, t1):
    return h1(t) * p0[1] + h2(t) * p1[1] + h3(t) * t0[1] + h4(t) *t1[1]

def dh1(t):
    return 6*t**2 - 6*t

def dh2(t):
    return -6*t**2 + 6*t

def dh3(t):
    return 3*t**2 - 4*t + 1

def dh4(t):
    return 3*t**2 - 2*t

def dhx(t, p0, p1, t0, t1):
    return dh1(t) * p0[0] + dh2(t) * p1[0] + dh3(t) * t0[0] + dh4(t) * t1[0]

def dhy(t, p0, p1, t0, t1):
    return dh1(t) * p0[1] + dh2(t) * p1[1] + dh3(t) * t0[1] + dh4(t) * t1[1]


def linear_velocity(t, p0, p1, t0, t1):
    dx = dhx(t, p0, p1, t0, t1)
    dy = dhy(t, p0, p1, t0, t1)
    return 0.03 * math.sqrt(dx**2 + dy**2) # 0.02

def angular_velocity(t, current_theta, p0, p1, t0, t1):
    target_theta = math.atan2(dhy(t, p0, p1, t0, t1), dhx(t, p0, p1, t0, t1))
    angle_diff = target_theta - current_theta
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    return 3 * angle_diff # 2.0

def reached_end_point(x, y, end_point, threshold=0.4):
    return abs(x - end_point[0]) + abs(y - end_point[1]) < threshold

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

    sub = rospy.Subscriber("/marvin/odom", Odometry, new_odom_callback) # /marvin/odom
    pub = rospy.Publisher("/marvin/cmd_vel", Twist, queue_size=5) # /marvin/cmd_vel

    rate = rospy.Rate(50)

    waypoints = [
        ([0, 0], [6, -2]),  # (position, tangent)
        ([4, 4], [-2, 4]),  # (position, tangent)
        ([6, 8], [12, 4])   # (position, tangent)
    ]

    for i in range(len(waypoints) - 1):
        p0, t0 = waypoints[i]
        p1, t1 = waypoints[i + 1]

        t = 0
        dt = 0.001
        while not rospy.is_shutdown() and t <= 1:
            next_x, next_y = hx(t, p0, p1, t0, t1), hy(t, p0, p1, t0, t1)
            rospy.loginfo(f"Target point at t={t}: ({next_x}, {next_y})")

            twist = Twist()
            twist.linear.x = linear_velocity(t, p0, p1, t0, t1)
            twist.angular.z = angular_velocity(t, theta, p0, p1, t0, t1)

            pub.publish(twist)
            rate.sleep()
            
            # When next point is reached, then t is increased
            if reached_end_point(x, y, [next_x, next_y]):
                t += dt
                if t > 1:
                    break

    # Stop the robot when the final point is reached
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)



    
    