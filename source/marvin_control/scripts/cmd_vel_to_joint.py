#!/usr/bin/env python3
# Subscribe to a cmd_vel Twist message and interpret the linear
# and angular components into a joint state for the virtual steer joint.

import math
import rospy
import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState



class CmdVelToJoint():
    def __init__(self):
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.angular_mode = rospy.get_param("~angular_mode", True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)

        self.steer_link = rospy.get_param("~steer_link", "lead_steer")
        self.steer_joint = rospy.get_param("~steer_joint", "lead_steer_joint")

        self.min_steer_angle = rospy.get_param("~min_steer_angle", -0.7)
        self.max_steer_angle = rospy.get_param("~max_steer_angle", 0.7)

        self.wheel_joint = rospy.get_param("~wheel_joint", "wheel_lead_axle")
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.15)

        self.fixed_axle_link = rospy.get_param("~fixed_axle_link", "back_axle")

        self.point_pub = rospy.Publisher("cmd_vel_spin_center", PointStamped, queue_size=1)
        self.steer_pub = rospy.Publisher("steer_joint_states", JointState, queue_size=1)

        self.joint_state = JointState()
        self.joint_state.name.append(self.steer_joint)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.joint_state.name.append(self.wheel_joint)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.cmd_vel = Twist()
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def update(self, event):
        try:
            fixed_to_steer = self.tf_buffer.lookup_transform(self.fixed_axle_link,
                                                             self.steer_link,
                                                             rospy.Time(),
                                                             rospy.Duration(4.0))
        except tf2.ExtrapolationException as e:
            rospy.logwarn(e)
            return
        except tf2.LookupException as e:
            rospy.logwarn(e)
            return

        self.joint_state.header.stamp = fixed_to_steer.header.stamp

        
        fixed_to_base = self.tf_buffer.lookup_transform(self.fixed_axle_link,
                                                        "base_link",
                                                        rospy.Time(),
                                                        rospy.Duration(4.0))
        
        linear_x = self.cmd_vel.linear.x
        linear_y = self.cmd_vel.linear.y
        angular_z = self.cmd_vel.angular.z
        wheel_angular_velocity = 0
        if self.angular_mode or linear_y == 0.0:
            self.joint_state.position[0] = 0.0
            self.joint_state.velocity[0] = 0.0
            wheel_angular_velocity = linear_x / self.wheel_radius
            
            if self.angular_mode and angular_z != 0.0 and linear_x != 0.0:
                base_turn_radius = abs(linear_x) / abs(angular_z)
                
                if base_turn_radius < fixed_to_base.transform.translation.x:
                    base_turn_radius = fixed_to_base.transform.translation.x
                    
                
                try:
                    fixed_to_base_angle = math.asin(fixed_to_base.transform.translation.x / base_turn_radius)
                except Exception as ex:
                    rospy.logwarn(f"{base_turn_radius}, {fixed_to_base.transform.translation.x}, {ex}")
                    return
                
                spin_center_y = base_turn_radius * math.cos(fixed_to_base_angle)
                steer_angle = math.atan2(fixed_to_steer.transform.translation.x,
                                         spin_center_y)
                if angular_z > 0.0:
                    spin_center_y = -spin_center_y
                    steer_angle = -steer_angle
                if linear_x < 0.0:
                    spin_center_y = -spin_center_y
                    steer_angle = -steer_angle
                self.joint_state.position[0] = steer_angle
        elif linear_x == 0.0:
            wheel_angular_velocity = linear_y / self.wheel_radius
            if linear_y > 0:
                self.joint_state.position[0] = self.min_steer_angle
            else:
                self.joint_state.position[0] = self.max_steer_angle
            self.joint_state.velocity[0] = 0.0
        else:
            lin_y = self.cmd_vel.linear.y
            lin_x = linear_x
            lin_mag = math.sqrt(lin_x * lin_x + lin_y * lin_y)
            base_y = fixed_to_base.transform.translation.y
            base_x = fixed_to_base.transform.translation.x
            ty = lin_y + base_y
            tx = lin_x + base_x

            lin_angle = math.atan2(lin_y, lin_x)
            
            radius_2 = (lin_x / 2.0 + base_x) / math.sin(lin_angle)
            spin_angle_traveled = 2.0 * math.atan2(lin_mag / 2.0, radius_2)
            
            base_offset_angle = lin_angle - spin_angle_traveled / 2.0
            base_spin_radius = radius_2 / math.cos(spin_angle_traveled / 2.0)

            back_spin_radius = base_spin_radius * math.cos(base_offset_angle) + base_y
            spin_center = PointStamped()
            spin_center.point.y = back_spin_radius
            spin_center.header.stamp = rospy.Time.now()
            spin_center.header.frame_id = self.fixed_axle_link
            self.point_pub.publish(spin_center)

            steer_spin_radius_dx = fixed_to_steer.transform.translation.x
            steer_spin_radius_dy = back_spin_radius - fixed_to_steer.transform.translation.y
        
            steer_angle = math.atan2(steer_spin_radius_dx, steer_spin_radius_dy)
            steer_spin_radius = math.sqrt(steer_spin_radius_dx**2 + steer_spin_radius_dy**2)
            distance_traveled = steer_spin_radius * spin_angle_traveled
            wheel_angular_velocity = distance_traveled / self.wheel_radius

            self.joint_state.position[0] = -steer_angle
            self.joint_state.velocity[0] = 0.0

        self.joint_state.position[1] += wheel_angular_velocity * self.period
        self.joint_state.velocity[1] = wheel_angular_velocity

        self.steer_pub.publish(self.joint_state)


if __name__ == '__main__':
    rospy.init_node("cmd_vel_to_joint")
    cmd_vel_to_joint = CmdVelToJoint()
    rospy.spin()
