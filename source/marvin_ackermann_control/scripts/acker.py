#!/usr/bin/env python3
# Copyright 2016 Lucas Walter

import math
import numpy
import rospy
import tf2_py as tf2
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf import transformations
from visualization_msgs.msg import Marker


class Acker():
    def __init__(self):
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.odom_pub = rospy.Publisher('/marvin/odom', Odometry, queue_size=10)  # Odometry data topic

        # Initialize the vehicle's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0  # Assuming 2D plane; represented as yaw angle

        self.tf_buffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.joints = rospy.get_param("~steered_joints",
                                      [{'link': 'right_front_caster_horizontal',
                                        'steer_joint': 'left_front_caster_to_shoulder',
                                        'steer_topic': '/marvin/front_left/steer_position_controller/command',
                                        'wheel_joint': 'left_front_wheel_joint',
                                        'wheel_topic': '/marvin/front_left/wheel_position_controller/command'},
                                       {'link': 'left_front_caster_horizontal',
                                        'steer_joint': 'right_front_caster_to_shoulder',
                                        'steer_topic': '/marvin/front_right/steer_position_controller/command',
                                        'wheel_joint': 'right_front_wheel_joint',
                                        'wheel_topic': '/marvin/front_right/wheel_position_controller/command'},
                                       {'link': 'left_back_caster_horizontal',
                                        'steer_joint': None,
                                        'steer_topic': None,
                                        'wheel_joint': 'left_back_wheel_joint',
                                        'wheel_topic': '/marvin/back_left/wheel_position_controller/command'},
                                       {'link': 'right_back_caster_horizontal',
                                        'steer_joint': None,
                                        'steer_topic': None,
                                        'wheel_joint': 'right_back_wheel_joint',
                                        'wheel_topic': '/marvin/back_right/wheel_position_controller/command'}])

        self.wheel_radius = rospy.get_param("~wheel_radius", 0.15)
        # gazebo joint controller commands
        self.command_pub = {}
        # use this to store all the positions persistently
        self.wheel_joint_states = JointState()
        for wheel in self.joints:
            steer_topic = wheel['steer_topic']
            if steer_topic is not None:
                self.command_pub[wheel['steer_joint']] = rospy.Publisher(steer_topic,
                                                                         Float64, queue_size=4)
            wheel_topic = wheel['wheel_topic']
            if wheel_topic is not None:
                self.command_pub[wheel['wheel_joint']] = rospy.Publisher(wheel_topic,
                                                                         Float64, queue_size=4)

            self.wheel_joint_states.name.append(wheel['wheel_joint'])
            self.wheel_joint_states.position.append(0.0)
            self.wheel_joint_states.velocity.append(0.0)


        self.ts = TransformStamped()
        self.ts.header.frame_id = "map"
        self.ts.child_frame_id = "base_link"
        self.ts.transform.rotation.w = 1.0
        # the angle of the base_link
        self.angle = 0

        # the fixed back axle- all the fixed wheels rotate around the y-axis
        # of the back axle
        self.back_axle_link = rospy.get_param("~back_axle_link", "back_axle")


        self.marker = Marker()
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.frame_locked = True
        self.marker.action = Marker.ADD
        self.marker.header.frame_id = self.back_axle_link
        self.marker.scale.x = 0.07
        self.marker.scale.y = 0.07
        self.marker.scale.z = 0.07
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 0.5
        self.marker.pose.orientation.w = 1.0

        self.marker_pub = rospy.Publisher("marker", Marker, queue_size=len(self.joints) * 2)
        self.point_pub = rospy.Publisher("spin_center", PointStamped, queue_size=1)
        self.joint_pub = rospy.Publisher("steered_joint_states", JointState, queue_size=3)
        self.lead_steer = rospy.get_param("~steer", {'link': 'lead_steer',
                                                     'joint': 'lead_steer_joint',
                                                     'wheel_joint': 'wheel_lead_axle'})
        self.twist_pub = rospy.Publisher("odom_cmd_vel", Twist, queue_size=3)
        #self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=3)
        self.joint_sub = rospy.Subscriber("joint_states", JointState,
                                          self.lead_steer_callback, queue_size=4)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def update(self, event):
        pass

    # get the angle to to the wheel from the spin center
    def get_angle(self, link, spin_center, steer_angle, stamp):
        # lookup the position of each link in the back axle frame
        ts = self.tf_buffer.lookup_transform(spin_center.header.frame_id, link,
                                             stamp, rospy.Duration(4.0))

        dy = ts.transform.translation.y - spin_center.point.y
        dx = ts.transform.translation.x - spin_center.point.x
        angle = math.atan2(dx, abs(dy))
        if steer_angle < 0:
            angle = -angle

        # visualize the trajectory forward or back of the current wheel
        # given the spin center
        radius = math.sqrt(dx * dx + dy * dy)
        self.marker.points = []
        for pt_angle in numpy.arange(abs(angle) - 1.0, abs(angle) + 1.0 + 0.025, 0.05):
            pt = Point()
            pt.x = spin_center.point.x + radius * math.sin(pt_angle)
            if steer_angle < 0:
                pt.y = spin_center.point.y - radius * math.cos(pt_angle)
            else:
                pt.y = spin_center.point.y + radius * math.cos(pt_angle)
            self.marker.ns = link
            self.marker.header.stamp = stamp
            self.marker.points.append(pt)
        self.marker_pub.publish(self.marker)
        return angle, radius

    def lead_steer_callback(self, msg):
        lead_steer_joint = self.lead_steer['joint']
        if lead_steer_joint not in msg.name:
            rospy.logwarn("no %s joint in joint state msg" % (lead_steer_joint))
            rospy.logwarn(msg)
            return
        lead_wheel_joint = self.lead_steer['wheel_joint']
        if lead_wheel_joint not in msg.name:
            rospy.logwarn("no %s joint in joint state msg" % (lead_wheel_joint))
            rospy.logwarn(msg)
            return

        dt = (msg.header.stamp - self.wheel_joint_states.header.stamp).to_sec()
        if self.wheel_joint_states.header.stamp.to_sec() == 0.0:
            dt = 0

        steer_ind = msg.name.index(lead_steer_joint)
        wheel_ind = msg.name.index(lead_wheel_joint)
        if len(msg.velocity) < wheel_ind:
            # rospy.logwarn("no velocity for wheel_ind %d", wheel_ind)
            return

        steer_angle = msg.position[steer_ind]
        lead_wheel_angular_velocity = msg.velocity[wheel_ind]
        # steer_velocity = msg.velocity[steer_ind]
        # steer_effort = msg.effort[steer_ind]

        joint_states = JointState()
        joint_states.header = msg.header
        self.wheel_joint_states.header = msg.header
        odom_cmd_vel = Twist()
        #odom = Odometry()

        if steer_angle == 0.0:
            for i in range(len(self.joints)):
                steer_joint = self.joints[i]['steer_joint']
                if steer_joint is not None:
                    joint_states.name.append(steer_joint)
                    joint_states.position.append(steer_angle)
                    # joint_states.velocity.append(steer_velocity)
                    # joint_states.effort.append(steer_effort)
                if steer_joint in self.command_pub.keys():
                    self.command_pub[steer_joint].publish(steer_angle)

                wheel_joint = self.joints[i]['wheel_joint']
                ind = self.wheel_joint_states.name.index(wheel_joint)
                self.wheel_joint_states.position[ind] += lead_wheel_angular_velocity * dt
                self.wheel_joint_states.velocity[ind] = lead_wheel_angular_velocity
                if wheel_joint in self.command_pub.keys():
                    self.command_pub[wheel_joint].publish(self.wheel_joint_states.position[ind])

            self.joint_pub.publish(joint_states)
            self.joint_pub.publish(self.wheel_joint_states)

            # upate odometry
            distance = self.wheel_radius * lead_wheel_angular_velocity * dt
            if dt > 0:
                odom_cmd_vel.linear.x = distance / dt
                #odom.pose.pose.position.x = distance / dt
            self.twist_pub.publish(odom_cmd_vel)
            #self.odom_pub.publish(odom)

            self.ts.transform.translation.x += distance * math.cos(self.angle)
            self.ts.transform.translation.y += distance * math.sin(-self.angle)

            # set the quaternion to current angle
            quat = transformations.quaternion_from_euler(0, 0, -self.angle)
            self.ts.transform.rotation.x = quat[0]
            self.ts.transform.rotation.y = quat[1]
            self.ts.transform.rotation.z = quat[2]
            self.ts.transform.rotation.w = quat[3]

            self.ts.header.stamp = msg.header.stamp
            # convert self.angle to quaternion
            self.br.sendTransform(self.ts)
            return

        # find spin center given steer joint
        steer_ts = self.tf_buffer.lookup_transform(self.back_axle_link, self.lead_steer['link'],
                                                   msg.header.stamp, rospy.Duration(4.0))

        spin_center = PointStamped()
        # R cos(steer_angle) = y
        # R sin(steer_angle) = x
        # x = steer_ts.position.x
        # R = steer_ts.position.x / sin(steer_angle)
        y = steer_ts.transform.translation.x / math.tan(-steer_angle)
        spin_center.point.y = y + steer_ts.transform.translation.y
        spin_center.header.stamp = msg.header.stamp
        spin_center.header.frame_id = self.back_axle_link
        self.point_pub.publish(spin_center)

        angle, lead_radius = self.get_angle(self.lead_steer['link'], spin_center,
                                            steer_angle, msg.header.stamp)

        for i in range(len(self.joints)):
            joint = self.joints[i]['steer_joint']
            link = self.joints[i]['link']

            angle, radius = self.get_angle(link, spin_center, steer_angle, msg.header.stamp)
            fr = radius / lead_radius

            # print link, angle, dx, dy
            if joint is not None:
                joint_states.name.append(joint)
                joint_states.position.append(angle)

            if joint in self.command_pub.keys():
                self.command_pub[joint].publish(angle)

            wheel_joint = self.joints[i]['wheel_joint']
            ind = self.wheel_joint_states.name.index(wheel_joint)
            
            self.wheel_joint_states.position[ind] += lead_wheel_angular_velocity * fr * dt
            self.wheel_joint_states.velocity[ind] = lead_wheel_angular_velocity * fr
            if wheel_joint in self.command_pub.keys():
                self.command_pub[wheel_joint].publish(self.wheel_joint_states.position[ind])

        # update odometry
        steer_angle, radius = self.get_angle("base_link", spin_center,
                                             steer_angle, msg.header.stamp)
        fr = radius / lead_radius
        # distance traveled along the radial path in base_link
        distance = self.wheel_radius * lead_wheel_angular_velocity * fr * dt

        self.x += distance * math.cos(self.orientation)  # Update x position
        self.y += distance * math.sin(self.orientation)  # Update y position
        self.orientation += steer_angle * dt  # Update orientation (yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set the orientation (converting from Euler to quaternion)
        quat2 = transformations.quaternion_from_euler(0, 0, self.orientation)
        odom.pose.pose.orientation.x = quat2[0]
        odom.pose.pose.orientation.y = quat2[1]
        odom.pose.pose.orientation.z = quat2[2]
        odom.pose.pose.orientation.w = quat2[3]

        # Publish the Odometry message
        self.odom_pub.publish(odom)
        
        angle_traveled = distance / radius
        # the distance traveled in the base_link frame:
        dx_in_ts = distance * math.cos(steer_angle)
        dy_in_ts = -distance * math.sin(steer_angle)
        if dt > 0:
            odom_cmd_vel.linear.x = dx_in_ts / dt
            odom_cmd_vel.linear.y = dy_in_ts / dt
            #odom.pose.pose.position.x = dx_in_ts / dt
            #odom.pose.pose.position.y = dy_in_ts / dt
            # print math.degrees(steer_angle), distance, odom_cmd_vel.linear.x, \
            #         odom_cmd_vel.linear.y, radius, math.degrees(angle_traveled)
            self.twist_pub.publish(odom_cmd_vel)
            # self.odom_pub.publish(odom)

        # then need to rotate x and y by self.angle
        dx_in_parent = distance * math.cos(self.angle + steer_angle)
        dy_in_parent = -distance * math.sin(self.angle + steer_angle)
        # then can add the rotated x and y to self.ts.transform.translation
        self.ts.transform.translation.x += dx_in_parent
        self.ts.transform.translation.y += dy_in_parent

        # set the quaternion to current angle
        quat = transformations.quaternion_from_euler(0, 0, -self.angle)
        self.ts.transform.rotation.x = quat[0]
        self.ts.transform.rotation.y = quat[1]
        self.ts.transform.rotation.z = quat[2]
        self.ts.transform.rotation.w = quat[3]

        self.ts.header.stamp = msg.header.stamp
        # convert self.angle to quaternion
        self.br.sendTransform(self.ts)

        self.joint_pub.publish(joint_states)
        self.joint_pub.publish(self.wheel_joint_states)

        if steer_angle > 0:
            self.angle += angle_traveled
        else:
            self.angle -= angle_traveled


if __name__ == '__main__':
    rospy.init_node("acker")
    acker = Acker()
    rospy.spin()
