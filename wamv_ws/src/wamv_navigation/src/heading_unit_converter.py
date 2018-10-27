#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64

class HeadingConverter():

    def __init__(self):
        # Initialize node
        rospy.init_node('heading_unit_converter_node', log_level = rospy.DEBUG)

        # Set up subscribers
        rospy.Subscriber('an_device/Imu/orientation/w', Float64, self.w_callback)
        rospy.Subscriber('an_device/Imu/orientation/x', Float64, self.x_callback)
        rospy.Subscriber('an_device/Imu/orientation/y', Float64, self.y_callback)
        rospy.Subscriber('an_device/Imu/orientation/z', Float64, self.z_callback)

        # Set up publisher
        self.pub = rospy.Publisher('current_heading', Float64, queue_size = 1000)

        # Initialize subscribed topic values
        self.w = Float64(0.0)
        self.x = Float64(0.0)
        self.y = Float64(0.0)
        self.z = Float64(0.0)

    def w_callback(self, msg):
        self.w = msg.data

    def x_callback(self, msg):
        self.x = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def z_callback(self, msg):
        self.z = msg.data

    def calculate(self):
        t1 = +2.0 * (self.w * self.z + self.x * self.y)
        t2 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        deg_heading = math.degrees(math.atan2(t1, t2))

        self.pub.publish(deg_heading)

if __name__ == '__main__':
    heading_converter = HeadingConverter()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        heading_converter.calculate()
        rate.sleep()
