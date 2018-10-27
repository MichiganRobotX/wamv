#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class HeadingConverter():

    def __init__(self):
        # Initialize node
        rospy.init_node('heading_unit_converter_node', log_level = rospy.DEBUG)

        # Set up subscribers
	    rospy.Subscriber('an_device/Imu', Imu, self.test_callback)

        # Set up publisher
        self.pub = rospy.Publisher('current_heading', Float64, queue_size = 1000)

        # Initialize subscribed topic values

        self.w = Float64(0.0)
        self.x = Float64(0.0)
        self.y = Float64(0.0)
        self.z = Float64(0.0)

    def test_callback(self, msg):
        self.w.data = msg.orientation.w
        self.x.data = msg.orientation.x
        self.y.data = msg.orientation.y
        self.z.data = msg.orientation.z

    def quat_to_deg(self, w, x, y, z):
    	t1 = 2.0 * (w * z + x * y)
        t2 = 1.0 - 2.0 * (y * y + z * z)
        deg_heading = math.degrees(math.atan2(t1, t2))
	    if deg_heading < 0.0:
		          deg_heading +=360
	    return deg_heading

    def calculate(self):
        deg_heading = self.quat_to_deg(self.w.data, self.x.data, self.y.data, self.z.data)
        rospy.loginfo("Current Heading = " + str(deg_heading))
        self.pub.publish(deg_heading)

if __name__ == '__main__':
    heading_converter = HeadingConverter()
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        heading_converter.calculate()
        rate.sleep()
