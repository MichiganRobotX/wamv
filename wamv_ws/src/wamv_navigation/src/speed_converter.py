#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class SpeedConverter():

    def __init__(self):
        # Initialize node
        rospy.init_node('speed_converter_node', log_level = rospy.DEBUG)

        # Set up subscribers
        rospy.Subscriber('/an_device/Twist', Twist, self.test_callback)

        # Set up publisher
        self.pub = rospy.Publisher('current_speed', Float64, queue_size = 1000)

        # Initialize subscribed topic values
        self.speed = Float64(0.0)

    def test_callback(self, msg):
        self.speed.data = msg.linear.x
        self.pub.publish(self.speed)

if __name__ == '__main__':
    converter = SpeedConverter()
    # rate = rospy.Rate(40)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     converter.calculate()
    #     rate.sleep()
