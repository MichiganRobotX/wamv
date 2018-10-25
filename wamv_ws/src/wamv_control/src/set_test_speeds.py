#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class TestSpeed():

    def __init__(self):
        self.pub = rospy.Publisher('set_speed', Float32, queue_size=1000)
        rospy.init_node('testing_speed_node', log_level=rospy.DEBUG)
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.min_speed = rospy.get_param('~min_speed', -1.0)
        self.period = rospy.get_param('~period', 10.0)

        self.speed = Float32()
        self.speed.data = self.max_speed

    def change_speed(self, curr_speed):

        if curr_speed == self.max_speed:
            return self.min_speed

        elif curr_speed == self.min_speed:
            return self.max_speed

    def process(self):

        curr_speed = self.speed.data
        rospy.loginfo("Set Speed: " + str(curr_speed))
        new_speed = self.change_speed(curr_speed)
        self.speed.data = new_speed
        self.pub.publish(self.speed)


test_speed = TestSpeed()
period = test_speed.period

while not rospy.is_shutdown():
    test_speed.process()
    rospy.sleep(period)
