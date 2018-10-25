#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class TestHeading():

    def __init__(self):
        self.pub = rospy.Publisher('heading_to_target', Float32, queue_size=1000)
        rospy.init_node('testing_heading_node', log_level=rospy.DEBUG)
        self.max_heading = rospy.get_param('~max_heading', 45.0)
        self.min_heading = rospy.get_param('~min_heading', -45.0)
        self.period = rospy.get_param('~period', 10)

        self.heading = Float32()
        self.heading.data = self.max_heading

    def change_heading(self, curr_heading):

        if curr_heading == self.max_heading:
            return self.min_heading

        elif curr_heading == self.min_heading:
            return self.max_heading

    def process(self):

        curr_heading = self.heading.data
        rospy.loginfo("Set Heading: " + str(curr_heading))
        new_heading = self.change_heading(curr_heading)
        self.heading.data = new_heading
        self.pub.publish(self.heading)


test_heading = TestHeading()
period = test_heading.period

while not rospy.is_shutdown():
    test_heading.process()
    rospy.sleep(period)
