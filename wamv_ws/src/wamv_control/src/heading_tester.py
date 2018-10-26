#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class HeadingTester():
    """ Tests the heading PID controller for calibration purposes.

        # Parameters
        - min_heading: ... in degrees. Default is -45 degrees.
        - max_heading: ... in degrees. Default is 45 degrees.
        - period: The period between switching from max_heading to min_heading,
            in seconds. Default is 10 seconds.

        # Published Topics
        - heading_to_target (Float32):
    """

    def __init__(self):
        rospy.init_node('heading_tester_node', log_level=rospy.DEBUG)

        self.min_heading = rospy.get_param('~min_heading', -45.0)
        self.max_heading = rospy.get_param('~max_heading', 45.0)

        self.heading = Float32(self.max_heading)
        self.pub = rospy.Publisher('heading_to_target', Float32, queue_size=1000)

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


if __name__ == '__main__':
    heading_tester = HeadingTester()
    period_duration = rospy.duration(
        rospy.get_param('~period', 10)
    )
    while not rospy.is_shutdown():
        heading_tester.process()
        period_duration.sleep()
