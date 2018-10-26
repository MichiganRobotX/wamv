#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class SpeedTester():
    """ Tests the speed PID controller for calibration purposes.

        # Parameters
        - min_speed: ... in (units?). Default is -1.0 ...
        - max_speed: ... in (units?). Default is 1.0 ...
        - period: The period between switching from max_speed to min_speed,
            in seconds. Default is 10 seconds.

        # Published Topics
        - set_speed (Float32):
    """

    def __init__(self):
        rospy.init_node('testing_speed_node', log_level=rospy.DEBUG)

        self.min_speed = rospy.get_param('~min_speed', -1.0)
        self.max_speed = rospy.get_param('~max_speed', 1.0)

        self.speed = Float32(self.max_speed)
        self.pub = rospy.Publisher('set_speed', Float32, queue_size=1000)

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


if __name__ == '__main__':
    speed_tester = SpeedTester()
    period_duration = rospy.duration(
        rospy.get_param('~period', 10)
    )
    while not rospy.is_shutdown():
        speed_tester.process()
        period_duration.sleep()
