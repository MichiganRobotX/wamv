#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16, Bool

SYSTEM_MODE = {
    0: 'limbo',
    1: 'remote_control',
    2: 'autonomous',
    3: 'killed'
}

###############################################################################
class StatusLightInterpretter():

    ###########################################################################
    def __init__(self):
        rospy.init_node('status_light_interpretter_node', log_level=rospy.INFO)

        # self.red_pub = rospy.Publisher('red_light', Bool, queue_size=100)
        self.green_pub = rospy.Publisher('green_light', Bool, queue_size=100)
        self.blue_pub = rospy.Publisher('blue_light', Bool, queue_size=100)
        self.yellow_pub = rospy.Publisher('yellow_light', Bool, queue_size=100)

        rospy.Subscriber('system_mode', Int16, self.mode_callback)

    ###########################################################################
    def mode_callback(self, msg):
        system_mode = SYSTEM_MODE[msg.data]

        if system_mode == 'remote_control':
            self.yellow_pub.publish(True)
            self.green_pub.publish(False)

        if system_mode == 'autonomous':
            self.green_pub.publish(True)
            self.yellow_pub.publish(False)

        if system_mode == 'limbo':
            self.green_pub.publish(False)
            self.yellow_pub.publish(False)

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        StatusLightInterpretter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
