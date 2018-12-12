#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('setpoint_publisher', log_level=rospy.INFO)
    rospy.loginfo(
        'Initialized PID tester node. Intended for PID tuning purposes.')

    setpoints = rospy.get_param('~setpoints', [])
    repeat = rospy.get_param('~repeat', True)
    period = rospy.rostime.Duration(rospy.get_param('~period', 10))

    pub = rospy.Publisher('setpoint', Float64, queue_size=100)

    rospy.sleep(1.0)
    while not rospy.is_shutdown():
        for setpoint in setpoints:
            rospy.loginfo("Publishing setpoint of {}".format(setpoint))
            pub.publish(setpoint)
            rospy.sleep(period)
        if not repeat:
            break


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
