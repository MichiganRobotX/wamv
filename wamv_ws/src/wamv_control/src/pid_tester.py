#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('pid_tester', log_level=rospy.INFO)
    rospy.loginfo(
        'Initialized PID tester node. Intended for PID tuning purposes.')

    point_a = rospy.get_param('~point_a')
    point_b = rospy.get_param('~point_b')

    setpoint = point_a
    pub = rospy.Publisher('setpoint', Float64, queue_size=1000)

    period_duration = rospy.rostime.Duration(rospy.get_param('~period', 10))

    rospy.sleep(1.0)
    while not rospy.is_shutdown():

        rospy.loginfo("Publishing setpoint of {}".format(setpoint))
        pub.publish(setpoint)

        if setpoint == point_a:
            setpoint = point_b
        elif setpoint == point_b:
            setpoint = point_a
        else:
            rospy.error(
                "The setpoint {} does not evaluate to point_a or point_b"
                .format(setpoint))

        rospy.sleep(period_duration)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
