#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('pid_tester', log_level=rospy.INFO)
    rospy.loginfo(
        'Initialized PID tester node. Intended for PID tuning purposes.')

    point_a = rospy.get_param('~point_a')
    point_b = rospy.get_param('~point_b')

    setpoint = Float64(point_a)
    pub = rospy.Publisher('setpoint', Float64, queue_size=1000)

    period_duration = rospy.rostime.Duration(rospy.get_param('~period', 10))
    while not rospy.is_shutdown():
        pub.publish(setpoint)

        rospy.loginfo("Publishing setpoint of {}".format(setpoint.data))

        if setpoint.data == point_a:
            setpoint.data = point_b
        elif setpoint.data == point_b:
            setpoint.data = point_a
        else:
            rospy.error(
                "The setpoint {} does not evaluate to point_a or point_b"
                .format(setpoint.data))

        rospy.sleep(period_duration)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
