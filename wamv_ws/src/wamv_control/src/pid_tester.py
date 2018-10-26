#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('pid_tester', log_level=rospy.DEBUG)

    point_a = rospy.get_param('~point_a')
    point_b = rospy.get_param('~point_b')

    setpoint = Float64(point_a)
    pub = rospy.Publisher('setpoint', Float64, queue_size=1000)

    period_duration = rospy.duration(rospy.get_param('~period', 10))
    while not rospy.is_shutdown():
        pub.publish(setpoint)

        rospy.loginfo("Setpoint: " + str(setpoint.data))

        if setpoint.data == point_a:
            setpoint.data = point_b
        elif setpoint.data == point_b:
            setpoint.data = point_a
        else:
            rospy.error("The setpoint does not evaluate to point_a or point_b")

        period_duration.sleep()
