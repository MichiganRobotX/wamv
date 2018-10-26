#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Int16

def sign(x):
    # cool, huh?
    return (1, -1)[x < 0]

class PIDInterpreter:
    def __init__(self):
        node_name = rospy.get_param('~node_name', 'interpreter_node')
        rospy.init_node(node_name, log_level=rospy.DEBUG)

        # Set up publishers
        port_motor_topic = rospy.get_param(
            '~port_motor_topic', 'port_motor_speed'
        )
        self.port_motor_publisher = rospy.Publisher(
            port_motor_topic, Int16, queue_size=1000
        )

        strbrd_motor_topic = rospy.get_param(
            '~strbrd_motor_topic', 'strbrd_motor_speed'
        )
        self.strbrd_motor_publisher = rospy.Publisher(
            strbrd_motor_topic, Int16, queue_size=1000
        )

        # Set up subscribers
        speed_control_effort_topic = rospy.get_param(
            '~speed_control_effort_topic', 'speed_control_effort'
        )
        rospy.Subscriber(
            speed_control_effort_topic, Int16, self.speed_callback
        )

        heading_control_effort_topic = rospy.get_param(
            '~heading_control_effort_topic', 'heading_control_effort'
        )
        rospy.Subscriber(
            heading_control_effort_topic, Int16, self.heading_callback
        )

        # Set up others
        self.maximum = rospy.get_param('~maximum', 127)
        self.heading_control_effort = 0
        self.speed_control_effort = 0

    def speed_callback(self, msg):
        """ Velocities fall in the range [-127,127] with -127 representing full
            reverse and 127 representing full forward
        """
        self.speed_control_effort = msg.data

    def heading_callback(self, msg):
        """ Headings fall in the range [-127,127] with -127 representing maximum
            counter-clockwise force and 127 representing maximum clockwise force
        """
        self.heading_control_effort = msg.data

    def process(self):
        """ Calculates and sends the motor speed signals.

            The motor works with values between [0,254] with 0 being full
            reverse and 254 being full forward. The translation happens like so:
                1) Suppose we have a heading_control_effort of 100, a maximum of
                    115, and a speed_control_effort of 80.
                2) PMS, SMS = 100, -100
                    max_ = 100 < maximum
                3) incr = 127 and speed_control_effort > 0
                    incr += min(80, 15)
                    incr = 142
                4) PMS, SMS = 242, 42
        """
        port_motor_speed = self.heading_control_effort
        strbrd_motor_speed = -self.heading_control_effort

        max_ = max(port_motor_speed, strbrd_motor_speed)
        if max_ > self.maximum:
            max_ = self.maximum
            port_motor_speed = max_ * sign(port_motor_speed)
            strbrd_motor_speed = max_ * sign(strbrd_motor_speed)

        incr = 127
        if self.speed_control_effort > 0:
            incr += min(self.speed_control_effort, self.maximum - max_)
        else:
            incr += max(self.speed_control_effort, max_ - self.maximum)

        port_motor_speed += incr
        strbrd_motor_speed += incr

        self.port_motor_publisher.publish(port_motor_speed)
        self.strbrd_motor_publisher.publish(strbrd_motor_speed)


if __name__ == '__main__':
    pid_interpreter = PIDInterpreter()
    loop_rate = rospy.get_param('~loop_rate', 10)
    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        pid_interpreter.process()
        rate.sleep()
