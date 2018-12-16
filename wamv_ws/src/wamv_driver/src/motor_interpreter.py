#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16
from wamv_msgs.msg import MotorCommand

SYSTEM_MODE = {
    0: 'limbo',
    1: 'remote_control',
    2: 'autonomous',
    3: 'killed'
}

###############################################################################
class MotorInterpreter():

    ###########################################################################
    def __init__(self):
        rospy.init_node('motor_interpreter_node', log_level=rospy.INFO)

        self.port_motor_pub = rospy.Publisher('port_motor_input', Int16, queue_size=100)
        self.strbrd_motor_pub = rospy.Publisher('strbrd_motor_input', Int16, queue_size=100)
        self.port_bow_thruster_pub = rospy.Publisher('port_bow_thruster_input', Int16, queue_size=100)
        self.strbrd_bow_thruster_pub = rospy.Publisher('strbrd_bow_thruster_input', Int16, queue_size=100)

        rospy.Subscriber('system_mode', Int16, self.mode_callback)
        rospy.Subscriber('motor_command', MotorCommand, self.command_callback)

        self.system_mode = 0

    ###########################################################################
    def publish_zero_speed(self):
        self.port_motor_pub.publish(127)
        self.strbrd_motor_pub.publish(127)
        self.port_bow_thruster_pub.publish(1500)
        self.strbrd_bow_thruster_pub.publish(1500)

    ###########################################################################
    def mode_callback(self, msg):
        self.system_mode = SYSTEM_MODE[msg.data]

    ###########################################################################
    def command_callback(self, msg):
        if self.system_mode == 'killed':
            self.publish_zero_speed()

        elif self.system_mode == 'limbo':
            self.publish_zero_speed()

        else:
            self.port_motor_pub.publish(msg.port_motor)
            self.strbrd_motor_pub.publish(msg.strbrd_motor)
            self.port_bow_thruster_pub.publish(msg.port_thruster)
            self.strbrd_bow_thruster_pub.publish(msg.strbrd_thruster)

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        MotorInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
