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
class MotorCommandInterpretter():

    ###########################################################################
    def __init__(self):
        rospy.init_node('motor_command_interpretter_node', log_level=rospy.INFO)

        self.port_motor_pub = rospy.Publisher('topic_to_port_motor', Int16, queue_size=100)
        self.strbrd_motor_pub = rospy.Publisher('topic_to_strbrd_motor', Int16, queue_size=100)
        self.port_bow_thruster_pub = rospy.Publisher('topic_to_port_bow_thruster', Int16, queue_size=100)
        self.strbrd_bow_thruster_pub = rospy.Publisher('topic_to_strbrd_bow_thruster', Int16, queue_size=100)

        rospy.Subscriber('system_mode', Int16, self.mode_callback)
        rospy.Subscriber('motor_command', MotorCommand, self.command_callback)

        self.zero_speed_msg = MotorCommand()
        self.zero_speed_msg.port_motor = 127
        self.zero_speed_msg.strbrd_motor = 127
        self.zero_speed_msg.port_bow_thruster = 1500
        self.zero_speed_msg.strbrd_bow_thruster = 1500

    ###########################################################################
    def mode_callback(self, msg):
        self.system_mode = SYSTEM_MODE[msg.data]

    ###########################################################################
    def command_callback(self, msg):
        if self.system_mode == 'killed':
            self.port_motor_pub.publish(self.zero_speed_msg.port_motor)
            self.strbrd_motor_pub.publish(self.zero_speed_msg.strbrd_motor)
            self.port_bow_thruster_pub.publish(self.zero_speed_msg.port_bow_thruster)
            self.strbrd_bow_thruster_pub.publish(self.zero_speed_msg.strbrd_bow_thruster)

        else:
            self.port_motor_pub.publish(msg.port_motor)
            self.strbrd_motor_pub.publish(msg.strbrd_motor)
            self.port_bow_thruster_pub.publish(msg.port_bow_thruster)
            self.strbrd_bow_thruster_pub.publish(msg.strbrd_bow_thruster)

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        MotorCommandInterpretter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
