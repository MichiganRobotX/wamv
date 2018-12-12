#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from wamv_msgs.msg import MotorCommand

SYSTEM_MODE = {
    0: 'limbo',
    1: 'remote_control',
    2: 'autonomous',
    3: 'killed'
}

###############################################################################
class TeleoperationsController:
    """ Handles joystick commands for running in teleoperations mode.

        # Published Topics
        - port_motor_speed (Int16): The port-side main motor input.
        - strbrd_motor_speed (Int16): The starboard-side main motor input.
        - port_bow_thruster_speed (Int16): The port-side bow thruster input.
        - strbrd_bow_thruster_speed (Int16): The starboard-side bow thruster
            input.
        - remote_control_status (Bool): True if successful activation of
            teleoperation controller, False otherwise.

        # Subscribed Topics
        - autonomy_status (Bool): True when the boat is running autonomously.
            Individual tasks publish here. Those tasks must be shutdown before
            teleoperations mode can be activated.
        - joy (Joy): Published to by the joystick.
    """

    ###########################################################################
    def __init__(self):
        rospy.init_node('teleoperations_controller_node', log_level=rospy.INFO)

        self.publisher = rospy.Publisher(
            'motor_command', MotorCommand, queue_size=100)
        self.system_mode_pub = rospy.Publisher(
            'system_mode', Int16, queue_size=100)

        self.zero_speed_msg = MotorCommand()
        self.zero_speed_msg.port_motor = 127
        self.zero_speed_msg.strbrd_motor = 127
        self.zero_speed_msg.port_bow_thruster = 1500
        self.zero_speed_msg.strbrd_bow_thruster = 1500

        rospy.Subscriber('joystick', Joy, self.joy_callback)
        # rospy.Subscriber('system_mode', Int16, self.mode_callback)

    ###########################################################################
    def joy_callback(self, data):
        """ Handles joystick inputs.

            The `back` button on the controller deactivates teleop mode and
            sends zero speeds to all the motors.

            The `start` button on the controller activates teleop mode if the
            platform is not currently running autonomously, as indicated by the
            `autonomy_status` topic.
        """
        green_button_pressed = data.buttons[0]
        red_button_pressed = data.buttons[1]
        back_button_pressed = data.buttons[6]
        start_button_pressed = data.buttons[7]

        if red_button_pressed:
            self.system_mode_pub.publish(3)
            rospy.loginfo("Killswitch engaged")

        elif green_button_pressed:
            self.system_mode_pub.publish(2)
            rospy.loginfo("Autonomous control activated")

        elif back_button_pressed:
            self.system_mode_pub.publish(0)
            self.publisher.publish(self.zero_speed_msg)
            rospy.loginfo("Joystick control deactivated")

        elif start_button_pressed:
            self.system_mode_pub.publish(1)
            rospy.loginfo("Joystick control activated.")

        left_stick_UD = data.axes[1]
        right_stick_UD = data.axes[4]
        left_stick_LR = data.axes[0]
        right_stick_LR = data.axes[3]

        msg = MotorCommand()
        msg.port_motor = int( 127 * (1 + left_stick_UD) )
        msg.strbrd_motor = int( 127 * (1 + right_stick_UD) )
        msg.port_bow_thruster = int( 1110 + 390 * (1 - left_stick_LR) )
        msg.strbrd_bow_thruster = int( 1110 + 390 * (1 + right_stick_LR) )

        self.publisher.publish(msg)

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        TeleoperationsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
