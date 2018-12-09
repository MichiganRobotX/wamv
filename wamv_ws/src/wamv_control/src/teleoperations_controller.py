#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

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
        self.is_teleoperations_activated = False
        self.is_running_autonomously = False

        rospy.init_node('teleop_controller_node', log_level=rospy.INFO)

        self.port_motor_pub = rospy.Publisher(
            'port_motor_speed', Int16, queue_size=100
        )
        self.strbrd_motor_pub = rospy.Publisher(
            'strbrd_motor_speed', Int16, queue_size=100
        )
        self.port_bow_thruster_pub = rospy.Publisher(
            'port_bow_thruster_speed', Int16, queue_size=100
        )
        self.strbrd_bow_thruster_pub = rospy.Publisher(
            'strbrd_bow_thruster_speed', Int16, queue_size=100
        )
        self.remote_control_status_pub = rospy.Publisher(
            'remote_control_status', Bool, queue_size=100
        )
        self.system_mode_pub = rospy.Publisher(
            'system_mode', Int16, queue_size=100
        )

        rospy.Subscriber('autonomy_status', Bool, self.autonomy_status_callback)
        rospy.Subscriber('joy', Joy, self.joy_callback)

    ###########################################################################
    def autonomy_status_callback(self, msg):
        """ Sets the value of self.is_running_autonomously based on the value
            published to the 'autonomy_status' topic.
        """
        if msg.data is True:
            self.is_running_autonomously = True
            rospy.loginfo('Autonomous mode is active.')

        elif msg.data is False:
            self.is_running_autonomously = False
            rospy.loginfo('Autonomous mode is NOT active.')

        else:
            rospy.logwarn('Autonomous mode is neither True nor False.')

    ###########################################################################
    def joy_callback(self, data):
        """ Handles joystick inputs.

            The `back` button on the controller deactivates teleop mode and
            sends zero speeds to all the motors.

            The `start` button on the controller activates teleop mode if the
            platform is not currently running autonomously, as indicated by the
            `autonomy_status` topic.
        """
        back_button_pressed = data.buttons[6] # back button
        start_button_pressed = data.buttons[7] # start button

        if back_button_pressed:
            self.is_teleoperations_activated = False

            zero_motor_speed = 127
            self.port_motor_pub.publish(zero_motor_speed)
            self.strbrd_motor_pub.publish(zero_motor_speed)

            zero_bow_thruster_speed = 1500
            self.port_bow_thruster_pub.publish(zero_bow_thruster_speed)
            self.strbrd_bow_thruster_pub.publish(zero_bow_thruster_speed)

            self.remote_control_status_pub.publish(False)
            rospy.loginfo("Joystick control deactivated")

        elif start_button_pressed:
            if not self.is_running_autonomously:
                self.is_teleoperations_activated = True
                self.system_mode_pub.publish(1)
                self.remote_control_status_pub.publish(True)
                rospy.loginfo("Joystick control activated.")

            else: # if the platform is running autonomously
                rospy.logerr(
                    "Joystick control cannot be activated while boat is "
                    "running autonomously.")
                self.is_teleoperations_activated = False

        if self.is_teleoperations_activated:
            self.publish_motor_commands(data)

    ###########################################################################
    def publish_motor_commands(self, data):
        """ Interprets joystick commands and publishes them to the motor topics.
        """
        left_stick_UD = data.axes[1]
        right_stick_UD = data.axes[4]
        left_stick_LR = data.axes[0]
        right_stick_LR = data.axes[3]

        port_motor_speed = int( 127 * (1 + left_stick_UD) )
        strbrd_motor_speed = int( 127 * (1 + right_stick_UD) )
        port_bow_thruster_speed = int( 1110 + 390 * (1 - left_stick_LR) )
        strbrd_bow_thruster_speed = int( 1110 + 390 * (1 + right_stick_LR) )

        self.port_motor_pub.publish(port_motor_speed)
        self.strbrd_motor_pub.publish(strbrd_motor_speed)
        self.port_bow_thruster_pub.publish(port_bow_thruster_speed)
        self.strbrd_bow_thruster_pub.publish(strbrd_bow_thruster_speed)


###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        teleop_controller = TeleoperationsController()
        rospy.loginfo('Initialized the teleoperations controller node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
