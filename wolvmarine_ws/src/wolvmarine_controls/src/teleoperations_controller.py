#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from wolvmarine_msgs.msg import MotorCommand

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

        self.max_x_velocity = rospy.get_param('~max_x_velocity', 1.0)
        self.max_y_velocity = rospy.get_param('~max_y_velocity', 1.0)
        self.max_yaw_velocity = rospy.get_param('~max_yaw_velocity', 1.0)

        self.system_mode_pub = rospy.Publisher(
            'system_mode', Int16, queue_size=100)
        self.velocity_pub = rospy.Publisher(
            'velocity_command', Twist, queue_size=100)
        self.motor_pub = rospy.Publisher(
            'motor_command', MotorCommand, queue_size=100)

        rospy.Subscriber('joystick', Joy, self.joystick_callback)

        self.enable_control('motor')

    ###########################################################################
    def enable_control(self, mode):
        self.publish = self._publish_motor
        self.publisher = self.motor_pub
        # if mode == 'velocity':
        #     self.publish = self._publish_velocity
        #     self.publisher = self.velocity_pub
        #
        # elif mode == 'motor':
        #     self.publish = self._publish_motor
        #     self.publisher = self.motor_pub

    ###########################################################################
    def _publish_motor(self, left_stick_up_down, right_stick_up_down,
                       left_stick_left_right, right_stick_left_right):
        msg = MotorCommand()

        msg.port_motor = int( 127 * (1 + left_stick_up_down) )
        msg.strbrd_motor = int( 127 * (1 + right_stick_up_down) )
        msg.port_thruster = int( 1110 + 390 * (1 - left_stick_left_right) )
        msg.strbrd_thruster = int( 1110 + 390 * (1 + right_stick_left_right) )

        rospy.loginfo('\nPort motor:\t{}\nStrbrd motor:\t{}\nPort thruster:\t{}\nStrbrd thruster:\t{}'
            .format(msg.port_motor, msg.strbrd_motor, msg.port_thruster, msg.strbrd_thruster))
        self.publisher.publish(msg)

    ###########################################################################
    def _publish_velocity(self, left_stick_up_down, right_stick_up_down,
                          left_stick_left_right, right_stick_left_right):
        msg = Twist()

        avg_up_down = (left_stick_up_down + right_stick_up_down) / 2.0
        x_velocity = avg_up_down * self.max_x_velocity
        msg.linear.x = x_velocity

        avg_left_right = (left_stick_left_right + right_stick_left_right) / 2.0
        y_velocity = avg_left_right * self.max_y_velocity
        msg.linear.y = y_velocity

        diff_up_down = (right_stick_up_down - left_stick_up_down) / 2.0
        yaw_velocity = diff_up_down * self.max_yaw_velocity
        msg.angular.z = yaw_velocity

        self.publisher.publish(msg)

    ###########################################################################
    def joystick_callback(self, data):
        """ Handles joystick inputs.

            The `back` button on the controller deactivates teleop mode and
            sends zero speeds to all the motors.

            The `start` button on the controller activates teleop mode if the
            platform is not currently running autonomously, as indicated by the
            `autonomy_status` topic.
        """
        # self.handle_buttons(data)
        # self.handle_joysticks(data)
        # self.publish()

        green_button_pressed = data.buttons[0]
        red_button_pressed = data.buttons[1]
        blue_button_pressed = data.buttons[2]
        yellow_button_pressed = data.buttons[3]
        back_button_pressed = data.buttons[6]
        start_button_pressed = data.buttons[7]

        if red_button_pressed:
            self.system_mode_pub.publish(3)
            rospy.loginfo("Killswitch engaged")

        elif green_button_pressed:
            self.system_mode_pub.publish(2)
            rospy.loginfo("Autonomous control activated")

        # elif yellow_button_pressed:
        #     self.enable_control('velocity')
        #     self.system_mode_pub.publish(1)
        #     rospy.loginfo("Joystick control enabled. Sending VELOCITY commands.")

        elif blue_button_pressed or start_button_pressed:
            self.enable_control('motor')
            self.system_mode_pub.publish(1)
            rospy.loginfo("Joystick control enabled. Sending MOTOR commands.")

        elif back_button_pressed:
            self.system_mode_pub.publish(0)
            rospy.loginfo("Control deactivated")

        left_stick_up_down = data.axes[1]
        right_stick_up_down = data.axes[4]
        left_stick_left_right = data.axes[0]
        right_stick_left_right = data.axes[3]

        self.publish(left_stick_up_down, right_stick_up_down,
                     left_stick_left_right, right_stick_left_right)

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        TeleoperationsController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
