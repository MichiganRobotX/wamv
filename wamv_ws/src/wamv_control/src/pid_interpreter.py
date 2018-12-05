#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64

def sign(x):
    """ Returns the sign of 'x'. Pretty cool, huh?
    """
    return (1, -1)[x < 0]

###############################################################################
class PIDInterpreter:
    """ The interpreter between the outputs of the PID controllers and the
        inputs of the main motors. Responsible for performing the necessary
        logic to transform 'heading' and 'speed' control efforts into port and
        starboard main motor inputs.

        # Parameters
        - maximum_output: The maximum output of the interpreter, aka the
            maximum main motor input. Prioritizes heading controller. Default
            is 127.
        - maximum_output_thrusters: The maximum output of the interpreter for
           the thrusters. Default is 390.
        - loop_rate: The rate of the main loop, in Hz. Default is 10 Hz.

        # Subscribed Topics
        - topic_from_heading_controller (Float64): The control effort being
            published by the heading controller.
        - topic_from_speed_controller (Float64): The control effort being
            published be the speed controller.
        - topic_from_lateral_controller (Float64): To control effort being
            published by the lateral_controller.

        # Published Topics
        - topic_to_port_motor (Int16): The port-side main motor input.
        - topic_to_strbrd_motor (Int16): The starboard-side main motor input.
        - topic_to_port_thruster (Int16): The port-side thruster input.
        - topic_to_strbrd_thruster (Int16): The starboard-side thruster input.
    """
    ###########################################################################
    def __init__(self):
        rospy.init_node('pid_interpreter_node', log_level=rospy.DEBUG)

        # Main Motor parameters
        self.output_lower_bound = rospy.get_param('~output_lower_bound', 0)
        self.output_upper_bound = rospy.get_param('~output_upper_bound', 254)
        self.output_maximum = rospy.get_param('~output_maximum', 127)
        self.port_motor_output_percentage = rospy.get_param(
            '~port_motor_output_percentage', 1.0
        )
        self.strbrd_motor_output_percentage = rospy.get_param(
            '~strbrd_motor_output_percentage', 1.0
        )

        # Thruster parameters
        self.output_lower_bound_lateral = rospy.get_param('~output_lower_bound_lateral', 1110)
        self.output_upper_bound_lateral = rospy.get_param('~output_upper_bound_lateral', 1890)
        self.output_maximum_lateral = rospy.get_param('~output_maximum_lateral', 390)
        self.port_thruster_output_percentage = rospy.get_param(
            '~port_thruster_output_percentage', 1.0
        )
        self.strbrd_thruster_output_percentage = rospy.get_param(
            '~strbrd_thruster_output_percentage', 1.0
        )

        # Set up subscribers
        self.heading_control_effort = 0.0
        rospy.Subscriber(
            'topic_from_heading_controller', Float64, self.heading_callback
        )
        self.speed_control_effort = 0.0
        rospy.Subscriber(
            'topic_from_speed_controller', Float64, self.speed_callback
        )
        self.lateral_control_effort = 0.0
        rospy.Subscriber(
            'topic_from_lateral_controller', Float64, self.lateral_callback
        )
        # Set up publishers
        self.port_motor_publisher = rospy.Publisher(
            'topic_to_port_motor', Int16, queue_size=1000
        )
        self.strbrd_motor_publisher = rospy.Publisher(
            'topic_to_strbrd_motor', Int16, queue_size=1000
        )
        self.port_thruster_publisher = rospy.Publisher(
            'topic_to_strbrd_thruster', Int16, queue_size=1000
        )
        self.strbrd_thruster_publisher = rospy.Publisher(
            'topic_to_strbrd_thruster', Int16, queue_size=1000
        )

    ###########################################################################
    def speed_callback(self, msg):
        """ Velocities fall in the range [-127,127] with -127 representing full
            reverse and 127 representing full forward
        """
        self.speed_control_effort = msg.data

    ###########################################################################
    def heading_callback(self, msg):
        """ Headings fall in the range [-127,127] with -127 representing maximum
            counter-clockwise force and 127 representing maximum clockwise force
        """
        self.heading_control_effort = msg.data

    ###########################################################################
    def lateral_callback(self, msg):
        """ Velocities fall in the range [-390,390] with -390 representing full
            reverse and 390 representing full forward
        """
        self.lateral_control_effort = msg.data

    ###########################################################################
    def apply_output_maximum(self, motor_speed):
        """ Returns +/- self.output_maximum if motor_speed is greater than it,
            and motor_speed otherwise
        """
        if abs(motor_speed) > self.output_maximum:
            return self.output_maximum * sign(motor_speed)
        else:
            return motor_speed

    ###########################################################################
    def apply_output_maximum_lateral(self, motor_speed):
        """ Returns +/- self.output_maximum if motor_speed is greater than it,
            and motor_speed otherwise
        """
        if abs(motor_speed) > self.output_maximum_lateral:
            return self.output_maximum_lateral * sign(motor_speed)
        else:
            return motor_speed

    ###########################################################################
    def prepare_for_publishing(self, motor_speed):
        """ Checks upper and lower bounds of motor_speed, and casts it to
            an integer
        """
        if motor_speed > self.output_upper_bound:
            val = self.output_upper_bound
        elif motor_speed < self.output_lower_bound:
            val = self.output_lower_bound
        else:
            val = motor_speed
        return int(val)

    ###########################################################################
    def prepare_for_publishing_lateral(self, motor_speed):
        """ Checks upper and lower bounds of motor_speed, and casts it to
            an integer
        """
        if motor_speed > self.output_upper_bound_lateral:
            val = self.output_upper_bound_lateral
        elif motor_speed < self.output_lower_bound_lateral:
            val = self.output_lower_bound_lateral
        else:
            val = motor_speed
        return int(val)

    ###########################################################################
    def process(self):
        """ Calculates and publishes the main motor and thruster input values.

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

            The thrusters work with values between [1110, 1890] with full
            forward being 1890 and full reverse being 1110. Since the thrusters
            are mounted opposite of one another, full port lateral motion
            requires the port motor to be full forward, and the starboard motor
            to be full reverse. The converse is also true.
        """

        # Get the control efforts
        heading_control_effort = self.heading_control_effort
        speed_control_effort = self.speed_control_effort
        lateral_control_effort = self.lateral_control_effort

        rospy.logdebug(
            'Received control efforts: {} (heading) {} (speed) {} (lateral)'
            .format(heading_control_effort, speed_control_effort, lateral_control_effort))

        ########################### MAIN MOTORS ################################
        # Allocate power based on angular velocity requirement first, and cap
        # at maximum if over.
        port_motor_speed = self.apply_output_maximum(
            heading_control_effort
        )
        strbrd_motor_speed = self.apply_output_maximum(
            -heading_control_effort
        )

        # Calculate the shift due to speed control effort
        max_abs = max(abs(port_motor_speed), abs(strbrd_motor_speed))
        max_shift = min(abs(speed_control_effort), self.output_maximum - max_abs)
        shift = 127
        if speed_control_effort > 0:
            shift += max_shift
        else:
            shift -= max_shift

        # Apply shift
        port_motor_speed += shift
        port_motor_speed *= self.port_motor_output_percentage
        port_motor_speed = self.prepare_for_publishing(port_motor_speed)

        strbrd_motor_speed += shift
        strbrd_motor_speed *= self.strbrd_motor_output_percentage
        strbrd_motor_speed = self.prepare_for_publishing(strbrd_motor_speed)

        ############################# THRUSTERS ################################

        port_thruster_speed = lateral_control_effort
        strbrd_thruster_speed = -lateral_control_effort

        lateral_shift = 1500

        port_thruster_speed += lateral_shift
        port_thruster_speed *= self.port_thruster_output_percentage
        port_thruster_speed = self.prepare_for_publishing_lateral(port_thruster_speed)

        strbrd_thruster_speed += lateral_shift
        strbrd_thruster_speed *= self.strbrd_thruster_output_percentage
        strbrd_thruster_speed = self.prepare_for_publishing_lateral(strbrd_thruster_speed)

        # Publish
        self.port_motor_publisher.publish(port_motor_speed)
        self.strbrd_motor_publisher.publish(strbrd_motor_speed)
        self.port_thruster_publisher.publish(port_thruster_speed)
        self.strbrd_thruster_publisher.publish(strbrd_thruster_speed)

        rospy.logdebug(
            'Published speeds: {} (port motor) {} (starboard motor) {} (port thruster) {} (starboard thruster)'
            .format(port_motor_speed, strbrd_motor_speed, port_thruster_speed, strbrd_thruster_speed))


###############################################################################
###############################################################################
def main():
    pid_interpreter = PIDInterpreter()
    # rospy.loginfo_once('Initialized the PID interpreter node')

    rate = rospy.Rate(
        rospy.get_param('~loop_rate', 10)
    )
    while not rospy.is_shutdown():
        pid_interpreter.process()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
