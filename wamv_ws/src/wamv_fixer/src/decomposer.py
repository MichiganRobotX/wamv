#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Converter():

    def __init__(self):
        self.current_heading = 0.0

        rospy.init_node('converter_node', log_level=rospy.INFO)

        velocity = rospy.get_param('~velocity', 'linear')
        if velocity == 'linear':
            callback = self.linear_callback
        elif velocity == 'angular':
            callback = self.angular_callback
        rospy.Subscriber('input_topic', Twist, callback)

        rospy.Subscriber('heading_topic', Float64, self.heading_callback)

        self.pub = rospy.Publisher('output_topic', Float64, queue_size = 1000)

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def linear_callback(self, msg):
        xvel = msg.linear.x
        yvel = msg.linear.y

        """ positive x going NORTH
            positive y going EAST
        """
        if self.current_heading > 45 and self.current_heading <= 135:
            if yvel > 0:
                is_moving_forward = True
            else:
                is_moving_forward = False

        elif self.current_heading > 135 and self.current_heading <= 225:
            if xvel < 0:
                is_moving_forward = True
            else:
                is_moving_forward = False

        elif self.current_heading > 225 and self.current_heading <= 315:
            if yvel < 0:
                is_moving_forward = True
            else:
                is_moving_forward = False

        elif self.current_heading > 315 or self.current_heading <= 45:
            if xvel > 0:
                is_moving_forward = True
            else:
                is_moving_forward = False

        speed = math.sqrt(xvel*xvel + yvel*yvel)
        if not is_moving_forward:
            speed *= -1.0

        self.pub.publish(speed)

    def angular_callback(self, msg):
        self.pub.publish(msg.angular.z)


if __name__ == '__main__':
    try:
        converter = Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
