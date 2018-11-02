#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('converter_node', log_level=rospy.INFO)

    pub = rospy.Publisher('output_topic', Float64, queue_size = 1000)
    def twist_linear_x_callback(self, msg):
        pub.publish(Float64(msg.linear.x))

    def twist_angular_z_callback(self, msg):
        pub.publish(Float64(msg.angular.z))

    velocity = rospy.get_param('~velocity', 'linear')
    callback = {
        'linear': twist_linear_x_callback,
        'angular': twist_angular_z_callback
    }[velocity]
    rospy.Subscriber('input_topic', Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
