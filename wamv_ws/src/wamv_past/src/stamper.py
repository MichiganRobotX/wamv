#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, TwistStamped

class Stamper():
    def __init__(self):
        rospy.init_node('stamper_node', log_level=rospy.INFO)

        in_type, callback = {
            'Twist': (Twist, self.twist_callback),
            'TwistStamped': (TwistStamped, self.twist_callback),
            'Imu': (Imu, self.imu_callback),
            'NavSatFix': (NavSatFix, self.nav_sat_fix_callback)
        }[rospy.get_param("~in_type")]
        rospy.Subscriber('in', in_type, callback)

        out_type = {
            'Twist': Twist,
            'TwistStamped': TwistStamped,
            'Imu': Imu,
            'NavSatFix': NavSatFix
        }[rospy.get_param("~out_type")]
        self.publisher = rospy.Publisher('out', out_type, queue_size=100)

        self.msg = out_type()
        self.frame_id = rospy.get_param("~new_frame_id", None)

    def set_header(self, msg):
        self.msg.header.frame_id = self.frame_id or msg.header.frame_id
        self.msg.header.stamp = rospy.Time.now()

    def twist_callback(self, msg):
        self.set_header(msg)
        self.msg.twist = msg
        self.publish()

    def imu_callback(self, msg):
        self.set_header(msg)
        self.msg.header.seq = msg.header.seq
        self.msg.orientation = msg.orientation
        self.msg.orientation_covariance = msg.orientation_covariance
        self.msg.angular_velocity = msg.angular_velocity
        self.msg.angular_velocity_covariance = msg.angular_velocity_covariance
        self.msg.linear_acceleration = msg.linear_acceleration
        self.msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.publish()

    def nav_sat_fix_callback(self, msg):
        self.set_header(msg)
        self.msg.header.seq = msg.header.seq
        self.msg.status = msg.status
        self.msg.latitude = msg.latitude
        self.msg.longitude = msg.longitude
        self.msg.altitude = msg.altitude
        self.msg.position_covariance = msg.position_covariance
        self.msg.position_covariance_type = msg.position_covariance_type
        self.publish()

    def publish(self):
        self.publisher.publish(self.msg)

if __name__ == '__main__':
    try:
        stamper = Stamper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
