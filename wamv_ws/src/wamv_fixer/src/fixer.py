#! /usr/bin/env python

import rospy
import tf2_ros
import message_filters
from sensor_msgs.msg import Imu, NavSatFix
import PyKDL
import copy

import tf2_geometry_msgs
from geometry_msgs.msg import \
    Pose, PoseStamped, Vector3, Vector3Stamped, \
    Point, PointStamped, Transform, TransformStamped, \
    Quaternion, QuaternionStamped, Twist, TwistStamped

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

# TwistStamped
def do_transform_twist(twist, transform):
    transform = copy.deepcopy(transform)
    kdl_tf = transform_to_kdl(transform)
    linear = kdl_tf * PyKDL.Vector(
        twist.twist.linear.x,
        twist.twist.linear.y,
        twist.twist.linear.z
    )
    angular = kdl_tf * PyKDL.Vector(
        twist.twist.angular.x,
        twist.twist.angular.y,
        twist.twist.angular.z
    )
    res = TwistStamped()
    res.header = transform.header
    res.twist.linear.x = linear[0]
    res.twist.linear.y = linear[1]
    res.twist.linear.z = linear[2]
    res.twist.angular.x = angular[0]
    res.twist.angular.y = angular[1]
    res.twist.angular.z = angular[2]
    return res
tf2_ros.TransformRegistration().add(TwistStamped, do_transform_twist)

# Imu
def do_transform_imu(imu_in, transform):
    transform = copy.deepcopy(transform)
    kdl_tf = transform_to_kdl(transform)
    quaternion = (kdl_tf.M * PyKDL.Rotation.Quaternion(
        imu_in.orientation.x,
        imu_in.orientation.y,
        imu_in.orientation.z,
        imu_in.orientation.w)).GetQuaternion()
    angular_velocity = kdl_tf * PyKDL.Vector(
        imu_in.angular_velocity.x,
        imu_in.angular_velocity.y,
        imu_in.angular_velocity.z)
    linear_acceleration = kdl_tf * PyKDL.Vector(
        imu_in.linear_acceleration.x,
        imu_in.linear_acceleration.y,
        imu_in.linear_acceleration.z)
    imu_out = Imu()
    imu_out.header = transform.header
    imu_out.orientation.x = quaternion[0]
    imu_out.orientation.y = quaternion[1]
    imu_out.orientation.z = quaternion[2]
    imu_out.orientation.w = quaternion[3]
    imu_out.angular_velocity.x = angular_velocity[0]
    imu_out.angular_velocity.y = angular_velocity[1]
    imu_out.angular_velocity.z = angular_velocity[2]
    imu_out.linear_acceleration.x = linear_acceleration[0]
    imu_out.linear_acceleration.y = linear_acceleration[1]
    imu_out.linear_acceleration.z = linear_acceleration[2]
    return imu_out
tf2_ros.TransformRegistration().add(Imu, do_transform_imu)



class Fixer():
    def __init__(self):
        rospy.init_node('fixer_node', log_level=rospy.INFO)

        type_, callback = {
            'Twist': (Twist, self.twist_callback),
            'TwistStamped': (TwistStamped, self.twist_callback),
            'Imu': (Imu, self.imu_callback),
            'NavSatFix': (NavSatFix, self.nav_sat_fix_callback)
        }[rospy.get_param('~type')]
        rospy.Subscriber('in', type_, callback)
        self.msg = type_()
        self.frame_id = rospy.get_param('~new_frame', None)

        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer)
        self.publisher = rospy.Publisher('out', type_, queue_size=100)

    def set_header(self, msg):
        self.msg.header.frame_id = self.frame_id or msg.header.frame_id
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq = msg.header.seq

    def twist_callback(self, msg):
        self.set_header(msg)
        self.msg.twist = msg
        self.publish()

    def imu_callback(self, msg):
        self.set_header(msg)
        self.msg.orientation = msg.orientation
        self.msg.orientation_covariance = msg.orientation_covariance
        self.msg.angular_velocity = msg.angular_velocity
        self.msg.angular_velocity_covariance = msg.angular_velocity_covariance
        self.msg.linear_acceleration = msg.linear_acceleration
        self.msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.publish()

    def nav_sat_fix_callback(self, msg):
        self.set_header(msg)
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
