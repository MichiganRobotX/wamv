#! /usr/bin/env python

import rospy
import tf2_ros
import message_filters
from sensor_msgs.msg import Imu, NavSatFix
import PyKDL
import copy

import tf2_geometry_msgs
from geometry_msgs.msg import \
    Point, PointStamped, Transform, TransformStamped, \
    Quaternion, QuaternionStamped, Vector3, Vector3Stamped, \
    Pose, PoseStamped, PoseWithCovarianceStamped, \
    Twist, TwistStamped, TwistWithCovarianceStamped

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

def rpy_to_quat(r, p, y):
    rot = PyKDL.Rotation.RPY(r,p,y)
    x, y, z, w = rot.GetQuaternion()
    return Quaternion(x,y,z,w)

def quat_to_rpy(x, y, z, w):
    rot = PyKDL.Rotation.Quaternion(x, y, z, w)
    return rot.GetRPY()


class Fixer():
    def __init__(self):
        rospy.init_node('fixer_node', log_level=rospy.INFO)

        in_type, callback = {
            'Twist': (Twist, self.twist_callback),
            'TwistStamped': (TwistStamped, self.twist_callback),
            'Imu': (Imu, self.imu_callback),
            'NavSatFix': (NavSatFix, self.nav_sat_fix_callback)
        }[rospy.get_param('~in_type')]

        self.out_type = {
            'Twist': Twist,
            'TwistStamped': TwistStamped,
            'TwistWithCovarianceStamped': TwistWithCovarianceStamped,
            'PoseStamped': PoseStamped,
            'PoseWithCovarianceStamped': PoseWithCovarianceStamped,
            'Imu': Imu,
            'NavSatFix': NavSatFix
        }[rospy.get_param('~out_type')]

        if self.out_type == PoseStamped:
            callback = self.pose_callback
        elif self.out_type == PoseWithCovarianceStamped:
            callback = self.pose_w_cov_callback
        elif self.out_type == TwistWithCovarianceStamped:
            callback = self.twist_w_cov_callback

        self.old_frame = rospy.get_param('~old_frame', None)
        self.new_frame = rospy.get_param('~new_frame', None)

        rospy.Subscriber('in', in_type, callback)
        self.publisher = rospy.Publisher('out', self.out_type, queue_size=100)

        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer)

        self.broadcaster = tf2_ros.TransformBroadcaster()

    def twist_callback(self, msg):
        temp = self.out_type()
        temp.header.frame_id = self.old_frame
        temp.header.stamp = rospy.Time.now()
        temp.twist = msg

        try:
            tf_msg = self.buffer.transform(temp, self.new_frame)
        except:
            rospy.loginfo('Unable to receive transform. Waiting...')
            rospy.sleep(1)
            return

        self.publisher.publish(tf_msg)

    def twist_w_cov_callback(self, msg):
        temp = TwistStamped()
        temp.header.frame_id = self.old_frame
        temp.header.stamp = rospy.Time.now()
        temp.twist = msg

        try:
            tf_msg = self.buffer.transform(temp, self.new_frame)
        except:
            rospy.loginfo('Unable to receive transform. Waiting...')
            rospy.sleep(1)
            return

        temp = TwistWithCovarianceStamped()
        temp.header = tf_msg.header
        temp.twist.twist = tf_msg.twist

        self.publisher.publish(temp)

    def imu_callback(self, msg):
        temp = msg
        temp.header.frame_id = self.old_frame
        temp.header.stamp = rospy.Time.now()

        try:
            tf_msg = self.buffer.transform(temp, self.new_frame)
        except:
            rospy.loginfo('Unable to receive transform. Waiting...')
            rospy.sleep(1)
            return

        self.publisher.publish(tf_msg)

    def pose_callback(self, msg):
        pose = PoseStamped()
        temp = msg
        temp.header.frame_id = self.old_frame
        temp.header.stamp = rospy.Time.now()

        try:
            tf_msg = self.buffer.transform(temp, self.new_frame)
        except:
            rospy.loginfo('Unable to receive transform. Waiting...')
            rospy.sleep(1)
            return

        pose.header = tf_msg.header
        pose.pose.orientation = tf_msg.orientation

        # _, _, yaw = quat_to_rpy(
        #     tf_msg.orientation.x,
        #     tf_msg.orientation.y,
        #     tf_msg.orientation.z,
        #     tf_msg.orientation.w
        # )
        # quat = rpy_to_quat(0.0, 0.0, yaw)
        # header = pose.header
        # self.broadcaster.sendTransform(TransformStamped(
        #     header,
        #     'imu_compass',
        #     Transform(pose.pose.position, quat)
        # ))

        self.publisher.publish(pose)

    def pose_w_cov_callback(self, msg):
        temp = msg
        temp.header.frame_id = self.old_frame
        temp.header.stamp = rospy.Time.now()

        try:
            tf_msg = self.buffer.transform(temp, self.new_frame)
        except:
            rospy.loginfo('Unable to receive transform. Waiting...')
            rospy.sleep(1)
            return

        pose = PoseWithCovarianceStamped()
        pose.header = tf_msg.header
        pose.pose.pose.orientation = tf_msg.orientation

        self.publisher.publish(pose)

    def nav_sat_fix_callback(self, msg):
        temp = msg
        temp.header.frame_id = self.new_frame
        temp.header.stamp = rospy.Time.now()

        # try:
        #     tf_msg = self.buffer.transform(temp, self.new_frame)
        # except:
        #     rospy.loginfo('Unable to receive transform. Waiting...')
        #     rospy.sleep(1)
        #     return

        self.publisher.publish(temp)

if __name__ == '__main__':
    try:
        stamper = Fixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
