#! /usr/bin/env python

import rospy
import tf
import tf2_ros
from math import pi, sin, cos, atan2, sqrt
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
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

# QuaternionStamped
def do_transform_quaternion(quat_in, transform):
    transform = copy.deepcopy(transform)
    kdl_tf = transform_to_kdl(transform)
    quaternion = (kdl_tf.M * PyKDL.Rotation.Quaternion(
        quat_in.quaternion.x,
        quat_in.quaternion.y,
        quat_in.quaternion.z,
        quat_in.quaternion.w
    )).GetQuaternion()
    quat_out = QuaternionStamped()
    quat_out.header = transform.header
    quat_out.quaternion.x = quaternion[0]
    quat_out.quaternion.y = quaternion[1]
    quat_out.quaternion.z = quaternion[2]
    quat_out.quaternion.w = quaternion[3]
    return quat_out
tf2_ros.TransformRegistration().add(QuaternionStamped, do_transform_quaternion)

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

def vector3(x, y, z):
    return Vector3(x, y, z)

def rpy_to_quat(r, p, y):
    rot = PyKDL.Rotation.RPY(r,p,y)
    x, y, z, w = rot.GetQuaternion()
    return Quaternion(x,y,z,w)

def quat_to_rpy(x, y, z, w):
    rot = PyKDL.Rotation.Quaternion(x, y, z, w)
    return rot.GetRPY()


###############################################################################
class OdometryPublisher():

    ###########################################################################
    def __init__(self):
        # Initialize node
        rospy.init_node('imu_odometry_estimation', log_level=rospy.INFO)

        # Initialize class variables
        self.previous_time = None
        self.position = Vector3()

        self.odom = Odometry()
        self.odom.header.frame_id = rospy.get_param("~frame_id", "odom")
        self.odom.child_frame_id = rospy.get_param("~child_frame_id", "base_footprint")

        # Set up publisher
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer)
        self.odom_publisher = rospy.Publisher(
            'odom', Odometry, queue_size=1000
        )

        message_filters.TimeSynchronizer((
            message_filters.Subscriber('twist', TwistStamped),
            message_filters.Subscriber('imu', Imu)),
            1000
        ).registerCallback(self.callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

    ###########################################################################
    def callback(self, twist_msg, imu_msg):
        """ TODO

            Twist in this message corresponds to the robot's velocity in the
            child frame, normally the coordinate frame of the mobile base,
            along with an optional covariance for the certainty of that
            velocity estimate.

            Pose in this message corresponds to the estimated position of the
            robot in the odometric frame along with an optional covariance for
            the certainty of that pose estimate.
        """
        current_time = twist_msg.header.stamp
        if self.previous_time is None:
            self.previous_time = current_time
            return

        try:
            twist = self.buffer.transform(twist_msg, 'base_link')
            imu = self.buffer.transform(imu_msg, 'base_link')
        except:
            rospy.loginfo('Unable to receive transform at time {}'.format(current_time))
            return

        # unpack
        ve, vn, vz = (twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z)
        wx, wy, wz = (twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z)
        orientation = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        qx, qy, qz, qw = orientation

        # reorganize
        rotation = quat_to_rpy(*orientation)
        tx, ty, tz = rotation

        vx = ve*cos(tz) + vn*sin(tz)
        vy = ve*sin(tz) + vn*cos(tz)

        # calculate change in position
        dt = (current_time - self.previous_time).to_sec()
        self.position.x = self.position.x + (dt * ve)
        self.position.y = self.position.y + (dt * vn)
        self.position.z = self.position.z + (dt * vz)
        # self.position.y += dt * vn
        # self.position.z += dt * vz

        ###############################
        ### odom --> base_footprint ###
        ###############################
        position = vector3(copy.deepcopy(self.position.x), copy.deepcopy(self.position.y), 0.0)
        orientation = rpy_to_quat(0.0, 0.0, tz)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position = position
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_publisher.publish(odom)

        self.broadcaster.sendTransform(TransformStamped(
            odom.header,
            odom.child_frame_id,
            Transform(position, orientation)
        ))

        ##########################################
        ### base_footprint --> base_stabilized ###
        ##########################################
        set_dist = 1.0
        position = vector3(0.0, 0.0, set_dist + self.position.z)
        orientation = rpy_to_quat(0.0, 0.0, 0.0)

        header = odom.header
        header.frame_id = 'base_footprint'

        self.broadcaster.sendTransform(TransformStamped(
            header,
            'base_stabilized',
            Transform(position, orientation)
        ))

        #####################################
        ### base_stabilized --> base_link ###
        #####################################
        position = vector3(0.0, 0.0, 0.0)
        orientation = rpy_to_quat(tx - 3.141529, ty, 0.0)

        header = odom.header
        header.frame_id = 'base_stabilized'

        self.broadcaster.sendTransform(TransformStamped(
            header,
            'base_link',
            Transform(position, orientation)
        ))

        self.previous_time = current_time

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        odometry = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
