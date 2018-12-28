#! /usr/bin/env python

import rospy
import tf2_ros
from math import pi, sin, cos, atan2, sqrt
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import PyKDL

import tf2_geometry_msgs
from geometry_msgs.msg import \
    Point, PointStamped, Transform, TransformStamped, \
    Quaternion, QuaternionStamped, Vector3, Vector3Stamped, \
    Pose, PoseStamped, PoseWithCovarianceStamped, \
    Twist, TwistStamped, TwistWithCovarianceStamped

def vector3(x, y, z):
    return Point(x, y, z)

        # try:
        #     tf_msg = self.buffer.transform(temp, self.new_frame)
        # except:
        #     rospy.loginfo('Unable to receive transform. Waiting...')
        #     rospy.sleep(1)
        #     return

def rpy_to_quat(r, p, y):
    rot = PyKDL.Rotation.RPY(r,p,y)
    x, y, z, w = rot.GetQuaternion()
    return Quaternion(x,y,z,w)

def quat_to_rpy(x, y, z, w):
    rot = PyKDL.Rotation.Quaternion(x, y, z, w)
    return rot.GetRPY()


###############################################################################
class ImuOdometry():

    ###########################################################################
    def __init__(self):
        # Initialize node
        rospy.init_node('imu_odometry_node', log_level=rospy.INFO)

        self.footprint_to_stabilized = rospy.get_param("~footprint_to_stabilized", 1.0)
        self.broadcast_transform = rospy.get_param("~broadcast_transform", False)
        self.two_d_mode = rospy.get_param("~two_d_mode", True)

        # Initialize class variables
        self.previous_time = None
        self.position = vector3(0,0,0)

        self.publisher = rospy.Publisher('odom', Odometry, queue_size=100)

        message_filters.ApproximateTimeSynchronizer((
            message_filters.Subscriber('twist', TwistStamped),
            message_filters.Subscriber('imu', Imu)),
            1000, 0.05, allow_headerless=True
        ).registerCallback(self.callback)

        # message_filters.TimeSynchronizer((
        #     message_filters.Subscriber('twist', TwistStamped),
        #     message_filters.Subscriber('imu', Imu)),
        #     10
        # ).registerCallback(self.callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        # self.seq = 0

    ###########################################################################
    def callback(self, twist_msg, imu_msg):
        """ Twist in this message corresponds to the robot's velocity in the
            child frame, normally the coordinate frame of the mobile base,
            along with an optional covariance for the certainty of that
            velocity estimate.

            Pose in this message corresponds to the estimated position of the
            robot in the odometric frame along with an optional covariance for
            the certainty of that pose estimate.
        """
        current_time = rospy.Time.now()
        # current_time = imu_msg.header.stamp
        # current_time = twist_msg.header.stamp
        if self.previous_time is None:
            self.previous_time = current_time
            return

        # unpack
        ve, vn, vz = (twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z)
        wx, wy, wz = (twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z)

        # ve, vn, vz = (twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z)
        # wx, wy, wz = (twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z)

        orientation = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        roll, pitch, yaw = quat_to_rpy(*orientation)

        vx = ve*cos(yaw) + vn*sin(yaw)
        vy = ve*sin(yaw) + vn*cos(yaw)

        # calculate change in position
        dt = (current_time - self.previous_time).to_sec()
        self.position.x += dt * ve
        self.position.y += dt * vn
        self.position.z += dt * vz

        ###############################
        ### odom --> base_footprint ###
        ###############################
        position = vector3(self.position.x, self.position.y, 0.0)
        orientation = rpy_to_quat(0.0, 0.0, yaw)

        msg = Odometry()
        # msg.header.stamp = current_time
        msg.header = Header(imu_msg.header.seq, rospy.Time.now(), 'odom')
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position = position
        msg.pose.pose.orientation = orientation
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz


        self.publisher.publish(msg)

        if self.broadcast_transform:
            self.broadcaster.sendTransform(TransformStamped(
                msg.header,
                msg.child_frame_id,
                Transform(position, orientation)
            ))

        ##########################################
        ### base_footprint --> base_stabilized ###
        ##########################################
        z_position = self.footprint_to_stabilized
        if not self.two_d_mode:
            z_position -= self.position.z
        position = vector3(0.0, 0.0, z_position)
        orientation = rpy_to_quat(0.0, 0.0, 0.0)

        self.broadcaster.sendTransform(TransformStamped(
            Header(imu_msg.header.seq, rospy.Time.now(), 'base_footprint'),
            'base_stabilized',
            Transform(position, orientation)
        ))

        #####################################
        ### base_stabilized --> base_link ###
        #####################################
        position = vector3(0.0, 0.0, 0.0)
        # TODO: figure out what is up with this.
        orientation = rpy_to_quat(roll - 3.141529, pitch, 0.0)
        # orientation = rpy_to_quat(roll, pitch, 0.0)

        self.broadcaster.sendTransform(TransformStamped(
            Header(imu_msg.header.seq, rospy.Time.now(), 'base_stabilized'),
            'base_link',
            Transform(position, orientation)
        ))

        self.previous_time = current_time
        # self.seq += 1

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        odometry = ImuOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
