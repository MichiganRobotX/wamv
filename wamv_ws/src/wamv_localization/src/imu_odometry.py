#! /usr/bin/env python

import rospy
import tf2_ros
from math import pi, sin, cos, atan2, sqrt
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import PyKDL

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
class ImuOdometry():

    ###########################################################################
    def __init__(self):
        # Initialize node
        rospy.init_node('imu_odometry_node', log_level=rospy.INFO)

        self.footprint_to_stabilized = rospy.get_param("~footprint_to_stabilized", 1.0)
        self.broadcast_transform = rospy.get_param("~broadcast_transform", False)

        # Initialize class variables
        self.previous_time = None
        self.position = Vector3()

        self.msg = Odometry()
        self.msg.header.frame_id = rospy.get_param("~frame_id", "odom")
        self.msg.child_frame_id = rospy.get_param("~child_frame_id", "base_footprint")
        self.publisher = rospy.Publisher('odom', Odometry, queue_size=100)

        message_filters.TimeSynchronizer((
            message_filters.Subscriber('twist', TwistStamped),
            message_filters.Subscriber('imu', Imu)),
            1000
        ).registerCallback(self.callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

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
        current_time = twist_msg.header.stamp
        if self.previous_time is None:
            self.previous_time = current_time
            return

        # unpack
        ve, vn, vz = (twist_msg.twist.linear.x, twist_msg.twist.linear.y, twist_msg.twist.linear.z)
        wx, wy, wz = (twist_msg.twist.angular.x, twist_msg.twist.angular.y, twist_msg.twist.angular.z)
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

        self.msg.header.stamp = current_time
        self.msg.pose.pose.position = position
        self.msg.pose.pose.orientation = orientation
        self.msg.twist.twist.linear.x = vx
        self.msg.twist.twist.linear.y = vy
        self.msg.twist.twist.angular.z = wz

        self.publisher.publish(self.msg)

        if self.broadcast_transform:
            self.broadcaster.sendTransform(TransformStamped(
                odom.header,
                odom.child_frame_id,
                Transform(position, orientation)
            ))

        ##########################################
        ### base_footprint --> base_stabilized ###
        ##########################################
        header = self.msg.header
        header.frame_id = 'base_footprint'
        position = vector3(0.0, 0.0, self.footprint_to_stabilized - self.position.z)
        orientation = rpy_to_quat(0.0, 0.0, 0.0)

        self.broadcaster.sendTransform(TransformStamped(
            header,
            'base_stabilized',
            Transform(position, orientation)
        ))

        #####################################
        ### base_stabilized --> base_link ###
        #####################################
        header.frame_id = 'base_stabilized'
        position = vector3(0.0, 0.0, 0.0)
        # TODO: figure out what is up with this.
        orientation = rpy_to_quat(tx - 3.141529, ty, 0.0)

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
        odometry = ImuOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
