#! /usr/bin/env python
PKG='wolvmarine_drivers'

import unittest
import rospy
import PyKDL
import tf2_ros
import tf2_geometry_msgs

from tf2_ros import Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import PyKDL
import copy

from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped, PoseStamped, WrenchStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped, QuaternionStamped

def to_msg_msg(msg):
    return msg
tf2_ros.ConvertRegistration().add_to_msg(QuaternionStamped, to_msg_msg)
tf2_ros.ConvertRegistration().add_to_msg(TwistStamped, to_msg_msg)

def from_msg_msg(msg):
    return msg
tf2_ros.ConvertRegistration().add_from_msg(QuaternionStamped, from_msg_msg)

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
        twist.twist.linear.z)
    angular = kdl_tf * PyKDL.Vector(
        twist.twist.linear.x,
        twist.twist.linear.y,
        twist.twist.linear.z)
    res = TwistStamped()
    res.twist.linear.x = linear[0]
    res.twist.linear.y = linear[1]
    res.twist.linear.z = linear[2]
    res.twist.angular.x = angular[0]
    res.twist.angular.y = angular[1]
    res.twist.angular.z = angular[2]
    res.header = transform.header
    return res
tf2_ros.TransformRegistration().add(TwistStamped, do_transform_twist)

class GeometryMsgs(unittest.TestCase):
    def test_transform(self):
        b = tf2_ros.Buffer()
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.rotation.x = 1
        t.header.stamp = rospy.Time(2.0)
        t.header.frame_id = 'a'
        t.child_frame_id = 'b'
        b.set_transform(t, 'eitan_rocks')
        out = b.lookup_transform('a','b', rospy.Time(2.0), rospy.Duration(2.0))
        self.assertEqual(out.transform.translation.x, 1)
        self.assertEqual(out.transform.rotation.x, 1)
        self.assertEqual(out.header.frame_id, 'a')
        self.assertEqual(out.child_frame_id, 'b')

        v = TwistStamped()
        v.header.stamp = rospy.Time(2)
        v.header.frame_id = 'a'
        v.twist.linear.x = 0
        v.twist.linear.y = 1
        v.twist.linear.z = 0
        v.twist.angular.x = 0
        v.twist.angular.y = 1
        v.twist.angular.z = 0
        # out = do_transform_twist(v, t)
        out = b.transform(v, 'b')
        self.assertEqual(out.twist.linear.y, -1)
        self.assertEqual(out.twist.angular.y, -1)

        v = PointStamped()
        v.header.stamp = rospy.Time(2)
        v.header.frame_id = 'a'
        v.point.x = 1
        v.point.y = 2
        v.point.z = 3
        out = b.transform(v, 'b')
        self.assertEqual(out.point.x, 0)
        self.assertEqual(out.point.y, -2)
        self.assertEqual(out.point.z, -3)

        v = PoseStamped()
        v.header.stamp = rospy.Time(2)
        v.header.frame_id = 'a'
        v.pose.position.x = 1
        v.pose.position.y = 2
        v.pose.position.z = 3
        v.pose.orientation.x = 1
        out = b.transform(v, 'b')
        self.assertEqual(out.pose.position.x, 0)
        self.assertEqual(out.pose.position.y, -2)
        self.assertEqual(out.pose.position.z, -3)

        # Translation shouldn't affect Vector3
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.translation.y = 2
        t.transform.translation.z = 3
        t.transform.rotation.w = 1
        v = Vector3Stamped()
        v.vector.x = 1
        v.vector.y = 0
        v.vector.z = 0
        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, 1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

        # Rotate Vector3 180 deg about y
        t = TransformStamped()
        t.transform.translation.x = 1
        t.transform.translation.y = 2
        t.transform.translation.z = 3
        t.transform.rotation.y = 1

        v = Vector3Stamped()
        v.vector.x = 1
        v.vector.y = 0
        v.vector.z = 0

        out = tf2_geometry_msgs.do_transform_vector3(v, t)
        self.assertEqual(out.vector.x, -1)
        self.assertEqual(out.vector.y, 0)
        self.assertEqual(out.vector.z, 0)

        v = WrenchStamped()
        v.wrench.force.x = 1
        v.wrench.force.y = 0
        v.wrench.force.z = 0
        v.wrench.torque.x = 1
        v.wrench.torque.y = 0
        v.wrench.torque.z = 0

        out = tf2_geometry_msgs.do_transform_wrench(v, t)
        self.assertEqual(out.wrench.force.x, -1)
        self.assertEqual(out.wrench.force.y, 0)
        self.assertEqual(out.wrench.force.z, 0)
        self.assertEqual(out.wrench.torque.x, -1)
        self.assertEqual(out.wrench.torque.y, 0)
        self.assertEqual(out.wrench.torque.z, 0)

if __name__ == '__main__':
    import rosunit
    rospy.init_node('tests')
    rosunit.unitrun(PKG, 'tests', GeometryMsgs)
