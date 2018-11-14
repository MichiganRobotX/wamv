#! /usr/bin/env python

import rospy
import tf2_ros
import transformations
import tf_conversions
import math
import sensor_msgs.msg
import geometry_msgs.msg

class Transformer():
    ROTATION = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2.0)

    def __init__(self):
        rospy.init_node('base_link_to_imu', log_level=rospy.INFO)

        rospy.Subscriber('imu', sensor_msgs.msg.Imu, self.callback)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.transform = geometry_msgs.msg.TransformStamped()
        self.transform.header.frame_id = rospy.get_param("~frame_id", 'base_link')
        self.transform.child_frame_id = rospy.get_param("~child_frame_id", 'imu')
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.0

    def callback(self, msg):
        self.transform.header.stamp = rospy.Time.now()
        rotation = tf_conversions.transformations.quaternion_multiply(
            (self.ROTATION[0], self.ROTATION[1], self.ROTATION[2], self.ROTATION[3]),
            (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        )
        self.transform.transform.rotation = geometry_msgs.msg.Quaternion(*rotation)
        self.broadcaster.sendTransform(self.transform)

if __name__ == '__main__':
    try:
        transformer = Transformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
