import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)

    # now we can do something with the PointCloud2 for example:
    # publish it
    pc_pub.publish(pc2_msg)

    # convert it to a generator of the individual points
    point_generator = pc2.read_points(pc2_msg)


    # we can access a generator in a loop
    sum = 0.0
    num = 0
    for point in point_generator:
        if not math.isnan(point[2])
            sum += point[2]
            num += 1
    # we can calculate the average z value for example
    print(str(sum/num))

    # or a list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)

    # we can access the point list with an index, each element is a namedtuple
    # we can access the elements by name, the generator does not yield namedtuples!
    # if we convert it to a list and back this possibility is lost
    print(point_list[len(point_list)/2].x)



rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()

laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud,listener_);

  // Do something with cloud.
}



#! /usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

###############################################################################
class OdometryPublisher():

    ###########################################################################
    def __init__(self):
        # Initialize node
        rospy.init_node('imu_odometry_estimation', log_level=rospy.INFO)

        # Get parameters
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.rot_cov = rospy.get_param("~rot_covariance", 99999.0)
        # self.sim = rospy.get_param("~sim", False)

        # Set up subscribers
        rospy.Subscriber('twist', Twist, self.twist_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)

        # Set up publisher
        self.odom_publisher = rospy.Publisher(
            'odometry/imu_estimated', Odometry, queue_size=100
        )

        # Initialize class variables
        self.odom = Odometry()
        self.odom.header.frame_id = self.frame_id
        self.odom.child_frame_id = self.child_frame_id
        # self.odom.pose.pose.position.x = 0.0
        # self.odom.pose.pose.position.y = 0.0
        # self.odom.pose.pose.position.z = 0.0

        self.previous_time = None

    ###########################################################################
    def imu_callback(self, msg):
        """ Collects quaternion coordinates from an_device/Imu

            Pose in this message corresponds to the estimated position of the
            robot in the odometric frame along with an optional covariance for
            the certainty of that pose estimate.
        """
        self.odom.header.stamp = msg.header.stamp
        if self.previous_time is None:
            self.previous_time = self.odom.header.stamp
            return

        quaternion = (
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )

        # self.odom.pose.pose.orientation.x = quaternion[0]
        # self.odom.pose.pose.orientation.y = quaternion[1]
        # self.odom.pose.pose.orientation.z = quaternion[2]
        # self.odom.pose.pose.orientation.w = quaternion[3]

        ori = tf.transformations.euler_from_quaternion(quaternion)
        quat = tf.transformations.quaternion_from_euler(0, 0, ori[2])
        self.odom.pose.pose.orientation.x = quat[0]
        self.odom.pose.pose.orientation.y = quat[1]
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]

        self.theta = ori[2]

        self.callback()

    ###########################################################################
    def twist_callback(self, msg):
        """ Collects linear and angular velocity data from an_device/Twist

            Twist in this message corresponds to the robot's velocity in the
            child frame, normally the coordinate frame of the mobile base,
            along with an optional covariance for the certainty of that
            velocity estimate.
        """
        self.odom.twist.twist.linear.x = msg.linear.x
        self.odom.twist.twist.linear.y = msg.linear.y
        # self.odom.twist.twist.linear.z = msg.linear.z
        # self.odom.twist.twist.angular.x = msg.angular.x
        # self.odom.twist.twist.angular.y = msg.angular.y
        # self.odom.twist.twist.angular.z = msg.angular.z

    ###########################################################################
    def callback(self):
        """
        """
        dt = (self.odom.header.stamp - self.previous_time).to_sec()
        self.odom.pose.pose.position.x += dt * (
            self.odom.twist.twist.linear.x * math.cos(self.theta)
          - self.odom.twist.twist.linear.y * math.sin(self.theta)
        )
        self.odom.pose.pose.position.y += dt * (
            self.odom.twist.twist.linear.x * math.sin(self.theta)
          + self.odom.twist.twist.linear.y * math.cos(self.theta)
        )
        # self.odom.pose.pose.position.z += dt * self.odom.twist.twist.linear.z

        self.odom_publisher.publish(self.odom)

        self.previous_time = self.odom.header.stamp

    ###########################################################################
    # def nav_sat_fix_callback(self, msg):
    #     if (msg.status.status == -1):
    #         rospy.loginfo("NO FIX.")
    #         return
    #
    #     if (msg.header.stamp == rospy.Time(0)):
    #         return
    #
    #     self.odom.header.stamp = msg.header.stamp
    #
    #     (northing, easting) = LLtoUTM(msg.latitude, msg.longitude)
    #     self.odom.pose.pose.position.x = easting #- ox
    #     self.odom.pose.pose.position.y = northing #- oy
    #     self.odom.pose.pose.position.z = 0.0
    #
    #     covariance = [
    #         msg.position_covariance[0],
    #         msg.position_covariance[1],
    #         msg.position_covariance[2],
    #         0, 0, 0,
    #         msg.position_covariance[3],
    #         msg.position_covariance[4],
    #         msg.position_covariance[5],
    #         0, 0, 0,
    #         msg.position_covariance[6],
    #         msg.position_covariance[7],
    #         msg.position_covariance[8],
    #         0, 0, 0,
    #         0, 0, 0,
    #         self.rot_cov, 0, 0,
    #         0, 0, 0,
    #         0, self.rot_cov, 0,
    #         0, 0, 0,
    #         0, 0, self.rot_cov
    #     ]
    #     self.odom.pose.covariance = covariance
    #
    #     # Publish odometry topic
    #     self.odom_pub.publish(self.odom)


###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        odometry = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
