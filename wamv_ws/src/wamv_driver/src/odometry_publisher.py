#! /usr/bin/env python

"""
NavSatStatus Message:

    int8 STATUS_NO_FIX=-1
    int8 STATUS_FIX=0
    int8 STATUS_SBAS_FIX=1
    int8 STATUS_GBAS_FIX=2
    uint16 SERVICE_GPS=1
    uint16 SERVICE_GLONASS=2
    uint16 SERVICE_COMPASS=4
    uint16 SERVICE_GALILEO=8
    int8 status
    uint16 service

"""

import rospy
from math import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

###############################################################################
def LLtoUTM(Lat, Long):
    RADIANS_PER_DEGREE = pi/180.0;

    # WGS84 Parameters
    WGS84_A = 6378137.0       # major axis
    WGS84_E = 0.0818191908    # first eccentricity

    # UTM Parameters
    UTM_K0 = 0.9996               # scale factor
    UTM_E2 = (WGS84_E*WGS84_E)    # e^2

    a = WGS84_A
    eccSquared = UTM_E2
    k0 = UTM_K0

    # Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long + 180.0) - int( (Long+180.) / 360. )*360. - 180.

    LatRad = Lat * RADIANS_PER_DEGREE
    LongRad = LongTemp * RADIANS_PER_DEGREE
    ZoneNumber = int( (LongTemp + 180.0) / 6.0 ) + 1

    if Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0:
        ZoneNumber = 32
        # Special zones for Svalbard

    if Lat >= 72.0 and Lat < 84.0:
        if LongTemp >= 0.0 and LongTemp < 9.0:
            ZoneNumber = 31
        elif LongTemp >= 9.0 and LongTemp < 21.0:
            ZoneNumber = 33
        elif LongTemp >= 21.0 and LongTemp < 33.0:
            ZoneNumber = 35
        elif LongTemp >= 33.0 and LongTemp < 42.0:
            ZoneNumber = 37

    # +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1.0)*6.0 - 180.0 + 3.0
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE

    eccPrimeSquared = eccSquared / (1.0 - eccSquared)
    N = a / sqrt(1 - eccSquared*sin(LatRad)*sin(LatRad))
    T = tan(LatRad)*tan(LatRad)
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad)
    A = cos(LatRad)*(LongRad - LongOriginRad)

    M = ( (1 - eccSquared/4.0 - 3.0*eccSquared*eccSquared/64.0 - 5.0*eccSquared*eccSquared*eccSquared/256.0) * LatRad
        - (3.0*eccSquared/8.0 + 3.0*eccSquared*eccSquared/32.0 + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(2.0*LatRad)
        + (15.0*eccSquared*eccSquared/256.0 + 45.0*eccSquared*eccSquared*eccSquared/1024.0)*sin(4.0*LatRad)
        - (35.0*eccSquared*eccSquared*eccSquared/3072.0)*sin(6.0*LatRad) )*a

    UTMEasting = (k0*N*(A + (1.0 - T + C)*A*A*A / 6.0 + (5.0 - 18.0*T + T*T + 72*C - 58.0*eccPrimeSquared)*A*A*A*A*A / 120.0) + 500000.0)

    UTMNorthing = (k0*(M + N*tan(LatRad)*(A*A / 2.0 + (5.0 - T + 9.0*C + 4.0*C*C)*A*A*A*A / 24.0 + (61.0 - 58.0*T + T*T + 600.0*C - 330.0*eccPrimeSquared)*A*A*A*A*A*A / 720.0)))

    if Lat < 0:
      # 10000000 meter offset for southern hemisphere
      UTMNorthing += 10000000.0

    return (UTMNorthing, UTMEasting)

###############################################################################
class OdometryPublisher():

    ###########################################################################
    def __init__(self):
        # Initialize node
        rospy.init_node('odometry_publisher_node', log_level = rospy.INFO)

        # Set up subscribers
        rospy.Subscriber('twist', Twist, self.twist_callback)
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('navsatfix', NavSatFix, self.nav_sat_fix_callback)

        # Set up publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1000)

        # Get parameters
        self.frame_id = rospy.get_param("~frame_id", "an_device")
        self.child_frame_id = rospy.get_param("~child_frame_id", "an_device")
        self.rot_cov = rospy.get_param("~rot_covariance", 99999.0)
        self.sim = rospy.get_param("~sim", False)

        # Initialize class variables
        self.odom = Odometry()
        self.imu = Imu()
        self.nsf = NavSatFix()


    ###########################################################################
    def imu_callback(self, msg):
        """ Collects quaternion coordinates from an_device/Imu
        """
        self.odom.pose.pose.orientation.w = msg.orientation.w
        self.odom.pose.pose.orientation.x = msg.orientation.x
        self.odom.pose.pose.orientation.y = msg.orientation.y
        self.odom.pose.pose.orientation.z = msg.orientation.z

    ###########################################################################
    def twist_callback(self, msg):
        """ Collects linear and angular velocity data from an_device/Twist
        """
        self.odom.twist.twist.linear.x = msg.linear.x
        self.odom.twist.twist.linear.y = msg.linear.y
        self.odom.twist.twist.linear.z = msg.linear.z

        self.odom.twist.twist.angular.x = msg.angular.x
        self.odom.twist.twist.angular.y = msg.angular.y
        self.odom.twist.twist.angular.z = msg.angular.z

    ###########################################################################
    def nav_sat_fix_callback(self, msg):
        pos_covariance = []
        if (msg.status.status == -1):
            rospy.loginfo("NO FIX.")
            return

        if (msg.header.stamp == rospy.Time(0)):
            return

        self.odom.header.stamp = msg.header.stamp
        self.odom.header.frame_id = self.frame_id
        self.odom.child_frame_id = self.child_frame_id

        (northing, easting) = LLtoUTM(msg.latitude, msg.longitude)
        self.odom.pose.pose.position.x = easting #- ox
        self.odom.pose.pose.position.y = northing #- oy
        self.odom.pose.pose.position.z = 0.0

        covariance = [
            msg.position_covariance[0],
            msg.position_covariance[1],
            msg.position_covariance[2],
            0, 0, 0,
            msg.position_covariance[3],
            msg.position_covariance[4],
            msg.position_covariance[5],
            0, 0, 0,
            msg.position_covariance[6],
            msg.position_covariance[7],
            msg.position_covariance[8],
            0, 0, 0,
            0, 0, 0,
            self.rot_cov, 0, 0,
            0, 0, 0,
            0, self.rot_cov, 0,
            0, 0, 0,
            0, 0, self.rot_cov
        ]
        self.odom.pose.covariance = covariance

        # Publish odometry topic
        self.odom_pub.publish(self.odom)


###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        odometry = OdometryPublisher()
        # rospy.loginfo_once('Initialized odometry publisher node')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
