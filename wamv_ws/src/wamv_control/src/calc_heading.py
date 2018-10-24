#! /usr/bin/env python

import sys
import rospy
import math
from std_msgs.msg import Int16

class TargetHeading():
    def __init__(self):
        # Initialize node
        rospy.init_node('calc_target_heading_node', log_level = rospy.DEBUG)
        # Set up subscribers
        ### CHECK THAT THESE SUBSCRIBERS ARE CORRECT!!!! SHOULD BE LINKED TO /odom?
        rospy.Subscriber('waypoint_loc', Int16, self.waypoint_callback)
        rospy.Subscriber('heading', Int16, self.heading_callback)
        rospy.Subscriber('loc', Int16, self.location_callback)
        # Set up publisher
        self.heading_publisher = rospy.Publisher('heading_to_target', Int16, queue_size=1000)
        # Initialize subscribed topic values
        ### CHECK THAT THESE ARE THE CORRECT FORMAT AND VALUES!!!!!
        self.waypoint_loc = (0.0,0.0)
        self.heading = 0.0
        self.location = (0.0,0.0)

    def waypoint_callback(self,msg):
        """ Waypoints are given as (x,y) locations in the global reference frame. This should be in the
        same format as the location_callback data.
        """
        self.waypoint_loc = msg.data

    def heading_callback(self,msg):
        """ Headings fall in the range (-180,180] with positive values corresponding to clockwise rotations.
        0 degrees = N
        90 degrees = E
        180 degrees = S
        270 degrees = W
        """
        self.heading = msg.data

    def location_callback(self,msg):
        """ Waypoints are given as (x,y) locations in the global reference frame.
        Should be in the same data format as waypoint_callback.
        """
        self.location = msg.data

    def calculate(self):
        """ Calculates the current target heading from the WAMV to a given waypoint, with the same sign convention as heading_callback.
        Once calculated - publish to ROS as 'heading_to_target' topic.
        """

        x_wamv = self.location[0]
        y_wamv = self.location[1]
        theta_wamv = self.heading

        x_wpt = self.waypoint_loc[0]
        y_wpt = self.waypoint_loc[1]

        dy = y_wpt - y_wamv
        dx = x_wpt - x_wamv

        # Could add in a 'round' if required.
        theta_wpt = math.degrees(math.atan2(dx,dy))
        
        self.heading_publisher.publish(theta_wpt)


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    tgt_heading = TargetHeading()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tgt_heading.calculate()
        rate.sleep()
