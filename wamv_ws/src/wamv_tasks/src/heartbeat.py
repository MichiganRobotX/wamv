#! /usr/bin/env python

import rospy
import datetime
from wamv_msgs.msg import Heartbeat
from standard_msgs.msg import Int32, String
from sensor_msgs.msg import NavSatFix

###############################################################################
class HeartbeatManager():

    ###########################################################################
    def __init__(self):
        rospy.init_node('heartbeat_manager_node', log_level=rospy.INFO)

        self.publisher = rospy.Publisher('heartbeat', Heartbeat, queue_size=100)
        # self.string_publisher = rospy.Publisher('heartbeat_string', String, queue_size=100)

        rospy.Subscriber('gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('system_mode', Int32, self.mode_callback)

		self.rate = rospy.rate(1)

		msg = Heartbeat()
		msg.id = '$RXHRB'
		msg.NS = 'N'
		msg.EW = 'E'
		msg.team_id = 'TBD'
		msg.auv_status = 1 # stowed
		self.msg = msg

    ###########################################################################
    def gps_callback(self, msg):
		self.msg.latitude = msg.latitude
		self.msg.longitude = msg.longitude

    ###########################################################################
    def mode_callback(self, msg):
		self.msg.system_mode = msg.data

	###########################################################################
	def set_msg(self):
		now = datetime.datetime.now()
		self.msg.time = now.strftime("%H%M%S")
		self.msg.date = now.strftime("%d%m%y")

        # The checksum is the bitwise exclusive OR of ASCII codes of all characters between the $ and *
        # TODO
		msg.checksum = -1

	###########################################################################
	def msg_to_str(self):
		return '{},{},{},{},{},{},{},{},{},{}*{}'.format(
			self.msg.id,
			self.msg.date,
			self.msg.time,
			self.msg.latitude,
			self.msg.NS,
			self.msg.longitude,
			self.msg.EW,
			self.msg.team_id,
			self.msg.system_mode,
			self.msg.auv_status,
			self.msg.checksum
		)

	###########################################################################
    def publish_msg(self):
		self.publisher.publish(self.msg)

	###########################################################################
    def loop(self):
		while not rospy.is_shutdown():
			self.set_msg()
			self.publish_msg()
			self.rate.sleep();


###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        manager = HeartbeatManager()
		manager.loop()
    except rospy.ROSInterruptException:
        pass
