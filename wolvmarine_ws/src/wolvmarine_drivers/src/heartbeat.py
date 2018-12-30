#! /usr/bin/env python

import rospy
import datetime
from wolvmarine_msgs.msg import Heartbeat
from std_msgs.msg import Int16, String
from sensor_msgs.msg import NavSatFix

def get_checksum(msg):
    start_pos = msg.find('$')
    end_pos = msg.find('*')
    result = ord(msg[start_pos + 1])
    for i in range(start_pos + 2, end_pos):
        result = result ^ ord(msg[i])
    return int(result)

###############################################################################
class HeartbeatManager():

    ###########################################################################
    def __init__(self):
        rospy.init_node('heartbeat_manager_node', log_level=rospy.INFO)

        self.publisher = rospy.Publisher('heartbeat', Heartbeat, queue_size=100)
        # self.string_publisher = rospy.Publisher('heartbeat_string', String, queue_size=100)

        rospy.Subscriber('gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('system_mode', Int16, self.mode_callback)

        self.rate = rospy.Rate(1)

        msg = Heartbeat()
        msg.id = '$RXHRB'
        msg.NS = 'N'
        msg.EW = 'E'
        msg.team_id = 'MICH'
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
    def publish_msg(self):
        now = datetime.datetime.now()
        self.msg.time = now.strftime("%H%M%S")
        self.msg.date = now.strftime("%d%m%y")

        msg = '{},{},{},{},{},{},{},{},{},{}*'.format(
            self.msg.id,
            self.msg.date,
            self.msg.time,
            self.msg.latitude,
            self.msg.NS,
            self.msg.longitude,
            self.msg.EW,
            self.msg.team_id,
            self.msg.system_mode,
            self.msg.auv_status
        )
        self.msg.checksum = get_checksum(msg)
        msg += str(self.msg.checksum)
        # msg += '<CR><LF>'def mode_callback(self, msg):

        self.publisher.publish(self.msg)
        # self.string_publisher.publish(msg)

    ###########################################################################
    def loop(self):
        while not rospy.is_shutdown():
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
