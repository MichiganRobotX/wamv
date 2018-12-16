#! /usr/bin/env python

import rospy
import datetime
from wamv_msgs.msg import Heartbeat
from std_msgs.msg import Int16, String
from sensor_msgs.msg import NavSatFix

# import threading
import SocketServer
class TCPHandler(SocketServer.BaseRequestHandler):

   def handle(self):
      self.msg = self.request.recv(1024).strip()
      if self.msg == "on<EOF>":
         print "Turning On..."
         #ECHO "SUCCESS<EOF>"        <----- I need the server to echo back "success"
      if self.msg == "off<EOF>":
         print "Turning Off..."
         #ECHO "SUCCESS<EOF>"        <----- I need the server to echo back "success"

host, port = '192.168.1.103', 50000
# Create server, bind to local host and port
server = SocketServer.TCPServer((host,port),TCPHandler)

print "server is starting on ", host, port

# start server
server.serve_forever()



###############################################################################
class CRIOManager():

    ###########################################################################
    def __init__(self):
        rospy.init_node('crio_manager_node', log_level=rospy.INFO)

        self.publisher = rospy.Publisher(' ', , queue_size=100)

        rospy.Subscriber(' ', , self.callback)

        self.rate = rospy.Rate(1)

    ###########################################################################
    def callback(self, msg):
        self.msg.latitude = msg.latitude
        self.msg.longitude = msg.longitude

    ###########################################################################
    def loop(self):
        while not rospy.is_shutdown():
            self.publish_msg()
            self.rate.sleep();

###############################################################################
###############################################################################
if __name__ == '__main__':
    try:
        manager = CRIOManager()
        manager.loop()
    except rospy.ROSInterruptException:
        pass
