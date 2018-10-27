#!/usr/bin/env python

# WAM-V Teleoperation Controller

# Written by: Conner Goodrum
# Last Update: October 15, 2018

# Import Libraries
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

# Set Global "Activation" Boolean which will control if statements
activate = False
autonomy = False

# Function to actually read joystick and send commands
def sendCommands(data):

	# tell it we're using the global variable vs a local one
    global activate

    # Read in the joystick data
    posL_lat = data.axes[0]
    posR_lat = data.axes[3]
    posL = data.axes[1]
    posR = data.axes[4]

    # Remap the joystick
    powerL_lat = 1500 + 390 * posL_lat
    powerR_lat = 1500 - 390 * posR_lat
    powerL = 127 + 127 * posL
    powerR = 127 + 127 * posR


    # convert float to int
    speedL = int(powerL)
    speedR = int(powerR)
    speedL_lat = int(powerL_lat)
    speedR_lat = int(powerR_lat)

    # Publish the result
    pub1.publish(speedL)
    pub2.publish(speedR)
    pub3.publish(speedL_lat)
    pub4.publish(speedR_lat)


# Function that gets called by the Joystick Subscriber
def callback(data):

	# tell it we're using the global variable vs a local one
    global activate
    global autonomy

    # Check if Telelop has been activated
    if (data.buttons[7]==True) and (autonomy==False): # start button
        activate = True # set global boolean control var
        rospy.loginfo("Joystick Control Activated")
        pub5.publish(activate)

    if (data.buttons[7]==True) and (autonomy==True): # start button
        rospy.logerr("Error: Boat is running autonomously")
        activate = False

    # Check to see if Teleop is deactivated
    if data.buttons[6]==True: # back button
        activate = False # set global bool

        # Reset motors to be off
        speedL = 127
        speedR = 127
        speedL_lat = 1500
        speedR_lat = 1500
        pub1.publish(speedL)
        pub2.publish(speedR)
        pub3.publish(speedL_lat)
        pub4.publish(speedR_lat)
        pub5.publish(activate)
        rospy.loginfo("Joystick Control Deactivated")

    # If teleop is activated, call the function to process the commands
    if activate:
        sendCommands(data)


# Function to check Autonomoy Status
def check(bool_status):
    global autonomy
    if bool_status.data==True:
        autonomy = True
        print('Autonomous Active')
    if bool_status.data==False:
        autonomy = False
        print('Autonomous Not Active')

# Main Function
if __name__ == '__main__':

    # Setup ROS Nodes
    rospy.init_node('Teleop_Controller', anonymous=True, log_level=rospy.DEBUG)

    # Setup Publishers
    global pub1
    pub1 = rospy.Publisher('LmotorSpeed',Int16, queue_size=1000)
    global pub2
    pub2 = rospy.Publisher('RmotorSpeed',Int16, queue_size=1000)
    global pub3
    pub3 = rospy.Publisher('LmotorSpeed_lateral',Int16, queue_size=1000)
    global pub4
    pub4 = rospy.Publisher('RmotorSpeed_lateral',Int16, queue_size=1000)
    global pub5
    pub5 = rospy.Publisher('RemoteControlStatus',Bool, queue_size=1000)

    # Setup Subscriber
    rospy.Subscriber("joy", Joy, callback)
    rospy.Subscriber("AutonomyStatus", Bool, check)

    # Starts the Node
    rospy.spin()
