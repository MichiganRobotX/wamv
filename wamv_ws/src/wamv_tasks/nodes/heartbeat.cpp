/*
Node to listen to LiDAR, Camera, GPS and controller
messages and publish signal accordingly for heartbeat
*/
#include <ros/ros.h>
#include <wamv_msgs/Heartbeat.h>
#include <std_msgs/String.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>

#include <sstream>
#include <bits/stdc++.h>

using namespace std;
using namespace ros;

///////////////////////
/* Global Variables*/
///////////////////////

// signal publisher
Publisher pub_signal;
Publisher pub_message;

// subscriber for each of the sensors
Subscriber sub_gps;
Subscriber sub_lidar;
Subscriber sub_camera;
Subscriber sub_controller;
Subscriber sub_laptop;

bool gps = false;
bool lidar = false;
bool camera = false;
bool controller = false;
bool laptop = false;

wamv_msgs::Heartbeat msg;
std_msgs::String final;

const string msgID = "$RXHRB";
const string teamID = "AUVS9";
const int checksum = 06;

/////////////////////////
/* Temporary Parameters*/
/////////////////////////
float prevLat, prevLong;


///////////////////////
/* Method Definitions*/
///////////////////////

/*
listeners for each message type that is published.
If message is heard on respective topic, then flag
is set to true, else after every loop, flag is set
to false. rate at which message is published is 2Hz.
*/
void listener_gps(const sensor_msgs::NavSatFix &msg1){
	gps = true;
	msg.latitude = msg1.latitude;
	msg.longitude = msg1.longitude;

	if(msg.latitude - prevLat >= 0)
		msg.NS = "N";
	else
		msg.NS = "S";

	if(msg.longitude - prevLong >= 0)
		msg.EW = "E";
	else
		msg.EW = "W";

	prevLat = msg.latitude;
	prevLong = msg.longitude;
}

void listener_lidar(const sensor_msgs::LaserScan &msg1){ lidar = true; }

void listener_camera(const sensor_msgs::CameraInfo &msg1){ camera = true; }

void listener_controller(const sensor_msgs::JointState &msg1){ controller = true; }

void listener_laptop(const sensor_msgs::JointState &msg1){ laptop = true; }


///////////////////////////////
/* Message Generation Methods*/
///////////////////////////////
/*
See time documentation library here:
https://www.tutorialspoint.com/c_standard_library/c_function_localtime.htm
*/

/*
Method to format number to 2 digits
For example, '1' will be returned as
'01' and '46' as '46' itself
*/
string formatNum(int number) {
    stringstream ss;
    ss << std::setw(2) << std::setfill('0') << number;
    return ss.str();
}

/*
Template function to convert any
datatype to string. std::to_string()
works with newer verions of C++ (11+)
*/

template <typename T>
string to_string(T val)
{
    stringstream ss;
    ss << val;
    return ss.str();
}


/*
Method to register Timestamp to msg
*/
void setTimestamp() {
    time_t now = time(0);
    struct tm tstruct;
    tstruct = *localtime(&now);
    int sec = tstruct.tm_sec; int min = tstruct.tm_min; int hr = tstruct.tm_hour;
    msg.time = formatNum(hr) + formatNum(min) + formatNum(sec);
}

/*
Method to register current date to msg
*/
void setDate() {
    time_t now = time(0);
    struct tm tstruct;
    tstruct = *localtime(&now);
    int dd = tstruct.tm_mday; int mm = tstruct.tm_mon + 1; int yy = tstruct.tm_year;
    msg.date =  formatNum(dd) + formatNum(mm) + formatNum(yy%100);
}

/*
Method to register current date to msg
*/
void setParams() {
	msg.msgID = msgID;
	msg.checksum = checksum;
	msg.teamID = teamID;
}


/*
Method to convert heartbeatmsg to finalMsg and display
*/
void displayMsg() {
	final.data = msg.msgID + ',' + msg.date + ',' + msg.time + ',' + \
		to_string(msg.latitude) + ',' + msg.NS + ',' + to_string(msg.longitude) + ',' + msg.EW + ',' + msg.teamID + \
		',' + to_string(msg.sysMode) + ',' + to_string(msg.AUVStatus) + '*' + \
		formatNum(msg.checksum);

	ROS_INFO_STREAM(final.data);
}

//////////////////////////
/* Method Main Heartbeat*/
//////////////////////////

int main(int argc, char **argv) {
	init(argc, argv, "heartbeat");
	NodeHandle nh;

	// subscribe to all topics being published
	sub_gps = nh.subscribe("/an_device/NavSatFix", 10000, &listener_gps);
	sub_lidar = nh.subscribe("/vel1/scan", 10000, &listener_lidar);
	sub_camera = nh.subscribe("/ladybug_camera/camera0/camera_info", 10000, &listener_camera);
	sub_controller = nh.subscribe("/controller_data", 10000, &listener_controller);
	sub_laptop = nh.subscribe("/laptop_data", 10000, &listener_laptop);

	// publish signal accordingly
	pub_signal = nh.advertise<wamv_msgs::Heartbeat>("/heartbeat", 10000);
	pub_message = nh.advertise<std_msgs::String>("/messages/heartbeat", 10000);

	// rate set to 1 because camera frequency <1.5
	Rate rate(1);

	while(ok()){
		ROS_INFO_STREAM("controller: "<<controller<<" | laptop: "<<laptop<<" | gps: "<<gps<<" | camera: "<<camera<<" | lidar: "<<lidar);

		/*******************************************
		Set Time Stamp, Date, msgID, teamID,checksum
		*******************************************/
		setTimestamp();
		setDate();
		setParams();

		/*****************
		Set AUVStatus here
		******************/
		// set states for the signal
		if(controller==false || laptop==false)
			msg.AUVStatus = 1;
		if(controller==true && laptop==true){
			msg.AUVStatus = 2;
			if(gps==true && lidar==true && camera==true){
				msg.AUVStatus = 3;
				// if controller button switched to autonmous set msg.AUVStatus = 4 here
			}
		}

		// produce appropriate state message
		if (msg.AUVStatus == 1)
			ROS_INFO("Controller and/or Laptop cannot be found on the network");
		else if (msg.AUVStatus == 2)
			ROS_INFO("Controller and/or Laptop found on network. Boat activated to Manual Mode.");
		else if (msg.AUVStatus == 3)
			ROS_INFO("All Sensors connected and boat operating in Manual Mode");
		else
			ROS_INFO("Boat operating in autonomous mode");

		// publish signal
		pub_signal.publish(msg);
		pub_message.publish(final);

		// reset states
		gps = false; laptop = false; controller = false; camera = false; lidar = false;

		// delay
		rate.sleep();

		displayMsg();

		spinOnce();
	}

}
