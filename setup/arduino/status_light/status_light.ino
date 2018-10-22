// WAM-V Light controller
// Version 2.0
// Last Updated: September 14, 2018
// Owner: James Controller

// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>

// Set the Arduino Parameters
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Red  2
#define Relay_Orange  3
#define Relay_Green  4
#define Relay_Blue  5

// Define Global Variable for Red 
bool all_stop = 1; 

// Setup Subscribers
void teleop (const std_msgs::Bool& isTeleop){

  std_msgs::Bool yellow_light = isTeleop;

  // Tell Yellow Light if it should be on 
  if yellow_light
    all_stop = 0;
    digitalWrite(Relay_Orange, RELAY_ON);// set the Relay ON
  else
    digitalWrite(Relay_Orange, RELAY_OFF);// set the Relay OFF
}

void connection (const std_msgs::Bool& isConnected){

  std_msgs::Bool blue_light = isConnected;

  // Tell Blue Light if it should be on 
  if blue_light
    digitalWrite(Relay_Blue, RELAY_ON);// set the Relay ON
  else
    digitalWrite(Relay_Blue, RELAY_OFF);// set the Relay OFF
    all_stop = 1;
}

void autonomous (const std_msgs::Bool& isAuto){

  std_msgs::Bool green_light = isAuto;

  // Tell Green Light if it should be on
  if green_light
    digitalWrite(Relay_Green, RELAY_ON);// set the Relay ON
    all_stop = 0;
  else
    digitalWrite(Relay_Green, RELAY_OFF);// set the Relay OFF
}

void stop (const std_msgs::Bool& Stopped){

  std_msgs::Bool red_light = stopped;
  if all_stop
    red_light = 1;

  // Tell Blue Light if it should be on 
  if red_light
    digitalWrite(Relay_Red, RELAY_ON);// set the Relay ON
  else
    digitalWrite(Relay_Red, RELAY_OFF);// set the Relay OFF
}


// Setup Sequence
void setup() {
//-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_Red, RELAY_OFF);
  digitalWrite(Relay_Orange, RELAY_OFF);
  digitalWrite(Relay_Green, RELAY_OFF);
  digitalWrite(Relay_Blue, RELAY_OFF);

//---( THEN set pins as outputs )----
  pinMode(Relay_Red, OUTPUT);
  pinMode(Relay_Orange, OUTPUT);
  pinMode(Relay_Green, OUTPUT);
  pinMode(Relay_Blue, OUTPUT);
  delay(5000);

  // ROS Input
  ros::Nodehandle nh;
  ros::Subscriber<std_msgs::Bool> sub1("RemoteControlStatus", &teleop);
  ros::Subscriber<std_msgs::Bool> sub1("ConnectionStatus", &connection);
  ros::Subscriber<std_msgs::Bool> sub1("AutonomyStatus", &autonomous);
  ros::Subscriber<std_msgs::Bool> sub1("StopStatus", &stop);
}

// Run Loop
void loop(){
  double random = 125;
  if all_stop
    digitalWrite(Relay_Red, RELAY_ON);// set the Relay ON
  else
    digitalWrite(Relay_Red, RELAY_OFF);// set the Relay OFF
  nh.spinOnce();
  delay(1000);
}
