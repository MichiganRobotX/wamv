// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>

// Set the Arduino Parameters
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Red  5 // swapped blue and red
#define Relay_Yellow  3
#define Relay_Green  4 // Not right pin as autonomy does not light up
#define Relay_Blue  2

// Setup Subscribers
void teleop (const std_msgs::Bool& isTeleop){
  std_msgs::Bool yellow_light = isTeleop;
  if (yellow_light.data) {
    digitalWrite(Relay_Yellow, RELAY_ON);// set the Relay ON
  }
  else {
    digitalWrite(Relay_Yellow, RELAY_OFF);// set the Relay OFF
  }
}

void connection (const std_msgs::Bool& isConnected){
  std_msgs::Bool blue_light = isConnected;
  if (blue_light.data) {
    digitalWrite(Relay_Blue, RELAY_ON);// set the Relay ON
  }
  else {
    digitalWrite(Relay_Blue, RELAY_OFF);// set the Relay OFF
  }
}

void autonomous (const std_msgs::Bool& isAuto){
  std_msgs::Bool green_light = isAuto;
  if (green_light.data) {
    digitalWrite(Relay_Green, RELAY_ON);// set the Relay ON
  }
  else {
    digitalWrite(Relay_Green, RELAY_OFF);// set the Relay OFF
  }
}

// ROS Input
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> rc_sub("remote_control_active", &teleop);
ros::Subscriber<std_msgs::Bool> network_sub("network_connection_active", &connection);
ros::Subscriber<std_msgs::Bool> autonomy_sub("autonomous_active", &autonomous);

// Setup Sequence
void setup() {
//-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_Red, RELAY_OFF);
  digitalWrite(Relay_Yellow, RELAY_OFF);
  digitalWrite(Relay_Green, RELAY_OFF);
  digitalWrite(Relay_Blue, RELAY_OFF);

//---( THEN set pins as outputs )----
  pinMode(Relay_Red, OUTPUT);
  pinMode(Relay_Yellow, OUTPUT);
  pinMode(Relay_Green, OUTPUT);
  pinMode(Relay_Blue, OUTPUT);

  delay(5000);

  nh.initNode();
  nh.subscribe(rc_sub);
  nh.subscribe(network_sub);
  nh.subscribe(autonomy_sub);
}

// Run Loop
void loop(){
  if (ros.ok()) {
      nh.spinOnce();
      delay(1000);
  }
  else {
      digitalWrite(Relay_Red, RELAY_ON);// set the Relay ON
      digitalWrite(Relay_Yellow, RELAY_OFF);
      digitalWrite(Relay_Green, RELAY_OFF);
      digitalWrite(Relay_Blue, RELAY_OFF);
  }
}
