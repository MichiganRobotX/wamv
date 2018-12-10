// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Set the Arduino Parameters
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Red  5
#define Relay_Yellow  3
#define Relay_Green  4
#define Relay_Blue  2

// Setup Subscribers
int time_since_heartbeat = 0;
void heartbeat_callback (const std_msgs::String& msg) {
    time_since_heartbeat = 0;
}

bool activate_yellow = 0;
void remote_control_status_callback (const std_msgs::Bool& yellow_light) {
    if (yellow_light.data) { activate_yellow = 1; }
    else { activate_yellow = 0; }
}

bool activate_blue = 0;
void network_connection_callback (const std_msgs::Bool& blue_light) {
    if (blue_light.data) { activate_blue = 1; }
    else { activate_blue = 0; }
}

bool activate_green = 0;
void autonomy_status_callback (const std_msgs::Bool& green_light) {
    if (green_light.data) { activate_green = 1; }
    else { activate_green = 0; }
}

bool activate_red = 0;

// void remote_control_status_callback (const std_msgs::Bool& yellow_light) {
//   if (yellow_light.data) { digitalWrite(Relay_Yellow, RELAY_ON); }
//   else { digitalWrite(Relay_Yellow, RELAY_OFF); }
// }
//
// void network_connection_callback (const std_msgs::Bool& blue_light) {
//   if (blue_light.data) { digitalWrite(Relay_Blue, RELAY_ON); }
//   else { digitalWrite(Relay_Blue, RELAY_OFF); }
// }
//
// void autonomy_status_callback (const std_msgs::Bool& green_light) {
//   if (green_light.data) { digitalWrite(Relay_Green, RELAY_ON); }
//   else { digitalWrite(Relay_Green, RELAY_OFF); }
// }

// ROS Input
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> rc_sub("remote_control_active", &remote_control_status_callback);
ros::Subscriber<std_msgs::Bool> network_sub("network_connection_active", &network_connection_callback);
ros::Subscriber<std_msgs::Bool> autonomy_sub("autonomous_active", &autonomy_status_callback);
ros::Subscriber<std_msgs::String> heartbeat_sub("heartbeat_string", &heartbeat_callback);

// Setup Sequence
void setup() {
  digitalWrite(Relay_Red, RELAY_OFF);
  digitalWrite(Relay_Yellow, RELAY_OFF);
  digitalWrite(Relay_Green, RELAY_OFF);
  digitalWrite(Relay_Blue, RELAY_OFF);

  pinMode(Relay_Red, OUTPUT);
  pinMode(Relay_Yellow, OUTPUT);
  pinMode(Relay_Green, OUTPUT);
  pinMode(Relay_Blue, OUTPUT);

  delay(5000);

  nh.initNode();
  nh.subscribe(rc_sub);
  nh.subscribe(network_sub);
  nh.subscribe(autonomy_sub);
  nh.subscribe(heartbeat_sub);
}

// Run Loop
int MAX_TIME = 2;
void loop(){
  nh.spinOnce();

  if (time_since_heartbeat < MAX_TIME) {
      ++time_since_heartbeat;

      digitalWrite(Relay_Red, RELAY_OFF);

      if (activate_yellow == 1) {
          digitalWrite(Relay_Yellow, RELAY_ON);
      } else {
          digitalWrite(Relay_Yellow, RELAY_OFF);
      }

      if (activate_green == 1) {
          digitalWrite(Relay_Green, RELAY_ON);
      } else {
          digitalWrite(Relay_Green, RELAY_OFF);
      }

      if (activate_blue == 1) {
          digitalWrite(Relay_Blue, RELAY_ON);
      } else {
          digitalWrite(Relay_Blue, RELAY_OFF);
      }
  }
  else {
      digitalWrite(Relay_Red, RELAY_ON);

      digitalWrite(Relay_Yellow, RELAY_OFF);
      digitalWrite(Relay_Green, RELAY_OFF);
      digitalWrite(Relay_Blue, RELAY_OFF);
  }

  delay(1000);
}
