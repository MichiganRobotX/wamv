// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>

// Set the Arduino Parameters
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Red  5 // swapped blue and red
#define Relay_Orange  3
#define Relay_Green  4 // Not right pin as autonomy does not light up
#define Relay_Blue  2

// Define Global Variable for Red
bool all_stop = 1;

// Define global for connection loss
// If light has not received any messages in the last *MAX_TIME* seconds, signal red light
#define MAX_TIME 3 // max time limit
int done = 0;


// Setup Subscribers
void teleop (const std_msgs::Bool& isTeleop){

  std_msgs::Bool yellow_light = isTeleop;

  done = 0;

  // Tell Yellow Light if it should be on
  if (yellow_light.data){
    all_stop = 0;
    digitalWrite(Relay_Orange, RELAY_ON);// set the Relay ON
  } else {
    digitalWrite(Relay_Orange, RELAY_OFF);// set the Relay OFF
  }
}

void connection (const std_msgs::Bool& isConnected){

  std_msgs::Bool blue_light = isConnected;

  done = 0;

  // Tell Blue Light if it should be on
  if (blue_light.data) {
    digitalWrite(Relay_Blue, RELAY_ON);// set the Relay ON
  } else {
    digitalWrite(Relay_Blue, RELAY_OFF);// set the Relay OFF
    all_stop = 1;
  }
}

void autonomous (const std_msgs::Bool& isAuto){

  std_msgs::Bool green_light = isAuto;

  done = 0;

  // Tell Green Light if it should be on
  if (green_light.data){
    digitalWrite(Relay_Green, RELAY_ON);// set the Relay ON
    all_stop = 0;
  }else{
    digitalWrite(Relay_Green, RELAY_OFF);// set the Relay OFF
  }
}

void stop_signal (const std_msgs::Bool& stopped){

  std_msgs::Bool red_light = stopped;
  if (all_stop) {
    red_light.data = 1;
  }

  done = 0;

  // Tell Blue Light if it should be on
  if (red_light.data) {
    digitalWrite(Relay_Red, RELAY_ON);// set the Relay ON
  } else {
    digitalWrite(Relay_Red, RELAY_OFF);// set the Relay OFF
  }
}

// ROS Input
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> sub1("RemoteControlStatus", &teleop);
ros::Subscriber<std_msgs::Bool> sub2("ConnectionStatus", &connection);
ros::Subscriber<std_msgs::Bool> sub3("AutonomyStatus", &autonomous);
ros::Subscriber<std_msgs::Bool> sub4("StopStatus", &stop_signal);

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
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);

}

// Run Loop
void loop(){
  double random = 125;

  // If more time than *MAX_TIME* has passed since last message received, set all stop to true
  // to signal that robot is no longer connected and disable any other lights
  if(done >= MAX_TIME){
    all_stop = 1;
    digitalWrite(Relay_Orange, RELAY_OFF);
    digitalWrite(Relay_Green, RELAY_OFF);
    digitalWrite(Relay_Blue, RELAY_OFF);
  } else {
    done++;
  }

  if (all_stop) {
    digitalWrite(Relay_Red, RELAY_ON);// set the Relay ON
  } else {
    digitalWrite(Relay_Red, RELAY_OFF);// set the Relay OFF
  }
  nh.spinOnce();
  delay(1000);
}
