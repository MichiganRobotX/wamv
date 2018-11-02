// WAM-V Motor Controller
// version 3.4
// Last Updated: November 1, 2018
// Owner: Conner Goodrum
//------------------------------------------------------------------------------
// This motor controller only controls the port and starboard motors of the WAMV.
// It does not include any logic to control lateral thrusters.
//------------------------------------------------------------------------------

// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Int16.h>

// Initialize Arduino Pins
int pinL = 9; // PWM Output port for the left side
int pinR = 10; // PWM Ouptut port for the right side

// Setup Subscribers
void LeftMotor ( const std_msgs::Int16& leftSpeed){

  // Set the joystick values
  std_msgs::Int16 speedL = leftSpeed;
  // tell left motor how fast
  analogWrite(pinL, speedL.data);
}

void RightMotor ( const std_msgs::Int16& rightSpeed){

  // Set the joystick values
  std_msgs::Int16 speedR = rightSpeed;
  // tell right motor how fast
  analogWrite(pinR, speedR.data);
}


// Setup ROS
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub1("LmotorSpeed", &LeftMotor);
ros::Subscriber<std_msgs::Int16> sub2("RmotorSpeed", &RightMotor);

// Setup Sequence
void setup() {

  // ROS Input
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  // Change the Arduino Output Frequency
  byte mode = 0x01;
  TCCR2B = TCCR2B & 0b11111000 | mode;
}

// Run Loop
void loop() {

  nh.spinOnce();

}
