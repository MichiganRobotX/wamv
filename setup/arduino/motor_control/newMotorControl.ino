// WAM-V Motor Controller
// version 3.1
// Last Updated: August 3, 2018
// Owner: James Coller

// Libraries
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <stdio.h>

// Initialize Arduino Pins
int pinL = 9; // PWM Output port for the left side
int pinR = 10; // PWM Ouptut port for the right side
//int potpin = 1; // Analog In Location from the potentiometer
//std_msgs::Int16 Lspeed;

// Set Publisher
ros::Publisher chatter("chatter", &Lspeed);

// Setup Subscribers
void leftmotor ( const std_msgs::Int16& leftSpeed){

  // Set the joystick values
  std_msgs::Int16 speedL = leftSpeed;

  analogWrite(pinL, speedL.data); // tell left motor how fast

  // say what you got:
  Lspeed = speedL;
  chatter.publish( &Lspeed );
}

void rightmotor ( const std_msgs::Int16& rightSpeed){

  // Set the joystick values
  std_msgs::Int16 speedR = rightSpeed;

  analogWrite(pinR, speedR.data); // tell left motor how fast
}


// Setup ROS
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub1("LmotorSpeed", &leftmotor);
ros::Subscriber<std_msgs::Int16> sub2("RmotorSpeed", &rightmotor);

// Change the Arduino Output Frequency 
byte mode = 0x01;
TCCR2B = TCCR2B & 0b11111000 | mode;

// Setup Sequence
void setup() {
  // ROS Input
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(chatter);
}

// Run Loop
void loop() {
  double random = 125;

  nh.spinOnce();

  delay(1000);
  //nh.spinOnce();
  //delay(1);
}
