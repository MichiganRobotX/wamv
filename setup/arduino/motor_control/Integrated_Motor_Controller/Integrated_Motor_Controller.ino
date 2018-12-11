// WAM-V Motor Controller
// version 3.4
// Last Updated: Nov 1, 2018
// Owner: Conner Goodrum
//------------------------------------------------------------------------------
// This motor controller controls both the WAMV motors and lateral thrusters.
//------------------------------------------------------------------------------

// Libraries
#include <Arduino.h>
#include <ros.h>
#include <Servo.h>
#include <stdio.h>
#include <std_msgs/Int16.h>

// Initialize Arduino Pins
int port_motor_pin = 9; // PWM Output port for the left side
int strbrd_motor_pin = 10; // PWM Ouptut port for the right side
byte strbrd_thruster_pin_1 = 8; // PWM Output port for Lateral thruster 1 (Right)
byte strbrd_thruster_pin_2 = 7; // PWM Output port for Lateral thruster 2 (Right)
byte port_thruster_pin_1 = 6; // // PWM Output port for Lateral thruster 3 (Left)
byte port_thruster_pin_2 = 5; // PWM Output port for Lateral thruster 4 (Left)

//Define Servos for thrusters
Servo strbrd_thruster_servo_1;
Servo strbrd_thruster_servo_2;
Servo port_thruster_servo_1;
Servo port_thruster_servo_2;

// Setup Subscribers
void port_motor_callback(const std_msgs::Int16& msg) {
    analogWrite(port_motor_pin, msg.data);
}

void strbrd_motor_callback (const std_msgs::Int16& msg) {
    analogWrite(strbrd_motor_pin, msg.data);
}

void port_bow_thruster_callback (const std_msgs::Int16& msg) {
    port_thruster_servo_1.writeMicroseconds(msg.data);
    port_thruster_servo_2.writeMicroseconds(msg.data);
}

void strbrd_bow_thruster_callback (const std_msgs::Int16& msg) {
    strbrd_thruster_servo_1.writeMicroseconds(msg.data);
    strbrd_thruster_servo_2.writeMicroseconds(msg.data);
}

void initialize_thruster(Servo servo, byte pin) {
    servo.attach(pin);
    servo.writeMicroseconds(1500);
}

// Setup ROS
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> port_motor_sub("port_motor_input", &port_motor_callback);
ros::Subscriber<std_msgs::Int16> strbrd_motor_sub("strbrd_motor_input", &strbrd_motor_callback);
ros::Subscriber<std_msgs::Int16> port_thruster_sub("port_thruster_input", &port_bow_thruster_callback);
ros::Subscriber<std_msgs::Int16> strbrd_bow_thruster_sub("strbrd_thruster_input", &strbrd_bow_thruster_callback);

// Setup Sequence
void setup() {
  Serial.begin(9600);

  delay(5000);

  // Initialize Thrusters
  initialize_thruster(strbrd_thruster_servo_1, strbrd_thruster_pin_1);
  initialize_thruster(strbrd_thruster_servo_2, strbrd_thruster_pin_2);
  initialize_thruster(port_thruster_servo_1, port_thruster_pin_1);
  initialize_thruster(port_thruster_servo_2, port_thruster_pin_2);

  // ROS Input
  nh.initNode();
  nh.subscribe(port_motor_sub);
  nh.subscribe(strbrd_motor_sub);
  nh.subscribe(port_thruster_sub);
  nh.subscribe(strbrd_bow_thruster_sub);

  // Change the Arduino Output Frequency for pins 9 & 10
  // This could be where the issue is in communicating with the thrusters
  byte mode;
  mode = 0x01;
  TCCR2B = TCCR2B & 0b11111000 | mode;
}

// Run Loop
void loop() { nh.spinOnce(); }
