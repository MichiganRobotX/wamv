//------------------------------------------------------------------------------
// Import libraries
#include <Arduino.h>
#include <ros.h>
#include <Servo.h>
#include <stdio.h>
#include <std_msgs/Int16.h>

//------------------------------------------------------------------------------
// Define Arduino pins

// PWM ouptut port for port-side and starboard-side main motors
int port_motor_pin = 9;
int strbrd_motor_pin = 10;

// PWM ouptut port for port-side and starboard-side lateral thrusters
byte port_thruster_pin_1 = 6;
byte port_thruster_pin_2 = 5;
byte strbrd_thruster_pin_1 = 8;
byte strbrd_thruster_pin_2 = 7;

//------------------------------------------------------------------------------
// Define servos for thrusters
Servo strbrd_thruster_servo_1;
Servo strbrd_thruster_servo_2;
Servo port_thruster_servo_1;
Servo port_thruster_servo_2;

//------------------------------------------------------------------------------
// Setup callbacks, node handle, and subscribers
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

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int16> port_motor_sub("port_motor_input", &port_motor_callback);
ros::Subscriber<std_msgs::Int16> strbrd_motor_sub("strbrd_motor_input", &strbrd_motor_callback);
ros::Subscriber<std_msgs::Int16> port_thruster_sub("port_thruster_input", &port_bow_thruster_callback);
ros::Subscriber<std_msgs::Int16> strbrd_bow_thruster_sub("strbrd_thruster_input", &strbrd_bow_thruster_callback);

//------------------------------------------------------------------------------
// Arduino setup
void setup() {
  Serial.begin(9600);
  delay(5000);

  // Initialize thrusters
  initialize_thruster(strbrd_thruster_servo_1, strbrd_thruster_pin_1);
  initialize_thruster(strbrd_thruster_servo_2, strbrd_thruster_pin_2);
  initialize_thruster(port_thruster_servo_1, port_thruster_pin_1);
  initialize_thruster(port_thruster_servo_2, port_thruster_pin_2);

  // Initialize ROS node and subscribers
  nh.initNode();
  nh.subscribe(port_motor_sub);
  nh.subscribe(strbrd_motor_sub);
  nh.subscribe(port_thruster_sub);
  nh.subscribe(strbrd_bow_thruster_sub);

  // Change the Arduino output frequency for pins 9 and 10. This could be where
  // the issue is in communicating with the thrusters.
  byte mode;
  mode = 0x01;
  TCCR2B = TCCR2B & 0b11111000 | mode;
}

//------------------------------------------------------------------------------
// Arduino loop
void loop() { nh.spinOnce(); }
