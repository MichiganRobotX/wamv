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
int pinL = 9; // PWM Output port for the left side
int pinR = 10; // PWM Ouptut port for the right side
byte pinLatR1 = 8; // PWM Output port for Lateral thruster 1 (Right)
byte pinLatR2 = 7; // PWM Output port for Lateral thruster 2 (Right)
byte pinLatL1 = 6; // // PWM Output port for Lateral thruster 3 (Left)
byte pinLatL2 = 5; // PWM Output port for Lateral thruster 4 (Left)

//Define Servos for thrusters
Servo servoLatR1;
Servo servoLatR2;
Servo servoLatL1;
Servo servoLatL2;

// Setup Subscribers
void LeftMotor ( const std_msgs::Int16& leftSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedL = leftSpeed;

  // tell left motor how fast
  analogWrite(pinL, speedL.data);

}

void RightMotor ( const std_msgs::Int16& rightSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedR = rightSpeed;

  // tell right motor how fast
  analogWrite(pinR, speedR.data);

}

void LeftLatThrusters(const std_msgs::Int16& leftLatSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedL_lat = leftLatSpeed;

  // Tell left thrusters how fast
  servoLatL1.writeMicroseconds(speedL_lat.data);
  servoLatL2.writeMicroseconds(speedL_lat.data);

}

void RightLatThrusters(const std_msgs::Int16& rightLatSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedR_lat = rightLatSpeed;

  // Tell right hrusters how fast
  servoLatR1.writeMicroseconds(speedR_lat.data);
  servoLatR2.writeMicroseconds(speedR_lat.data);

}

void InitThrusters(Servo servo, byte servoPin) {

  servo.attach(servoPin);
  servo.writeMicroseconds(1500);

}

// Currently not used, but is included for troubleshooting purposes
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


// Setup ROS
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> sub1("LmotorSpeed", &LeftMotor);
ros::Subscriber<std_msgs::Int16> sub2("RmotorSpeed", &RightMotor);
ros::Subscriber<std_msgs::Int16> sub3("RmotorSpeed_lateral", &RightLatThrusters);
ros::Subscriber<std_msgs::Int16> sub4("LmotorSpeed_lateral", &LeftLatThrusters);


// Setup Sequence
void setup() {

  Serial.begin(9600);
  delay(1000);

  // Initialize Thrusters
  InitThrusters(servoLatR1, pinLatR1);
  InitThrusters(servoLatR2, pinLatR2);
  InitThrusters(servoLatL1, pinLatL1);
  InitThrusters(servoLatL2, pinLatL2);

  // ROS Input
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);


  // Change the Arduino Output Frequency for pins 9 & 10
  // This could be where the issue is in communicating with the thrusters
  byte mode;
  mode = 0x01;
  TCCR2B = TCCR2B & 0b11111000 | mode;

}

// Run Loop
void loop() {

  nh.spinOnce();

}
