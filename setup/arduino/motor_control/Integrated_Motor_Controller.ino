// WAM-V Motor Controller
// version 3.3
// Last Updated: October 15, 2018
// Owner: Conner Goodrum

// Libraries
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <stdio.h>
#include <Servo.h>
#include <Arduino.h>

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


std_msgs::Int16 Lspeed;

// Set Publisher
ros::Publisher chatter("chatter", &Lspeed);

// Setup Subscribers
void leftmotor ( const std_msgs::Int16& leftSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedL = leftSpeed;

  analogWrite(pinL, speedL.data); // tell left motor how fast

  // say what you got:
  Lspeed = speedL;
  chatter.publish( &Lspeed );
}

void rightmotor ( const std_msgs::Int16& rightSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedR = rightSpeed;

  analogWrite(pinR, speedR.data); // tell left motor how fast
}

void RightLatThrusters(const std_msgs::Int16& rightLatSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedR_lat = rightLatSpeed;

  servoLatR1.writeMicroseconds(speedR_lat.data);
  servoLatR2.writeMicroseconds(speedR_lat.data);

}

void LeftLatThrusters(const std_msgs::Int16& leftLatSpeed) {

  // Set the joystick values
  std_msgs::Int16 speedL_lat = leftLatSpeed;

  servoLatL1.writeMicroseconds(speedL_lat.data);
  servoLatL2.writeMicroseconds(speedL_lat.data);

}

void InitThrusters(Servo servo, byte servoPin) {

  servo.attach(servoPin);
  servo.writeMicroseconds(1500);
  delay(1000);
}

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
ros::Subscriber<std_msgs::Int16> sub1("LmotorSpeed", &leftmotor);
ros::Subscriber<std_msgs::Int16> sub2("RmotorSpeed", &rightmotor);
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
  nh.advertise(chatter);


  // Change the Arduino Output Frequency for pins 9 & 10
  // This could be where the issue is in communicating with the thrusters
  byte mode;
  mode = 0x01;
  TCCR2B = TCCR2B & 0b11111000 | mode;

}

// Run Loop
void loop() {

  nh.spinOnce();
  delay(1);

}
