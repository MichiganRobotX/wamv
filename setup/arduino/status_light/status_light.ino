// Libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// Set the Arduino Parameters
#define RELAY_ON 0
#define RELAY_OFF 1
#define Relay_Red  5
#define Relay_Yellow  4
#define Relay_Green  3
#define Relay_Blue  2

// Setup Subscribers
int time_since_heartbeat = 0;
void heartbeat_callback (const std_msgs::String& msg) {
    time_since_heartbeat = 0;
}

bool activate_yellow = 0;
void yellow_light_callback (const std_msgs::Bool& yellow_light) {
    if (yellow_light.data) { activate_yellow = 1; }
    else { activate_yellow = 0; }
}

bool activate_blue = 0;
void blue_light_callback (const std_msgs::Bool& blue_light) {
    if (blue_light.data) { activate_blue = 1; }
    else { activate_blue = 0; }
}

bool activate_green = 0;
void green_light_callback (const std_msgs::Bool& green_light) {
    if (green_light.data) { activate_green = 1; }
    else { activate_green = 0; }
}

bool activate_red = 0;
void red_light_callback (const std_msgs::Bool& red_light) {
    if (red_light.data) { activate_red = 1; }
    else { activate_red = 0; }
}

// ROS Input
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> yellow_sub("yellow_light_active", &yellow_light_callback);
ros::Subscriber<std_msgs::Bool> blue_sub("blue_light_active", &blue_light_callback);
ros::Subscriber<std_msgs::Bool> green_sub("green_light_active", &green_light_callback);
ros::Subscriber<std_msgs::Bool> red_sub("red_light_active", &red_light_callback);
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
    nh.subscribe(yellow_sub);
    nh.subscribe(blue_sub);
    nh.subscribe(green_sub);
    nh.subscribe(red_sub);
    nh.subscribe(heartbeat_sub);
}

// Run Loop

int MAX_TIME = 3;
void loop() {
    nh.spinOnce();

    if (time_since_heartbeat < MAX_TIME) {
        ++time_since_heartbeat;

        if (activate_red == 1) { digitalWrite(Relay_Red, RELAY_ON); }
        else { digitalWrite(Relay_Red, RELAY_OFF);}

        if (activate_yellow == 1) { digitalWrite(Relay_Yellow, RELAY_ON); }
        else { digitalWrite(Relay_Yellow, RELAY_OFF); }

        if (activate_green == 1) { digitalWrite(Relay_Green, RELAY_ON); }
        else { digitalWrite(Relay_Green, RELAY_OFF); }

        if (activate_blue == 1) { digitalWrite(Relay_Blue, RELAY_ON); }
        else { digitalWrite(Relay_Blue, RELAY_OFF); }
    }
    else {
        digitalWrite(Relay_Red, RELAY_ON);

        digitalWrite(Relay_Yellow, RELAY_OFF);
        digitalWrite(Relay_Green, RELAY_OFF);
        digitalWrite(Relay_Blue, RELAY_OFF);
    }

    delay(1000);
}
