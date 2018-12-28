//------------------------------------------------------------------------------
// Import libraries
#include <ros.h>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

//------------------------------------------------------------------------------
// Define Arduino parameters
#define turn_on 0
#define turn_off 1
#define relay_red  5
#define relay_yellow  4
#define relay_green  3
#define relay_blue  2

//------------------------------------------------------------------------------
// Setup callbacks and constants
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

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Bool> yellow_sub("yellow_light_active", &yellow_light_callback);
ros::Subscriber<std_msgs::Bool> blue_sub("blue_light_active", &blue_light_callback);
ros::Subscriber<std_msgs::Bool> green_sub("green_light_active", &green_light_callback);
ros::Subscriber<std_msgs::Bool> red_sub("red_light_active", &red_light_callback);
ros::Subscriber<std_msgs::String> heartbeat_sub("heartbeat_string", &heartbeat_callback);

//------------------------------------------------------------------------------
// Arduino setup
void setup() {
    digitalWrite(relay_red, turn_off);
    digitalWrite(relay_yellow, turn_off);
    digitalWrite(relay_green, turn_off);
    digitalWrite(relay_blue, turn_off);

    pinMode(relay_red, OUTPUT);
    pinMode(relay_yellow, OUTPUT);
    pinMode(relay_green, OUTPUT);
    pinMode(relay_blue, OUTPUT);

    delay(5000);

    nh.initNode();
    nh.subscribe(yellow_sub);
    nh.subscribe(blue_sub);
    nh.subscribe(green_sub);
    nh.subscribe(red_sub);
    nh.subscribe(heartbeat_sub);
}

//------------------------------------------------------------------------------
// Arduino loop
int MAX_TIME = 3;
void loop() {
    nh.spinOnce();

    if (time_since_heartbeat < MAX_TIME) {
        ++time_since_heartbeat;

        if (activate_red == 1) { digitalWrite(relay_red, turn_on); }
        else { digitalWrite(relay_red, turn_off); }

        if (activate_yellow == 1) { digitalWrite(relay_yellow, turn_on); }
        else { digitalWrite(relay_yellow, turn_off); }

        if (activate_green == 1) { digitalWrite(relay_green, turn_on); }
        else { digitalWrite(relay_green, turn_off); }

        if (activate_blue == 1) { digitalWrite(relay_blue, turn_on); }
        else { digitalWrite(relay_blue, turn_off); }
    }

    else {
        digitalWrite(relay_red, turn_on);

        digitalWrite(relay_yellow, turn_off);
        digitalWrite(relay_green, turn_off);
        digitalWrite(relay_blue, turn_off);
    }

    delay(1000);
}
