#include "SoftwareSerial.h"

SoftwareSerial XBee(0,1);

unsigned long previous_time = 0;//used for timer reset once communication is established
const int dead_time = 2500;//millisecond grace period of no communication until shutoff


void setup() {
  // put your setup code here, to run once:
 pinMode(13,OUTPUT); //microcontroller LED is on d13
 pinMode(3,OUTPUT); //D3 as output for the contactor switch
 Serial.begin(9600);
 //Serial.println("Arduino started receiving bytes via XBee");

  XBee.begin(9600);
}


void loop() 
{
 unsigned long current_time = millis();
 if (XBee.available())
  {
    previous_time = current_time;
    
    char b_state = XBee.read();
    //Serial.print("Data has arrived on the arduino");
    Serial.println(b_state);
    //Serial.println();
   if (b_state == 'H' )
   {
    digitalWrite(3,LOW);//*note, relay is on when this is low and vice versa 
    digitalWrite(13,LOW);//board led turns off
   }
   else if (b_state == 'O' )//this is an else if, since interference or a bad read can cause gibberish ascii values
   {
    digitalWrite(3,HIGH);//kills relay 
    digitalWrite(13,HIGH);//led on board goes on
   }
  }

  if ((unsigned long)(current_time - previous_time) >= dead_time) //if boat receives no data from remote for 3 seconds, kill power
  {
    digitalWrite(3,HIGH);//kills relay
    digitalWrite(13,HIGH);//board led turns on
  }
}



