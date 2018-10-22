#include "SoftwareSerial.h"

SoftwareSerial XBee(1,0);


//const int delay_time_before_send = 100;

const int stop_remote_pin = 2;
const int reset_remote_pin = 4;

int buttonState_one = 0;
int buttonState_two = 0;

void setup() {
  // put your setup code here, to run once:
  //pinMode(13,OUTPUT); //microcontroller LED is on d13

// pinMode(3,OUTPUT); //D3 as output for the contactor switch
 
 pinMode(stop_remote_pin, INPUT); //button pin set to an input
 pinMode(reset_remote_pin, INPUT); //button pin set to an input

  

 Serial.begin(9600);
//Serial.begin(19200);
 //Serial.println("Arduino started sending bytes via XBee");
 XBee.begin(9600);
 //xbee.begin(19200);
}


void loop() {
  // put your main code here, to run repeatedly:
int current_time = millis();
char on = 'H';
char off = 'O';

buttonState_one = digitalRead(stop_remote_pin); // read in button pin
buttonState_two = digitalRead(reset_remote_pin);
//Serial.println(buttonState_one);
  if ( buttonState_two == 1) //if green button is unpressed, send an 'O' to boat
  {
    //digitalWrite(13, HIGH);
    
    //XBee.print(on); //boat is running
   // while (millis() < current_time + delay_time_before_send)
    //{}
    Serial.println('O');
    //Serial.print("red button = ");
    //Serial.print(buttonState_one);
    //Serial.print('\n');


     //Serial.print("green button = ");
    //Serial.print(buttonState_two);
    //Serial.print('\n');
  }
  else //if (buttonState_one == 0)//if red button is pressed, turn off board led and send an 'O' to boat
  {
    //Serial.print("red button = ");
    //Serial.print(buttonState_one);
    //Serial.print('\n');

    //Serial.print("green button = ");
    //Serial.print(buttonState_two);
    //Serial.print('\n');

    while (buttonState_one == 1) //!= 0)//if red button is unpressed, boat stays on
    {
      buttonState_one = digitalRead(stop_remote_pin);//checks state of green button
      
      //Serial.print("green button = ");
      //Serial.print(buttonState_two);
      //Serial.print('\n');

      
      //digitalWrite(13,LOW);
      
     // XBee.print(off);
      //while (millis() < current_time + delay_time_before_send)
      //{}
      Serial.println('H');
    }
   
  }

}
