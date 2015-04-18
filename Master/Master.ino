#include <SoftwareSerial.h>
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

SoftwareSerial mySerial(10, 11); // RX,TX
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
}
   unsigned long startTime;
   unsigned long RoundTripTime;
// the loop function runs over and over again forever
void loop() {
  while(1){

   //Serial.println("start");    
   startTime = micros();
   mySerial.write('a');
   //Serial.println("beech me");  
   while(mySerial.available()<=0); 
   char incomingByte = mySerial.read();
   
   RoundTripTime = micros() - startTime;
   //Serial.println("end");    
   Serial.println(RoundTripTime);
   //Serial.println(incomingByte);
   if(incomingByte=='a'){

    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
   }
   else{
       digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
   }
   delay(1000);  
   startTime = micros();
   mySerial.write('b');
   while(mySerial.available()<=0); 
   incomingByte = mySerial.read();
   RoundTripTime = micros() - startTime;
   Serial.println(RoundTripTime);
   //Serial.println(incomingByte);
   if(incomingByte=='a'){

    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
   }
   else{
       digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
   }
//  delay(1000);              // wait for a second
   delay(1000);  
  }
}
