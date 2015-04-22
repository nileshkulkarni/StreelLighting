/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <XBee.h>
#include <SoftwareSerial.h>
#include <Timer.h>
#define CONF_SYN 10
#define CONF_ACK 11

#define SWITCH_ON 1
#define SWITCH_ON_ACK 2

#define MAX_NBRS 2
#define READ_TIMEOUT 10
#define LED_ON_COST 10000
#define MIN_ON_TIME 2000
#define ACK_WAIT_TIME 2000
/*
This example is for Series 1 XBee (802.15.4)
Receives either a RX16 or RX64 packet and sets a PWM value based on packet data.
Error led is flashed if an unexpected packet is received
*/
uint8_t payload[] = {0};
//enum SIDE{ LEFT, RIGHT,UNDEFINED};

// 64-bit addressing: This is the SH + SL address of remote XBee
//XBeeAddress64 addr64 = XBeeAddress64(0x00, 0xFFFF);
XBeeAddress64 addr64;
// unless you have MY on the receiving radio set to FFFF, this will be received as a RX16 packet
//Tx64Request tx = Tx64Request(addr64, payload, sizeof(payload));
Tx64Request tx;

TxStatusResponse txStatus = TxStatusResponse();
bool isResendScheduled = false;
typedef struct {
  uint32_t DH;
  uint32_t DL;
  int strength;
} NodeDetails;

uint8_t* ackList = (uint8_t*)malloc(MAX_NBRS*sizeof(uint8_t));
NodeDetails* nbrList = (NodeDetails*)malloc(MAX_NBRS*sizeof(NodeDetails));
int noOfNbrs=0;
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

int statusLed = 13;
int errorLed = 12;
int dataLed = 9;
int sensorpin = 0;
int sensorVal = 0;                 // variable to store the values from sensor(initially zero)
unsigned long meanArrivalTime = 20000;
int passingCount = 0;
int passing = 0;
bool on = 0;
int offEventID = -1;
Timer t;
unsigned long lastTime = 0;


void autoConfiguration(){
 // Broadcast your packet with AUTO_SYN
 noOfNbrs = 0;
 Serial.println("Broadcasting");
 addr64 = XBeeAddress64(0x00, 0xFFFF);
 payload[0] = CONF_SYN;
 tx = Tx64Request(addr64, payload, sizeof(payload));
 xbee.send(tx);
 for (int i =0;i<MAX_NBRS;i++){
   ackList[i] =0;
 }
}

void signalOn(){
   Serial.println("Sending to nbrs that found a car");
   for(int i=0;i<noOfNbrs;i++)
   {
     addr64 = XBeeAddress64(nbrList[i].DH, nbrList[i].DL);
     payload[0] = SWITCH_ON;
     tx = Tx64Request(addr64, payload, sizeof(payload));
     ackList[i]=1;
     xbee.send(tx);
   }
   if(!isResendScheduled){
      t.after(ACK_WAIT_TIME, reSendSignal);
      isResendScheduled = true;
    }
}

void turnOff(){
  on = false;
  digitalWrite(statusLed,LOW);
  Serial.print("Current Time: ");
  Serial.println(millis());
  Serial.println("turning off");
  //delay(100000);
  for(int i =0;i<noOfNbrs;i++)
    ackList[i]=0;
}

void sensing()
{
//  Serial.println(millis());
  sensorVal = analogRead(sensorpin); 

  //Serial.println(sensorVal);
  if(passing == 0 && sensorVal > 200)
  {
    passing = 1;
    passingCount = 0; 
    Serial.print("Incoming");
    Serial.println(millis());
       
  }
  else if(passing == 1 && sensorVal >= 200)
  {

    if(passingCount >= 2)
    {
      passing = 2;
      meanArrivalTime = 0.5 * ( millis() - lastTime) + 0.5* meanArrivalTime;
      Serial.print("u mean ");
      Serial.println(meanArrivalTime);
      lastTime= millis();
      t.stop(offEventID);
      on = true;      
      digitalWrite(statusLed, HIGH);
      signalOn();

      if(meanArrivalTime > LED_ON_COST)  
      {
        Serial.print("offing after ");  
        Serial.println(MIN_ON_TIME);
        Serial.print("offing event scheduled at: ");
        Serial.println(MIN_ON_TIME+millis());
        offEventID = t.after((unsigned long)MIN_ON_TIME,turnOff);
      }

      else
      {
        offEventID = t.after(2*LED_ON_COST - meanArrivalTime,turnOff);       
        Serial.print("offing after ");
        Serial.print(2*LED_ON_COST - meanArrivalTime);
      }
//      offEventID = t.after(2000,turnOff);       
//      Serial.println(offEventID);
    }
    else
    {
      passingCount++;
//      Serial.println("ignored");
    }
  }
  else if (passing == 1)
  {
    Serial.print("ignored ");
    Serial.println(millis());
    passing = 0;
    
  }  
  else if(passing == 2 && sensorVal <200)
  {
    passing = 0;
  }
}



uint8_t option = 0;
uint8_t data = 0;
SoftwareSerial mySerial(10, 11);
int ledcount=0;
void flashLed(int pin, int times, int wait) {
    
    for (int i = 0; i < times; i++) {
      digitalWrite(pin, HIGH);
      delay(wait);
      digitalWrite(pin, LOW);
      
      if (i + 1 < times) {
        delay(wait);
      }
    }  
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(dataLed,  OUTPUT);
  
  // start serial
  mySerial.begin(9600);  
  Serial.begin(9600);
  xbee.setSerial(mySerial);
  //flashLed(statusLed, 3, 50);
  autoConfiguration();
  t.every(15,sensing);
  t.every(600000,autoConfiguration);
}

void addNbr(Rx64Response recv64){
  XBeeAddress64 NbrAddr64;
  NbrAddr64 = recv64.getRemoteAddress64();
  bool alreadyExist = false;
  for(int i =0;i<noOfNbrs;i++){
    Serial.print("matching ");
    Serial.print(nbrList[i].DL);
    Serial.print(" with ");
    Serial.println(NbrAddr64.getLsb());
    if(nbrList[i].DH == NbrAddr64.getMsb() && nbrList[i].DL == NbrAddr64.getLsb()){
        alreadyExist=true;
        Serial.println("nbr exists");
        break; 
    }
  }
  if(alreadyExist==false ){

   int maxStrength = nbrList[0].strength;
   int maxIndex = 0; 
   bool closer = true;
    if(noOfNbrs<MAX_NBRS) 
    { 
      maxIndex = noOfNbrs;
      noOfNbrs++;
    }

    else
    {   

      for(int i=0;i<noOfNbrs;i++)
      {
        if(nbrList[i].strength > maxStrength)
        {
          maxStrength = nbrList[i].strength;
          maxIndex = i;
        }
      }
      if(maxStrength <= rx64.getRssi())
      {
        closer = false;
      }
      else
      {
        Serial.print("Replacing id ");   
        Serial.println(nbrList[maxIndex].DL);
      }
    }
    if(closer){
      Serial.println("Adding nbr");      
      nbrList[maxIndex].DH = NbrAddr64.getMsb();
      nbrList[maxIndex].DL= NbrAddr64.getLsb();
      Serial.println(nbrList[maxIndex].DH);
      Serial.println(nbrList[maxIndex].DL);
      nbrList[maxIndex].strength = rx64.getRssi();
//      nbrList[maxIndex].side = UNDEFINED;      
    }
  }
  Serial.print("Current size ");
  Serial.println(noOfNbrs);
  for(int i=0;i<noOfNbrs;i++)
  {
    Serial.print("Member ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(nbrList[i].DL);
    Serial.print(" ");
    Serial.println(nbrList[i].strength);
  }
}

void reSendSignal(){
  bool scheduleResendEvent = false;
  for(int i =0;i<noOfNbrs;i++){
      if(ackList[i] ==1){
        Serial.print("Expecting ACK ");
        Serial.println(nbrList[i].DL);
        addr64 = XBeeAddress64(nbrList[i].DH, nbrList[i].DL);
        payload[0] = SWITCH_ON;
        tx = Tx64Request(addr64, payload, sizeof(payload));
        ackList[i]=1;
        xbee.send(tx);
        scheduleResendEvent =true;
      }
    }
    isResendScheduled=false;    
    if(scheduleResendEvent){
      t.after(ACK_WAIT_TIME, reSendSignal);
      isResendScheduled=true;    
    }

}
void verfiyAck(Rx64Response recv64){
  XBeeAddress64 NbrAddr64;
  NbrAddr64 = recv64.getRemoteAddress64();
  for(int i =0;i<noOfNbrs;i++){
    if(nbrList[i].DH == NbrAddr64.getMsb() && nbrList[i].DL == NbrAddr64.getLsb()){
       ackList[i] =0;
       Serial.print("Received ACK From ");
       Serial.println(NbrAddr64.getLsb());
       
    }
  }
}
void sendACKs(Rx64Response recv64){
  payload[0] = SWITCH_ON_ACK;
  tx = Tx64Request(recv64.getRemoteAddress64(), payload, sizeof(payload));
  xbee.send(tx);
  Serial.println("Sending ACKs");
}
void processPayload(Rx64Response recv64){
  Serial.print("Payload Received");
 uint8_t pL = rx64.getData(0);
  Serial.println(pL); 
 switch(pL){
  case CONF_SYN:
      addNbr(recv64);
      payload[0] = CONF_ACK;
      tx = Tx64Request(recv64.getRemoteAddress64(), payload, sizeof(payload));
      xbee.send(tx);     
    break;
  case CONF_ACK:
        addNbr(recv64);
    break;
  case SWITCH_ON:
      t.stop(offEventID);
      on = true;      
      digitalWrite(statusLed, HIGH);
      Serial.println("Led turned on by wireless message");
      delay(100);
      
      if(meanArrivalTime > LED_ON_COST)  
      {
        Serial.print("offing after ");
        Serial.print(MIN_ON_TIME);
        offEventID = t.after((unsigned long)MIN_ON_TIME,turnOff);
      }
      else 
      {
        offEventID = t.after(2*LED_ON_COST - meanArrivalTime,turnOff);
        Serial.print("offing after ");
        Serial.print(2*LED_ON_COST - meanArrivalTime);
      }
      sendACKs(recv64);
   break; 
   case SWITCH_ON_ACK:
        verfiyAck(recv64);
   break;
 } 
  
}

// continuously reads packets, looking for RX16 or RX64
void loop() {
  
  
   t.update();
    
    xbee.readPacket(READ_TIMEOUT);
    if (xbee.getResponse().isAvailable()) {
      // got something
      Serial.println("got something");
             // got a rx packet
        
        if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
                xbee.getResponse().getRx16Response(rx16);
        	option = rx16.getOption();
        	data = rx16.getData(0);
                Serial.print("RSSI 16");
                Serial.println(rx16.getRssi());
                processPayload(rx64);                  
        } else if (xbee.getResponse().getApiId() == RX_64_RESPONSE) {
                xbee.getResponse().getRx64Response(rx64);
        	option = rx64.getOption();
        	data = rx64.getData(0);
                Serial.print("RSSI 64");
                Serial.println(rx64.getRssi());
                processPayload(rx64);  
        }
        else{
         // This is our packet

        }
        // TODO check option, rssi bytes    
        //flashLed(statusLed, 1, 10);        
        // set dataLed PWM to value of the first byte in the data
        //analogWrite(dataLed, data);
        
    }
    else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");  
      //nss.println(xbee.getResponse().getErrorCode());
      // or flash error led
    } 
}
