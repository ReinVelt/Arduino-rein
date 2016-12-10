




/****************************************************************************************
* File:     Loraduinotracker.ino
* Author:   Rein Velt
****************************************************************************************/
/****************************************************************************************
* Created on:         20-11-2015
* Supported Hardware: ID150119-02 Nexus board with RFM95
* 
* Description
* 
* Minimal Uplink for LoRaWAN
* 
* This code demonstrates a LoRaWAN connection on a Nexus board. This code sends a messege every minute
* on channel 0 (868.1 MHz) Spreading factor 7.
* On every message the frame counter is raised
* 
* This code does not include
* Receiving packets and handeling
* Channel switching
* MAC control messages
* Over the Air joining* 
*
* Firmware version: 1.0
* First version
* 
* Firmware version 2.0
* Working with own AES routine
* 
* Firmware version 3.0
* Listening to receive slot 2 SF9 125 KHz Bw
* Created seperate file for LoRaWAN functions
****************************************************************************************/

/*
*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************
*/
#include <SPI.h>
#include "AES-128_V10.h"
#include "Encrypt_V30.h"
#include "LoRaWAN_V30.h"
#include "RFM95_V20.h"
#include "LoRaMAC_V10.h"
#include "Waitloop_V10.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>


#define sendIntervalMs 10000
/*
*****************************************************************************************
* GLOBAL VARIABLES
*****************************************************************************************
*/
// { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
unsigned char NwkSkey[16] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

unsigned char AppSkey[16] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

unsigned char DevAddr[4] = {
  0xD0, 0x99, 0x13, 0xDD
};

SoftwareSerial gpsDevice(GPSRX,A0);
TinyGPSPlus gpsParser;
unsigned char message[255];




void setup() 
{
   //Initialize the UART
  Serial.begin(115200);
  Serial.println("LORADUINOTRACKER 1.0 by Theo's Mechanic Ape");
  Serial.println("http://mechanicape.com");
  Serial.println("setup: Init SPI port");
   //Initialise the SPI port
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  
  Serial.println("setup: Init GPS device");
  gpsDevice.begin(4800);
  
  //Initialize I/O pins
  pinMode(DS2401,OUTPUT);
  pinMode(MFP,INPUT);
  pinMode(DIO0,INPUT);
  pinMode(DIO1,INPUT); 
  pinMode(DIO5,INPUT);
  pinMode(DIO2,INPUT);
  pinMode(CS,OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(GPSRX,INPUT);
  

  digitalWrite(DS2401,HIGH);
  digitalWrite(CS,HIGH);

  WaitLoop_Init();
  
  //Wait until RFM module is started
  WaitLoop(20); 
  Serial.println("setup: RFM module started");
  digitalWrite(LED,HIGH);  
}


void GpsConvertPositionIntoBinary(double Latitude, double Longitude)
{
  const int32_t MaxPosition = 8388607;       // 2^23 - 1 (= maxint for 24 bits)
  

    long double temp;
    uint32_t LatitudeBinary=0;
    uint32_t LongitudeBinary=0;
    Serial.print("\tlat:\t");
    Serial.print(Latitude);
    Serial.print("\tlon:\t");
    Serial.print(Longitude);
 
    
    temp = (90 + Latitude) * MaxPosition;
    LatitudeBinary = temp / 180;
      
    temp = (180+Longitude) * MaxPosition;
    LongitudeBinary = temp / 360;
    

  
  message[0]=0x00; //eerste byte..led off=0 on=1
  message[1]=0;
  message[2]=0;
  message[3]=0;
  message[4]=0;
  message[5]=0;
  message[6]=0;
  
  memcpy(&message[1], &LatitudeBinary, 3); //lat
  memcpy(&message[4], &LongitudeBinary, 3); //lng
 
 
}

void gpsFeed(unsigned long delayms)
{
  unsigned long timeStart;
  unsigned long timeCurrent;
  timeStart=millis();
  timeCurrent=0;
  while (timeStart+delayms>timeCurrent )
  {
    timeCurrent=millis();
    while (gpsDevice.available()>0)
    {
      gpsParser.encode(gpsDevice.read());
    } 
     
  }
  

  
}



void loop() 
{
  unsigned char Test = 0x00;
  unsigned char Sleep_Sec = 0x00;
  unsigned char Sleep_Time = 0x01;
  unsigned char Data_Tx[256];
  unsigned char Data_Rx[64];
  unsigned char Data_Length_Tx=7;
  unsigned char Data_Length_Rx = 0x00;

  //Initialize RFM module
  RFM_Init();

  while(1)
  {
     Serial.print("#");
    gpsFeed(sendIntervalMs);
    GpsConvertPositionIntoBinary(gpsParser.location.lat(),gpsParser.location.lng());
   
    //Construct data
    Data_Length_Rx = LORA_Cycle(message, Data_Rx, Data_Length_Tx);
    Serial.print("\tmsg:\t");
    for (int i=0;i<8;i++)
    {
      Serial.print(message[i]);
      Serial.print("\t");
    }
	  Serial.println();
	
    if(Data_Length_Rx != 0x00)
    {
      Test = Data_Rx[0];
	    Serial.println("loop: Data received");
    }
    

  }//While(1)
}
