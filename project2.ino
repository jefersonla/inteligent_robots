/* 
   -- New project -- 
    
   This source code of graphical user interface  
   has been generated automatically by RemoteXY editor. 
   To compile this code using RemoteXY library 2.2.5 or later version  
   download by link http://remotexy.com/en/library/ 
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                    
     - for ANDROID 3.7.1 or later version; 
     - for iOS 1.0.7 or later version; 
     
   This source code is free software; you can redistribute it and/or 
   modify it under the terms of the GNU Lesser General Public 
   License as published by the Free Software Foundation; either 
   version 2.1 of the License, or (at your option) any later version.     
*/ 

////////////////////////////////////////////// 
//        RemoteXY include library          // 
////////////////////////////////////////////// 

// RemoteXY select connection mode and include library  
#define REMOTEXY_MODE__ESP8266WIFIPOINT_LIB
#include <ESP8266WiFi.h> 

#include <RemoteXY.h> 

// RemoteXY connection settings  
#define REMOTEXY_WIFI_SSID "RobotControl" 
#define REMOTEXY_WIFI_PASSWORD "qwertyui" 
#define REMOTEXY_SERVER_PORT 6377 


// RemoteXY configurate   
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 6,0,90,0,6,0,0,1,0,24
  ,14,12,12,2,47,92,0,1,0,12
  ,25,12,12,2,60,32,0,1,0,36
  ,25,12,12,2,62,0,1,0,24,37
  ,12,12,2,92,47,0,1,0,61,26
  ,12,12,1,88,0,1,0,76,26,12
  ,12,6,89,0,129,0,19,2,63,6
  ,11,67,111,110,116,114,111,108,101,32
  ,87,105,32,45,32,70,105,32,82,111
  ,98,111,116,0 }; 
   
// this structure defines all the variables of your control interface  
struct { 

    // input variable
  uint8_t button_1; // =1 if button pressed, else =0 
  uint8_t button_2; // =1 if button pressed, else =0 
  uint8_t button_3; // =1 if button pressed, else =0 
  uint8_t button_4; // =1 if button pressed, else =0 
  uint8_t button_5; // =1 if button pressed, else =0 
  uint8_t button_6; // =1 if button pressed, else =0 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop) 

///////////////////////////////////////////// 
//           END RemoteXY include          // 
///////////////////////////////////////////// 



void setup()  
{ 
  RemoteXY_Init ();  
   
   
  // TODO you setup code 
   
} 

void loop()  
{  
  RemoteXY_Handler (); 
   
   
  // TODO you loop code 
  // use the RemoteXY structure for data transfer 


}
