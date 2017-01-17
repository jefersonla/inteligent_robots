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

/* RemoteXY select connection mode and include library */
#define REMOTEXY_MODE__ESP8266WIFIPOINT_LIB
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

/* RemoteXY connection settings */
#define REMOTEXY_WIFI_SSID "RoboControl"
#define REMOTEXY_WIFI_PASSWORD "qwertyui"
#define REMOTEXY_SERVER_PORT 6377


/* RemoteXY configurate  */
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 4,0,56,0,6,0,0,5,0,14
  ,20,30,30,2,1,0,59,28,12,12
  ,6,88,0,1,0,73,28,12,12,1
  ,89,0,129,0,23,7,58,6,8,82
  ,111,98,111,116,105,99,97,32,73,110
  ,116,101,108,105,103,101,110,116,101,0
   };
  
/* this structure defines all the variables of your control interface */
struct {

    /* input variable */
  int8_t joystick_x; /* =-100..100 x-coordinate joystick position */
  int8_t joystick_y; /* =-100..100 y-coordinate joystick position */
  uint8_t button1; /* =1 if button pressed, else =0 */
  uint8_t button2; /* =1 if button pressed, else =0 */

    /* other variable */
  uint8_t connect_flag;  /* =1 if wire connected, else =0 */

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define PIN_BUTTON1 13
#define PIN_BUTTON2 12


void setup() 
{
  RemoteXY_Init (); 
  
  pinMode (PIN_BUTTON1, OUTPUT);
  pinMode (PIN_BUTTON2, OUTPUT);
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  
  digitalWrite(PIN_BUTTON1, (RemoteXY.button1==0)?LOW:HIGH);
  digitalWrite(PIN_BUTTON2, (RemoteXY.button2==0)?LOW:HIGH);
  
  // TODO you loop code
  // use the RemoteXY structure for data transfer


}
