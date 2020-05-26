#include "Arduino.h"   //tutorial sanoi että tämä kannattaa olla varalta
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

//servon liikerata 900-990
void servo_cb( const std_msgs::UInt16& cmd_msg)
{
 
  unsigned char hexl, hexh, CRC_L, CRC_H;

  if(int(cmd_msg.data) == 0)  //pos 900
  {
    hexl = 0x84;
    hexh = 0x03;
    CRC_L = 0x42;
    CRC_H = 0xC5;
  }
  else if(int(cmd_msg.data) == 1)  //pos 990
  {
    hexl = 0xDE;
    hexh = 0x03;
    CRC_L = 0x59;
    CRC_H = 0x8D;
  }
  else  //default suu kiinni
  {
    hexl = 0x84;
    hexh = 0x03;
    CRC_L = 0x42;
    CRC_H = 0xC5;
  }
     
  //                                         headeria          length      write osoite      arvo 
  unsigned char TxPacket[16] = { 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x03, 0x1E, 0x00, hexl, hexh, 0x00, 0x00, CRC_L, CRC_H};

  Serial.write(TxPacket,16);
  //delay(1);

}

ros::Subscriber<std_msgs::UInt16> sub("servo", &servo_cb);

void setup()
{ 
  Serial.begin(1000000);

  nh.getHardware()->setBaud(1000000);  //set nh to same baudrate as serial
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);    
}


