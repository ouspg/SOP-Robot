/*
 * Code for receiving and parsing serial data is made
 * by Robin2. The original code can be found from
 * https://forum.arduino.cc/index.php?topic=396450.0
 */

#include "XL320.h"
#include <SoftwareSerial.h>

XL320 robot;

//SoftwareSerial mySerial(7, 8);

char rgb[] = "rgbypcwo";

const byte numChars = 32;
char received[numChars];
char temp[numChars];

char message[numChars];
int val_1 = 0;
int val_2 = 0;
int val_3 = 0;
boolean newData = false;

void setup() {
  Serial.begin(57600);
  //mySerial.begin(115200);
  robot.begin(Serial);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
        strcpy(temp, received);
        parseData();
        showParsedData();
        newData = false;

    switch (val_1) {
      case 0:
        robot.LED(val_2, &rgb[val_3] ); // 0...7
        delay(100);
        break;
      case 1: 
        robot.moveJoint(val_2, val_3); // 0...1023
        delay(100);
        mySerial.println(robot.getJointPosition(val_2));
        delay(100);
        break;
      case 2:
        robot.setJointSpeed(val_2, val_3); // 0...1023
        break;
    default:
      break;
    }
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (mySerial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                received[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                received[ndx] = '?';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {
    char * strtokIndx;

    strtokIndx = strtok(temp,",");
    val_1 = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ",");
    val_2 = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    val_3 = atoi(strtokIndx);
}

void showParsedData() {
    Serial.print("val_1: ");
    Serial.println(val_1);
    Serial.print("val_2: ");
    Serial.println(val_2);
    Serial.print("val_3: ");
    Serial.println(val_3);
}
