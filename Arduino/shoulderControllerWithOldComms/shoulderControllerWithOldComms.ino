#include <Servo.h>


const int NUM_SERVOS = 9;
const int POT_PINS[NUM_SERVOS] = {A0, A1, A2, A3, A4, A5};
const int SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
const int ServoMins[NUM_SERVOS] = {0, 0, 20, 0, 0, 0, 0, 55, 0};
const int ServoMax[NUM_SERVOS] = {180, 180, 100, 60, 180, 180, 100, 115, 180};

const int expectedStartingPos[NUM_SERVOS] = {30, 90, 10, 0, 34, 80, 10, 0};

/*
servo pin - servo function
        2 - R shoulder lift
        3 - R upper arm rotation
        4 - R bicep
        5 - R shoulder out

        6 - L shoulder lift
        7 - L upper arm rotion
        8 - L bicep
        9 - L shoulder out

       10 - ? empty ?
*/

// can be replaced with constrain() ?
int check(int value, int min, int max){
  return (value + min < max) ? (min + value) : ((min + value > max ) ? max : value);
}

// convert potentiometer values to degrees
int potToDegree(int value) {
  return map(value, 0, 1023, 0, 180);
}

Servo servos[NUM_SERVOS];

void setup() {
  SerialUSB.begin(115200);

  // Attach servos to pins and check if potentiometers have moved

  for (int i = 0; i < NUM_SERVOS; ++i) {
    int currentPos = potToDegree(analogRead(POT_PINS[i]));

    if (abs(expectedStartingPos[i]) - currentPos) > 5 {
      SerialUSB.println("Potentiometer misaligned on servo: ");
      SerialUSB.print(SERVO_PINS[i]);
      SerialUSB.println();

      // stop execution to avoid dmg
      while(1);
    }
    servos[i].write(expectedStartingPos[i]);
    servos[i].attach(SERVO_PINS[i]);
  }

  // Only R upper rotation
  // int currentPos = potToDegree(analogRead(POT_PINS[1]));

  //   if (abs(expectedStartingPos[1]) - currentPos) > 5 {
  //     SerialUSB.println("Potentiometer misaligned on servo: ");
  //     SerialUSB.print(SERVO_PINS[1]);
  //     SerialUSB.println();

  //     // stop execution to avoid dmg
  //     while(1);
  //   }
  //   servos[1].attach(SERVO_PINS[1]);
}

void loop() {
  if (SerialUSB.available() > 0) {
    String command = SerialUSB.readStringUntil('\n');

    // Split the received command by commas
    int angles[NUM_SERVOS];
    int angleIndex = 0;
    int start = 0;
    int end = command.indexOf(',');

    while (end != -1) {
      angles[angleIndex++] = command.substring(start, end).toInt();
      start = end + 1;
      end = command.indexOf(',', start);
    }
    angles[angleIndex] = command.substring(start).toInt();

    // Set angles for each servo
    for (int i = 0; i < NUM_SERVOS; ++i) {
      if(angles[i] != -1){
        int angle = check(angles[i], ServoMins[i], ServoMax[i]);
        angles[i] = angle;
        servos[i].write(angle);
      }

      // Delay so each instruction has time to complete
      while (abs(potToDegree(analogRead(POT_PINS[servoIndex])) - angle) > 5) {
        delay(300);
      }
    }

    // Send back the received angles
    SerialUSB.print("Received angles: ");
    for (int i = 0; i < NUM_SERVOS; ++i) {
      SerialUSB.print(angles[i]);
      if (i < NUM_SERVOS - 1) {
        SerialUSB.print(",");
      }
    }
    SerialUSB.println();
  }
}