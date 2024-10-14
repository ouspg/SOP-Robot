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

       10 - ?
*/

// can be replaced with constrain() ?
// int check(int value, int min, int max){
//   return (value + min < max) ? (min + value) : ((min + value > max ) ? max : value);
// }

// convert potentiometer values to degrees
int potToDegree(int value) {
  return map(value, 0, 1023, 0, 180);
}

Servo servos[NUM_SERVOS];

void setup() {
  SerialUSB.begin(115200);

  // Attach servos to pins and check if potentiometers have moved

  for (int i = 0; i < NUM_SERVOS; ++i) {
    int currentPos = potToDegree(analogRead(i));

    if (abs(expectedStartingPos[i]) - currentPos) > 5 {
      SerialUSB.println("Potentiometer misaligned on servo: ");
      SerialUSB.print(SERVO_PINS[i]);
      SerialUSB.println();

      // stop execution to avoid dmg
      while(1);
    }
    servos[i].attach(SERVO_PINS[i]);
  }

  // Only R upper rotation
  // int currentPos = potToDegree(analogRead(i));

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
  // The new version, uses id:angle commands
  if (SerialUSB.available() > 0) {
    String command = SerialUSB.readStringUntil('\n')

    // Split command string by commas and colons
    int servosToMove[NUM_SERVOS];
    int angles[NUM_SERVOS];
    int angleIndex = 0;

    int start = 0;
    int split = command.indexOf(':') + 1;
    int end = command.indexOf(',');

    while (end != -1) {
      // Assign ids and angles to arrays
      servosToMove[angleIndex] = command.substring(start, split).toInt();
      angles[angleIndex] = command.substring(split, end).toInt();
      angleIndex++;

      start = end + 1;
      split = command.indexOf(':', start) + 1;
      end = command.indexOf(',', start);
    }
    
    // get the last pair
    servosToMove[angleIndex] = command.substring(start, split).toInt();
    angles[angleIndex] = command.substring(split).toInt();
    angleIndex++;

    // set angles on specified servos
    for (int i = 0; i < angleIndex; ++i) {
      int servoIndex = servosToMove[i] - 2;
      // constrain() or check()
      int constrainedAngle = constrain(angles[i], ServoMins[servoIndex], ServoMax[servoIndex]);
      servos[servoIndex].write(constrainedAngle);

      // Delay so each instruction has time to complete
      while (abs(potToDegree(analogRead(POT_PINS[servoIndex])) - constrainedAngle) > 5) {
        delay(300);
      }
    }

    // Send back the received angles
    SerialUSB.print("Received angles: ");
    for (int i = 0; i < angleIndex; ++i) {
      SerialUSB.print(angles[i]);
      if (i < angleIndex - 1) {
        SerialUSB.print(",");
      }
    }
    SerialUSB.println();
  }
}