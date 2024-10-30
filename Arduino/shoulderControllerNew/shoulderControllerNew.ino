#include <Servo.h>

const int NUM_SERVOS = 9;

const int POT_PINS[2] = {A0, A1};
const int SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

const int ServoMins[NUM_SERVOS] = {10, 10, 20, 0, 0, 0, 0, 55, 0};
const int ServoMax[NUM_SERVOS] = {60, 180, 100, 60, 180, 180, 100, 115, 180};


const int PotMins[NUM_SERVOS] = {0, 140, 0, 0, 0, 0, 0, 0, 0}
const int PotMins[NUM_SERVOS] = {1023, 900, 1023, 1023, 1023, 1023, 1023, 1023, 1023}

const int expectedStartingPos[NUM_SERVOS] = {30, 90, 0, 0, 0, 0, 0, 0};

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

// Convert potentiometer values to degrees
int potToDegree(int value, int servoIndex) {
  int min = ServoMins[servoIndex];
  int max = ServoMax[servoIndex];
  int degree = map(value, 140, 900, min, max);
  return degree;
}

Servo servos[NUM_SERVOS];

void setup() {
  SerialUSB.begin(115200);
  // Attach servos to pins and check if potentiometers have moved

  // stupid hard coding, because all servos can't have a potentiometer
  int currentPosL[NUM_SERVOS] = {potToDegree(analogRead(A0), 0)), potToDegree(analogRead(A1), 1), 0, 0, 0, 0, 0, 0, 0};

  for (int i = 0; i < NUM_SERVOS; ++i) {

    int currentPos = currentPosL[i];
    
    if (abs(expectedStartingPos[i]) - currentPos) > 5 {
      SerialUSB.println("Potentiometer misaligned on servo: ");
      SerialUSB.print(SERVO_PINS[i]);
      SerialUSB.println();

      // stop execution to avoid dmg
      while(1);

    }
  
    // Write the starting positions, so servos don't move when attached
    servos[i].write(expectedStartingPos[i]);
    servos[i].attach(SERVO_PINS[i]);
  }

  /*
  // Only R upper rotation
  int currentPos = potToDegree(analogRead(A1), 1);
    if (abs(expectedStartingPos[1] - currentPos) > 5) {
      SerialUSB.println("Potentiometer misaligned on servo: ");
      SerialUSB.print(SERVO_PINS[1]);
      SerialUSB.println();

      // stop execution to avoid dmg
      while(1);
    }

    servos[1].write(expectedStartingPos[1]);
    servos[1].attach(SERVO_PINS[1]);

    SerialUSB.println("Connected");
    */
}

void loop() {
  // The new version, uses id:angle commands

  if (SerialUSB.available() > 0) {
    String command = SerialUSB.readStringUntil('\n');

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

      delay(1000);
      /*
      // Delay so each instruction has time to complete
      while (abs(potToDegree(analogRead(POT_PINS[servoIndex]), servoIndex) - constrainedAngle) > 5) {
        delay(300);
      }
      */
    }

    SerialUSB.println(analogRead(A0));
    SerialUSB.println(potToDegree(analogRead(A0), 0));

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