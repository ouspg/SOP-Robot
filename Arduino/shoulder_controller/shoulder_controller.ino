#include <Servo.h>

const int NUM_SERVOS = 9;

const int POT_PINS[6] = {A0, A1, A2, A3, A4, A5};
const int SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

const int ServoMins[NUM_SERVOS] = {10, 10, 20, 0, 0, 0, 0, 55, 0};
const int ServoMax[NUM_SERVOS] = {80, 180, 100, 60, 180, 180, 100, 115, 180};

// values based on manual measurements from each potentiometer
const int PotMins[NUM_SERVOS] = {144, 140, 0, 0, 0, 0, 0, 0, 0};
const int PotMax[NUM_SERVOS] = {360, 900, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

// Set expected for 0 on empty servos, to not trigger potentiometer check
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

// Converts potentiometer values to degrees
int potToDegree(int value, int servoIndex) {
  int servoMin = ServoMins[servoIndex];
  int servoMax = ServoMax[servoIndex];

  int potMin = PotMins[servoIndex];
  int potMax = PotMax[servoIndex];

  return map(value, potMin, potMax, servoMin, servoMax);
}

Servo servos[NUM_SERVOS];

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB) {
    delay(500);
  }

  // Has to be NUM_SERVOS long, to not break loops
  int currentPosL[NUM_SERVOS] = {potToDegree(analogRead(A0), 0), potToDegree(analogRead(A1), 1), 0, 0, 0, 0, 0, 0, 0};

  for (int i = 0; i < NUM_SERVOS; ++i) {
    
    // Uncomment if you want to check if all servos are in default position on startup
    /*
    //check if potentiometers have moved
    if (abs(expectedStartingPos[i] - currentPosL[i]) > 5) {
      SerialUSB.println("Potentiometer misaligned on servo: ");sdd
      SerialUSB.print(SERVO_PINS[i]);
      SerialUSB.println();

      // stop execution to avoid dmg
      while(1);
    }
    */
    
    // Write the starting positions, so servos don't move when attached
    servos[i].write(expectedStartingPos[i]);
    servos[i].attach(SERVO_PINS[i]);
  }
}

void loop() {
  // Moves all joints to default position when serial connection is lost
  if (!SerialUSB.dtr()) {
    for (int i = 0; i < NUM_SERVOS; ++i) {
      servos[i].write(expectedStartingPos[i]);
    }
    SerialUSB.end();

    // restart serial, and waits for connection
    SerialUSB.begin(115200);
    while (!SerialUSB) {
      delay(500);
    }
  }

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
      int constrainedAngle = constrain(angles[i], ServoMins[servoIndex], ServoMax[servoIndex]);
      servos[servoIndex].write(constrainedAngle);
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