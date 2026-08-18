#include <Servo.h>

const int NUM_SERVOS = 9;
const int MIN_SERVO_OFFSET = -90;
const int MAX_SERVO_OFFSET = 90;

// Commands address servos by their index in this array, independently of pin numbers.
const int SERVO_PINS[NUM_SERVOS] = {11, 3, 4, 5, 6, 7, 8, 9, 10};

// Values at the same index always describe the same servo.
const int SERVO_MINS[NUM_SERVOS] = {10, 10,  20,  0,  0,   0,   0,   55,  0};
const int SERVO_MAXS[NUM_SERVOS] = {80, 180, 100, 60, 180, 180, 100, 115, 180};

// Manually update these raw servo angles after finding the mechanical zeros.
const int SERVO_ZEROS[NUM_SERVOS] = {30, 90, 20, 0, 34, 80, 10, 55, 0};

/*
servo index - pin - servo function
          0 -  11 - R shoulder lift
          1 -   3 - R upper arm rotation
          2 -   4 - R bicep
          3 -   5 - R shoulder out

          4 -   6 - L shoulder lift
          5 -   7 - L upper arm rotation
          6 -   8 - L bicep
          7 -   9 - L shoulder out

          8 -  10 - unused
*/

Servo servos[NUM_SERVOS];
bool wasConnected = false;
bool sessionMoved = false;

void writeServoOffset(int servoIndex, int offset) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
    return;
  }

  int safeOffset = constrain(offset, MIN_SERVO_OFFSET, MAX_SERVO_OFFSET);
  int target = constrain(
    SERVO_ZEROS[servoIndex] + safeOffset,
    SERVO_MINS[servoIndex],
    SERVO_MAXS[servoIndex]
  );

  servos[servoIndex].write(target);
  if (!servos[servoIndex].attached()) {
    servos[servoIndex].attach(SERVO_PINS[servoIndex]);
  }
}

void moveAllServosToZero() {
  for (int i = 0; i < NUM_SERVOS; ++i) {
    servos[i].write(SERVO_ZEROS[i]);
    if (!servos[i].attached()) {
      servos[i].attach(SERVO_PINS[i]);
    }
  }
}

void setup() {
  SerialUSB.begin(115200);
  SerialUSB.setTimeout(100);

  // Servos remain detached so startup cannot move a shifted joint.
}

void loop() {
  bool connected = SerialUSB.dtr();
  if (!connected) {
    if (wasConnected && sessionMoved) {
      moveAllServosToZero();
    }
    wasConnected = false;
    sessionMoved = false;
    delay(10);
    return;
  }

  if (!wasConnected) {
    wasConnected = true;
    sessionMoved = false;
  }

  if (SerialUSB.available() <= 0) {
    return;
  }

  String command = SerialUSB.readStringUntil('\n');
  command.trim();

  int servoIndices[NUM_SERVOS];
  int offsets[NUM_SERVOS];
  int commandCount = 0;
  int start = 0;

  while (start < command.length() && commandCount < NUM_SERVOS) {
    int end = command.indexOf(',', start);
    if (end == -1) {
      end = command.length();
    }

    int split = command.indexOf(':', start);
    if (split == -1 || split >= end) {
      SerialUSB.println("Invalid command");
      return;
    }

    int servoIndex = command.substring(start, split).toInt();
    if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
      SerialUSB.println("Invalid servo index");
      return;
    }

    servoIndices[commandCount] = servoIndex;
    offsets[commandCount] = command.substring(split + 1, end).toInt();
    commandCount++;
    start = end + 1;
  }

  for (int i = 0; i < commandCount; ++i) {
    writeServoOffset(servoIndices[i], offsets[i]);
  }
  sessionMoved = sessionMoved || commandCount > 0;

  SerialUSB.print("Received offsets: ");
  for (int i = 0; i < commandCount; ++i) {
    SerialUSB.print(servoIndices[i]);
    SerialUSB.print(":");
    SerialUSB.print(offsets[i]);
    if (i < commandCount - 1) {
      SerialUSB.print(",");
    }
  }
  SerialUSB.println();
}
