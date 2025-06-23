#include <AUnit.h>
using namespace aunit;

const int NUM_SERVOS = 9;

String extractServos(String command) {
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

  // Send back the received angles
  String string1;

  for (int i = 0; i < angleIndex; ++i) {
    string1.concat(servosToMove[i]);
    if (i < angleIndex - 1) {
      string1.concat(",");
    }
  }

  return string1;
}

String extractAngles(String command) {
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

  // Send back the received angles
  String string1;

  for (int i = 0; i < angleIndex; ++i) {
    string1.concat(angles[i]);
    if (i < angleIndex - 1) {
      string1.concat(",");
    }
  }

  return string1;
}

test(extractCommand) {
  assertEqual(extractAngles("2:69,3:80,4:93,8:40"), (String) "69,80,93,40");
  assertEqual(extractServos("2:69,3:80,4:93,8:40"), (String) "2,3,4,8");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  TestRunner::run();
}
