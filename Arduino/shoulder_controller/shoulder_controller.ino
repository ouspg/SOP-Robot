#include <Servo.h>


const int NUM_SERVOS = 9;
const int SERVO_PINS[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9,10};
const int ServoMins[NUM_SERVOS] = {0, 0, 20, 0, 0, 0, 0, 55,0};
const int ServoMax[NUM_SERVOS] = {180, 180, 100, 60, 180, 180, 100, 115,180};


int check(int value, int min, int max){
  return (value + min < max) ? (min + value) : ((min + value > max ) ? max : value);
}


Servo servos[NUM_SERVOS];

void setup() {
  SerialUSB.begin(115200);

  // Attach servos to pins
  for (int i = 0; i < NUM_SERVOS; ++i) {
    servos[i].attach(SERVO_PINS[i]);
  }
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
