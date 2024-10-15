const int NUM_SERVOS = 9;


int check(int value, int min, int max){
  return (value + min < max) ? (min + value) : ((min + value > max ) ? max : value);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("15 = ");
  Serial.print(check(15, 5, 20));

  Serial.println("5 = ");
  Serial.print(check(15, 5, 20));

  Serial.println("14 = ");
  Serial.print(check(15, 5, 20));
}

void loop() {
  if (1) {
    String command = "2:69,3:80,4:93,8:40";

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

    // for (int i = 0; i < angleIndex; ++i) {
    //   int servoIndex = servosToMove[i];
    //   servos[servoIndex].write(angles[i]);
    // }

    // Send back the received angles
    Serial.print("Received angles: ");
    for (int i = 0; i < angleIndex; ++i) {
      Serial.print(angles[i]);
      if (i < angleIndex - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
    Serial.print("Servos: ");
    for (int i = 0; i < angleIndex; ++i) {
      Serial.print(servosToMove[i]-2);
      if (i < angleIndex - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
  }

  delay(3000);
}
