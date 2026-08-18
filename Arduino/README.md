# Arduino Firmware

Firmware for Arduino-based servo controllers in the SOP-Robot system.

## i2head_controller

Controls InMoov i2Head facial expression servos via PCA9685 PWM driver.

**Hardware:**
- Arduino (Uno/Mega/Leonardo compatible)
- Adafruit 16-Channel 12-bit PWM/Servo Driver (PCA9685)
- 16+ RC servos connected to PCA9685 channels

**Protocol:**
Receives serial commands in format: `channel:angle,channel:angle,...` at 115200 baud.
Acknowledges with `OK:<command>` on successful execution.

**Dependencies:**
- Adafruit_PWMServoDriver library (install via Arduino Library Manager)
- Wire.h (built-in)

## shoulder_controller

Controls shoulder servos (RC servos with potentiometer feedback) via direct GPIO PWM.

**Protocol:**
Same format: `ID:angle,ID:angle,...` at 115200 baud.
Where ID = servo_pin (IDs 2-10 map to pins 2-10).
