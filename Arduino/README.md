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
Receives newline-terminated `ID:angle,ID:angle,...` commands at 115200 baud.
ID is the zero-based index into `SERVO_PINS`, not a pin number:

| ID | Pin | Function | Command limits | Startup angle |
| --- | --- | --- | --- | --- |
| 0 | 3 | Right shoulder lift | 10–80° | 30° |
| 1 | 4 | Right bicep | 20–100° | 10° |
| 2 | 5 | Right upper-arm rotation | 10–180° | 0° |
| 3 | 8 | Left shoulder lift | 0–180° | 34° |
| 4 | 9 | Left bicep | 0–100° | 10° |
| 5 | 10 | Left upper-arm rotation | 0–180° | 80° |

Normal commands are truncated to integers and clamped to each servo's limits.
Startup and serial-disconnect recovery write `expectedStartingPos` directly,
including the two right-arm starting angles below their command minima.
