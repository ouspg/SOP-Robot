#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define MAX_CH 16
#define BAUD 115200
#define PWM_FREQ 50
#define MIN_PULSE 500
#define MAX_PULSE 2500

int16_t minPulse[MAX_CH], maxPulse[MAX_CH];
int16_t minAngle[MAX_CH], maxAngle[MAX_CH], homeAngle[MAX_CH];
bool reversed[MAX_CH], configured[MAX_CH];
uint8_t numCh;
bool cfgDone;
unsigned long lastAct;

static const struct {
    int16_t minP;
    int16_t maxP;
    int16_t minA;
    int16_t maxA;
    int16_t home;
    bool rev;
} DEFAULT_CFG[MAX_CH] = {
    // Adjust these values to match your 3D-printed parts' mechanical limits.
    // After tuning, the Python bridge (i2head_bridge_node.py) will override
    // these via CFG: commands from config/i2head.yaml at runtime.
    //                         minP  maxP  minA maxA home rev
    /*  0: */ {600, 2400, 60, 100, 80, false},
    /*  1: */ {600, 2400, 90, 110, 100, false},
    /*  2: */ {600, 2400, 80, 100, 90, true},
    /*  3: */ {600, 2400, 80, 100, 95, true},
    /*  4: */ {600, 2400, 90, 110, 90, false},
    /*  5: */ {600, 2400, 80, 120, 90, false},
    /*  6: */ {600, 2400, 80, 120, 90, true},
    /*  7: */ {600, 2400, 0, 180, 90, false},
    /*  8: */ {600, 2400, 60, 120, 90, false},
    /*  9: */ {600, 2400, 60, 120, 90, true},
    /* 10: */ {600, 2400, 80, 100, 90, false},
    /* 11: */ {600, 2400, 80, 100, 90, true},
    /* 12: */ {600, 2400, 0, 180, 90, false},
    /* 13: */ {600, 2400, 0, 180, 90, false},
    /* 14: */ {600, 2400, 60, 120, 90, false},
    /* 15: */ {600, 2400, 30, 90, 30, false},
};

void setPWM(uint8_t ch, int16_t angle) {
  if (ch >= MAX_CH) return;
  int16_t a = reversed[ch] ? (180 - angle) : angle;
  a = constrain(a, minAngle[ch], maxAngle[ch]);

  int16_t pmin = configured[ch] ? minPulse[ch] : MIN_PULSE;
  int16_t pmax = configured[ch] ? maxPulse[ch] : MAX_PULSE;

  // Map the constrained angle over the full 0-180 range to pulse width.
  // minAngle/maxAngle are software end-stops only, not the map input range.
  int16_t pulse = map(a, 0, 180, pmin, pmax);
  pwm.setPWM(ch, 0, pulse * 4096L / 20000);
}

void home() {
    Serial.println(F("Setting servos to HOME"));
    for (uint8_t i = 0; i < numCh; i++)
        if (configured[i]) setPWM(i, homeAngle[i]);
    Serial.println(F("HOME_OK"));
    Serial.flush();
}

void testAll() {
    // Sweep each configured channel within its own min/max angle range.
    // Channels where minAngle == maxAngle (fixed) are skipped.
    for (uint8_t ch = 0; ch < numCh; ch++) {
        if (!configured[ch]) continue;
        int16_t aMin = minAngle[ch];
        int16_t aMax = maxAngle[ch];
        if (aMin == aMax) continue;

        setPWM(ch, homeAngle[ch]);

        Serial.print(F("TEST:ch"));
        Serial.print(ch);
        Serial.print(': ');
        Serial.print(aMin);
        Serial.print(F(" -> "));
        Serial.println(aMax);
        Serial.flush();

        for (int d = aMin; d <= aMax; d += 2) {
            setPWM(ch, d);
            delay(80);
        }

        Serial.print(F("TEST:ch")); Serial.print(ch);
        Serial.print(':'); Serial.print(aMax);
        Serial.print(F("->")); Serial.println(aMin);
        Serial.flush();

        for (int d = aMax; d >= aMin; d -= 2) {
            setPWM(ch, d);
            delay(80);
        }

        setPWM(ch, homeAngle[ch]);
        delay(300);
    }

    Serial.println(F("TEST_DONE"));
    Serial.flush();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(BAUD);

    // Apply safe defaults. The Python bridge later overrides via CFG: commands.
    for (uint8_t i = 0; i < MAX_CH; i++) {
        minPulse[i] = DEFAULT_CFG[i].minP;
        maxPulse[i] = DEFAULT_CFG[i].maxP;
        minAngle[i] = DEFAULT_CFG[i].minA;
        maxAngle[i] = DEFAULT_CFG[i].maxA;
        homeAngle[i] = DEFAULT_CFG[i].home;
        reversed[i] = DEFAULT_CFG[i].rev;
        configured[i] = true;
    }
    numCh = MAX_CH;
    cfgDone = true;

    for (uint8_t i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100);
    }
    
    Serial.println(F("i2head starting..."));
    if (!pwm.begin()) {
        Serial.println(F("WARN:PCA9685 not found"));
    }
    else {
        pwm.setPWMFreq(PWM_FREQ);
        Serial.println(F("PCA9685 OK"));
    }

    Serial.println(F("Ready"));
    home();
}

void loop() {
    if (Serial.available() > 0) {
        String s = Serial.readStringUntil('\n');
        s.trim();
        if (!s.length()) return;
        lastAct = millis();

        if (s == F("TEST")) {
            testAll();
            return;
        }
        if (s == F("HOME")) { home(); return; }
        if (s.startsWith(F("CFG:"))) {
            int16_t v[7], idx = 0, pos = 4;
            for (; idx < 7; idx++) {
                int c = s.indexOf(':', pos);
                v[idx] = s.substring(pos, c < 0 ? s.length() : c).toInt();
                if (c < 0) break;
                pos = c + 1;
            }
            uint8_t ch = v[0];
            if (ch < MAX_CH) {
                minPulse[ch] = v[1]; maxPulse[ch] = v[2];
                minAngle[ch] = v[3]; maxAngle[ch] = v[4];
                homeAngle[ch] = v[5]; reversed[ch] = (v[6] == 1);
                configured[ch] = true;
                if (ch >= numCh) numCh = ch + 1;
                Serial.print(F("CFG_OK:")); Serial.println(ch);
            }
            return;
        }

        if (s == F("CFG_DONE")) {
            cfgDone = true;
            Serial.print(F("CFG_DONE:")); Serial.print(numCh); Serial.println(F("ch"));
            return;
        }

        if (cfgDone) {
            uint8_t cnt = 0, pos = 0;
            while (pos < (int)s.length()) {
                int c = s.indexOf(':', pos), cm = s.indexOf(',', pos);
                if (c < 0) break;
                uint8_t ch = s.substring(pos, c).toInt();
                int16_t ang = s.substring(c + 1, cm < 0 ? s.length() : cm).toInt();
                if (ch < numCh && configured[ch]) {
                    setPWM(ch, ang);
                    cnt++;
                }
                pos = cm < 0 ? s.length() : cm + 1;
            }
            Serial.print(F("OK:"));
            Serial.print(cnt);
            Serial.println(F(" sv"));
        }
        else {
            Serial.println(F("ERR:nocfg"));
        }
    }

}
