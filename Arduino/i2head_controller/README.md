# i2Head Controller — Quick Start Guide

## Wiring

| PCA9685 | Arduino Uno |
|---------|-------------|
| SCL     | A5          |
| SDA     | A4          |
| VCC     | 5V          |
| GND     | GND         |

Servo signal wires connect to PCA9685 channels 0–15.

---

## 1. Flash the Firmware

```bash
arduino-cli compile --fqbn arduino:avr:uno \
  Arduino/i2head_controller/i2head_controller.ino

arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyACM0 \
  Arduino/i2head_controller/i2head_controller.ino
```

On boot, all 16 channels are pre-configured with safe defaults from
`DEFAULT_CFG[]` in the `.ino` file. Servos move to home (90°) immediately.
No auto-test runs.

To verify:

```bash
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=3)
time.sleep(3)
while ser.in_waiting:
    print(ser.readline().decode().strip())
ser.close()
"
```

Expected output:

```
i2head starting...
PCA9685 OK
Ready
HOME_OK
```

If you see `WARN:PCA9685 not found`, check wiring.

---

## 2. Send YAML Config

Uncomment the servos you have wired in `config/i2head.yaml`, then:

```bash
python3 client/i2head_tester.py /dev/ttyACM0 115200 config/i2head.yaml
```

This sends `CFG:` commands for each uncommented servo, then `CFG_DONE`
to apply them. The YAML values override the firmware defaults.

---

## 3. Test Servos

### Manual commands via serial monitor

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

| Command | Action |
|---------|--------|
| `HOME` | All servos to home position |
| `TEST` | Sweep all channels 0°→180°→0° |
| `0:90` | Move channel 0 to 90° |
| `0:45,1:135` | Move multiple channels at once |

### Interactive tester (recommended)

```bash
python3 client/i2head_tester.py /dev/ttyACM0 115200
```

Once connected, available commands:

```
> home              # all servos to home
> sweep             # step each channel through 0→180→0 one at a time
> 0:90,1:45         # set specific channels
> config config/i2head.yaml   # (re)load YAML config
> quit
```

### Tune mechanical limits

Edit `DEFAULT_CFG[]` in `Arduino/i2head_controller/i2head_controller.ino`
to restrict range for specific channels (e.g. `{600, 2400, 0, 60, 30, false}`
limits angle to 0–60° with home at 30°). Re-flash after changes.

---

## Serial Protocol Reference

| Command | Format | Description |
|---------|--------|-------------|
| Config | `CFG:ch:minP:maxP:minA:maxA:home:rev` | Configure one channel |
| Config done | `CFG_DONE` | Apply config, move to home |
| Home | `HOME` | Move all to home |
| Move | `ch:ang,ch:ang,...` | Set channels to angles |
| Test | `TEST` | Sweep all channels (use with caution) |

## Architecture

```
config/i2head.yaml          source of truth (servo limits, channels)
       |
client/i2head_bridge_node.py  loads YAML, sends CFG: via serial
       |                       (or: client/i2head_tester.py)
       v
Arduino/i2head_controller.ino  stores config, clamps angles,
       |                        maps angle→pulse, drives PCA9685
       v
PCA9685 → RC servos
```
