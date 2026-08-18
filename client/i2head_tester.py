"""
i2head_tester.py

Interactive CLI tool for testing i2Head PCA9685 servos via Arduino.
Works with the data-driven firmware (CFG protocol).

Requires a YAML config file to set servo limits on the Arduino.

Usage:
  python3 client/i2head_tester.py <config.yaml>                  # default /dev/ttyACM0
  python3 client/i2head_tester.py <config.yaml> /dev/ttyACM1     # custom port
  python3 client/i2head_tester.py <config.yaml> /dev/ttyACM0 115200
"""

import sys
import serial
import time
import yaml

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def load_config(path):
    """Load YAML and return list of (name, cfg) sorted by channel."""
    with open(path) as f:
        config = yaml.safe_load(f)
    servos = []
    if "servos" in config:
        for name, cfg in config["servos"].items():
            servos.append((name, cfg))
    servos.sort(key=lambda x: x[1].get("channel", 0))
    return servos


def send_cfg(ser, servos):
    """Send full config to Arduino firmware."""
    for name, cfg in servos:
        ch = cfg.get("channel", 0)
        cmd = f"CFG:{ch}:{cfg.get('pulse_min',500)}:{cfg.get('pulse_max',2500)}:{cfg.get('angle_min',0)}:{cfg.get('angle_max',180)}:{cfg.get('home',90)}:{1 if cfg.get('reversed',False) else 0}\n"
        ser.write(cmd.encode())
        resp = ser.readline().decode().strip()
        print(f"  {cmd.strip()} -> {resp}")

    ser.write(b"CFG_DONE\n")
    print(f"  CFG_DONE -> {ser.readline().decode().strip()}")

    ser.write(b"HOME\n")
    print(f"  HOME -> {ser.readline().decode().strip()}")


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0 if sys.argv[1:] else 1)

    config_path = sys.argv[1]
    port = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PORT
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_BAUD

    servos = load_config(config_path)
    print(f"Loaded {len(servos)} servos from {config_path}")

    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(f"Connected to {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    time.sleep(2)

    while 1:
        feedback = ser.readline().decode().strip()
        print(f"Arduino: {feedback}")

        if feedback == "HOME_OK":
            break
        
        
    send_cfg(ser, servos)
    print(f"Configured {len(servos)} servos\n")

    for name, cfg in servos:
        print(f"{name} : ch {cfg.get('channel')}");
    
    print("Commands:")
    print("  ch:ang,ch:ang,...  Set servo channels to angles (Arduino clamps/inverts)")
    print("  home               Move all to home")
    print("  sweep              Test all configured channels")
    print("  quit               Exit\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        if cmd == "quit":
            break

        if cmd == "home":
            ser.write(b"HOME\n")
            print(ser.readline().decode().strip())
            continue

        if cmd == "sweep":
            print("Sweeping configured channels...")
            for name, cfg in servos:
                ch = cfg.get("channel", 0)
                a_min = cfg.get("angle_min", 0)
                a_max = cfg.get("angle_max", 180)
                for ang in range(a_min, a_max + 1, max(1, (a_max - a_min) // 6)):
                    ser.write(f"{ch}:{ang}\n".encode())
                    ser.readline()  # consume OK response to avoid buffer overflow
                    time.sleep(0.05)
                for ang in range(a_max, a_min - 1, -max(1, (a_max - a_min) // 6)):
                    ser.write(f"{ch}:{ang}\n".encode())
                    ser.readline()
                    time.sleep(0.05)
            print("Sweep done")
            
            ser.write(b"HOME\n")
            print(ser.readline().decode().strip())
            
            continue

        # Parse and send channel:angle pairs (Arduino clamps/inverts)
        pairs = []
        for part in cmd.split(","):
            part = part.strip()
            if ":" not in part:
                continue
            try:
                ch_str, ang_str = part.split(":", 1)
                ch = int(ch_str.strip())
                ang = int(ang_str.strip())
                pairs.append(f"{ch}:{ang}")
            except ValueError:
                print(f"  Invalid: {part}")
                continue

        if pairs:
            line = ",".join(pairs) + "\n"
            print(line.encode())
            ser.write(line.encode())
            feedback = ser.readline().decode().strip()
            print(feedback)
        else:
            print("  Unknown command")

    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
