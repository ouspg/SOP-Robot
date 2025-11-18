"""
i2head_bridge_node.py

Data-driven ROS2-to-Arduino serial bridge for InMoov i2Head PCA9685 servo control.

Loads ALL servo parameters from a YAML module definition file and sends
them to the Arduino firmware at startup. Firmware stays completely generic.

Usage:
  python3 i2head_bridge.py

  TODO - make ros node for this, this would allow to run
  ros2 run i2head_bridge i2head_bridge_node --ros-args \
    -p joint_config:=config/i2head.yaml
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import serial
import yaml
import os

from core.util import run_node
import time


class I2HeadBridgeNode(Node):
    """
    Bridges JointTrajectory commands to Arduino over serial.
    All servo configuration comes from YAML - no hardcoded values.
    """

    def __init__(self):
        super().__init__("i2head_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("joint_config", "../config/i2head.yaml")
        self.declare_parameter("topic_name", "/i2head/joint_commands")
        self.declare_parameter("feedback_topic", "/i2head/feedback")
        self.declare_parameter("pwm_frequency", 50)

        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        config_path = self.get_parameter("joint_config").value
        topic_name = self.get_parameter("topic_name").value
        feedback_topic = self.get_parameter("feedback_topic").value

        # --- Load full servo configuration from YAML ---
        self.servos = {}        # joint_name -> servo config dict
        self.joint_map = {}     # joint_name -> channel
        self.num_channels = 0

        if config_path and os.path.isfile(config_path):
            with open(config_path) as f:
                config = yaml.safe_load(f)
            if "servos" in config:
                for joint_name, cfg in config["servos"].items():
                    self.servos[joint_name] = cfg
                    self.joint_map[joint_name] = cfg["channel"]
                    ch = cfg["channel"]
                    if ch >= self.num_channels:
                        self.num_channels = ch + 1
            self.get_logger().info(
                f"Loaded {len(self.servos)} joints from {config_path}"
            )
        else:
            self.get_logger().warn("No joint config file specified. Use joint_config parameter.")

        # --- Serial connection ---
        self.serial = None
        if serial_port:
            try:
                self.serial = serial.Serial(serial_port, baud_rate, timeout=2)
                self.get_logger().info(f"Serial port {serial_port} opened at {baud_rate} baud")
                time.sleep(2)  # Wait for Arduino reset

                while 1:
                    feedback = ser.readline().decode().strip()
                    print(f"Arduino: {feedback}")

                if feedback == "HOME_OK":
                    break
                        
                self._flush_serial()
                self._send_config_to_arduino()
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
                self.serial = None
        else:
            self.get_logger().warn("No serial port specified. Run with serial_port:=/dev/ttyACM0")

        # --- Publisher (feedback) ---
        self.feedback_pub = self.create_publisher(String, feedback_topic, 10)

        # --- Subscriber (joint commands) ---
        self.sub = self.create_subscription(
            JointTrajectory, topic_name, self.joint_command_callback, 10
        )
        self.get_logger().info(f"Subscribed to {topic_name}")
        self.get_logger().info(
            f"  Sub: {topic_name} (trajectory_msgs/JointTrajectory)"
        )
        self.get_logger().info(
            f"  Pub: {feedback_topic} (std_msgs/String)"
        )

    def _flush_serial(self):
        """Read and discard any startup messages from Arduino."""
        while self.serial.in_waiting:
            line = self.serial.readline().decode().strip()
            self.get_logger().info(f"Arduino: {line}")

    def _send_config_to_arduino(self):
        """
        Send every servo's full configuration to Arduino.
        Firmware stores these in runtime arrays - no hardcoded limits.
        """
        sent = 0
        for joint_name, cfg in self.servos.items():
            ch = cfg["channel"]
            pulse_min = cfg.get("pulse_min", 500)
            pulse_max = cfg.get("pulse_max", 2500)
            angle_min = cfg.get("angle_min", 0)
            angle_max = cfg.get("angle_max", 180)
            home = cfg.get("home", 90)
            rev = 1 if cfg.get("reversed", False) else 0

            cmd = f"CFG:{ch}:{pulse_min}:{pulse_max}:{angle_min}:{angle_max}:{home}:{rev}\n"
            self.serial.write(cmd.encode())
            resp = self.serial.readline().decode().strip()
            self.get_logger().debug(f"  {cmd.strip()} -> {resp}")
            sent += 1

        self.serial.write(b"CFG_DONE\n")
        resp = self.serial.readline().decode().strip()
        self.get_logger().info(f"Config done: {resp}")

        # Move to home
        self.serial.write(b"HOME\n")
        resp = self.serial.readline().decode().strip()
        self.get_logger().info(f"Home: {resp}")

        self.get_logger().info(f"Configured {sent} servos on Arduino")

    def joint_command_callback(self, msg: JointTrajectory):
        """Convert JointTrajectory to channel:angle commands using YAML config."""
        if self.serial is None:
            return
        if not msg.points:
            return

        pairs = []
        for i, joint_name in enumerate(msg.joint_names):
            cfg = self.servos.get(joint_name)
            if cfg is None:
                self.get_logger().warn(f"Unknown joint: {joint_name}, skipping")
                continue

            ch = cfg["channel"]
            angle = msg.points[0].positions[i] if i < len(msg.points[0].positions) else 0.0
            angle_deg = int(round(angle))

            pairs.append(f"{ch}:{angle_deg}")

        if not pairs:
            return

        cmd = ",".join(pairs) + "\n"
        joint_summary = ", ".join(
            f"{msg.joint_names[i]}={pairs[i].split(':')[1]}°"
            for i in range(len(pairs))
        )
        self.get_logger().info(f"Bridge command: {joint_summary}")

        try:
            self.serial.write(cmd.encode())
            feedback = self.serial.readline().decode().strip()
            if feedback:
                fb_msg = String()
                fb_msg.data = feedback
                self.feedback_pub.publish(fb_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    run_node(I2HeadBridgeNode)


if __name__ == "__main__":
    main()
