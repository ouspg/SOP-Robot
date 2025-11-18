import math
import random
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String

from core.util import run_node


class HeadGestureNode(Node):
    
    def __init__(self):
        super().__init__("head_gesture")

        self.declare_parameter("head_pan_min_deg", 0.0)
        self.declare_parameter("head_pan_max_deg", 180.0)
        self.declare_parameter("head_pan_home_deg", 90.0)

        self.declare_parameter("eye_h_min_deg", 40.0)
        self.declare_parameter("eye_h_max_deg", 140.0)
        self.declare_parameter("eye_h_home_deg", 90.0)

        self.declare_parameter("eye_v_min_deg", 50.0)
        self.declare_parameter("eye_v_max_deg", 130.0)
        self.declare_parameter("eye_v_home_deg", 90.0)

        self.declare_parameter("jaw_min_deg", 80.0)
        self.declare_parameter("jaw_max_deg", 150.0)
        self.declare_parameter("jaw_home_deg", 90.0)

        self.declare_parameter("head_pan_joint", "head_pan")
        self.declare_parameter("eye_left_h_joint", "left_eye_horizontal")
        self.declare_parameter("eye_left_v_joint", "left_eye_vertical")
        self.declare_parameter("eye_right_h_joint", "right_eye_horizontal")
        self.declare_parameter("eye_right_v_joint", "right_eye_vertical")
        self.declare_parameter("mouth_jaw_joint", "mouth_jaw")

        self.declare_parameter("i2head_joint_commands_topic", "/i2head/joint_commands")
        self.declare_parameter("shake_magnitude", 30.0)
        self.declare_parameter("shake_repetitions", 3)
        self.declare_parameter("shake_duration", 0.2)

        self.HEAD_PAN_MIN_DEG = self.get_parameter("head_pan_min_deg").value
        self.HEAD_PAN_MAX_DEG = self.get_parameter("head_pan_max_deg").value
        self.HEAD_PAN_HOME_DEG = self.get_parameter("head_pan_home_deg").value

        self.EYE_H_MIN_DEG = self.get_parameter("eye_h_min_deg").value
        self.EYE_H_MAX_DEG = self.get_parameter("eye_h_max_deg").value
        self.EYE_H_HOME_DEG = self.get_parameter("eye_h_home_deg").value

        self.EYE_V_MIN_DEG = self.get_parameter("eye_v_min_deg").value
        self.EYE_V_MAX_DEG = self.get_parameter("eye_v_max_deg").value
        self.EYE_V_HOME_DEG = self.get_parameter("eye_v_home_deg").value

        self.JAW_MIN_DEG = self.get_parameter("jaw_min_deg").value
        self.JAW_MAX_DEG = self.get_parameter("jaw_max_deg").value
        self.JAW_HOME_DEG = self.get_parameter("jaw_home_deg").value

        self.HEAD_PAN_JOINT = self.get_parameter("head_pan_joint").value
        self.EYE_LEFT_H_JOINT = self.get_parameter("eye_left_h_joint").value
        self.EYE_LEFT_V_JOINT = self.get_parameter("eye_left_v_joint").value
        self.EYE_RIGHT_H_JOINT = self.get_parameter("eye_right_h_joint").value
        self.EYE_RIGHT_V_JOINT = self.get_parameter("eye_right_v_joint").value
        self.MOUTH_JAW_JOINT = self.get_parameter("mouth_jaw_joint").value

        self.I2HEAD_TOPIC = self.get_parameter("i2head_joint_commands_topic").value

        self.SHAKE_MAGNITUDE = self.get_parameter("shake_magnitude").value
        self.SHAKE_REPETITIONS = self.get_parameter("shake_repetitions").value
        self.SHAKE_DURATION = self.get_parameter("shake_duration").value

        self.head_pan_deg: Optional[float] = None
        self.eye_h_deg: Optional[float] = None
        self.eye_v_deg: Optional[float] = None
        self.jaw_deg: Optional[float] = None

        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, self.I2HEAD_TOPIC, 10
        )

        self.create_subscription(
            String,
            "/head_gesture_command",
            self.head_gesture_callback,
            10
        )

        self.get_logger().info("Node [head_gesture] Initialized.")
        self.get_logger().info(
            f"  Sub: /head_gesture_command (std_msgs/String)"
        )
        self.get_logger().info(
            f"  Pub: {self.I2HEAD_TOPIC} (trajectory_msgs/JointTrajectory)"
        )
        self._print_help()

    def _print_help(self):
        self.get_logger().info("commands {shake, glance}.")
        
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))

    def _init_joint_state(self):
        if self.head_pan_deg is None:
            self.head_pan_deg = self.HEAD_PAN_HOME_DEG
        if self.eye_h_deg is None:
            self.eye_h_deg = self.EYE_H_HOME_DEG
        if self.eye_v_deg is None:
            self.eye_v_deg = self.EYE_V_HOME_DEG
        if self.jaw_deg is None:
            self.jaw_deg = self.JAW_HOME_DEG

    def _publish_all_joints(self, duration_sec: float):
        self._init_joint_state()
        msg = JointTrajectory()
        msg.joint_names = [
            self.HEAD_PAN_JOINT,
            self.EYE_LEFT_H_JOINT,
            self.EYE_LEFT_V_JOINT,
            self.EYE_RIGHT_H_JOINT,
            self.EYE_RIGHT_V_JOINT,
            self.MOUTH_JAW_JOINT,
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            self.head_pan_deg,
            self.eye_h_deg,
            self.eye_v_deg,
            self.eye_h_deg,
            self.eye_v_deg,
            self.jaw_deg,
        ]
        point.time_from_start = Duration(
            sec=math.floor(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )
        msg.points.append(point)
        self.joint_cmd_pub.publish(msg)

    def head_gesture_callback(self, msg: String):
        command = msg.data.split(",")[0].strip().lower()

        if command == "shake":
            self.get_logger().info("Executing gesture: shake")
            self._execute_shake()
            self.get_logger().info("Gesture complete: shake")
        elif command == "glance":
            self.get_logger().info("Executing gesture: glance")
            self._execute_glance()
            self.get_logger().info("Gesture complete: glance")
        elif command == "eyeroll":
            self.get_logger().info("Executing gesture: eyeroll")
            self._execute_eyeroll()
            self.get_logger().info("Gesture complete: eyeroll")
        else:
            self.get_logger().warn(f"Unknown gesture: {command}")

    def _execute_shake(self):
        self._init_joint_state()

        original_pan = self.head_pan_deg

        for _ in range(self.SHAKE_REPETITIONS):
            self.head_pan_deg = self._clamp(
                original_pan + self.SHAKE_MAGNITUDE,
                self.HEAD_PAN_MIN_DEG,
                self.HEAD_PAN_MAX_DEG,
            )
            self._publish_all_joints(self.SHAKE_DURATION)
            time.sleep(self.SHAKE_DURATION + 0.05)

            self.head_pan_deg = self._clamp(
                original_pan - self.SHAKE_MAGNITUDE,
                self.HEAD_PAN_MIN_DEG,
                self.HEAD_PAN_MAX_DEG,
            )
            self._publish_all_joints(self.SHAKE_DURATION)
            time.sleep(self.SHAKE_DURATION + 0.05)

        self.head_pan_deg = original_pan
        self._publish_all_joints(self.SHAKE_DURATION)

    def _execute_glance(self, delay: float = 0.5):
        self._init_joint_state()

        orig_h = self.eye_h_deg
        orig_v = self.eye_v_deg

        self.eye_h_deg = self._clamp(
            self.eye_h_deg + random.choice([-25.0, 25.0]),
            self.EYE_H_MIN_DEG,
            self.EYE_H_MAX_DEG,
        )
        self.eye_v_deg = self._clamp(
            self.eye_v_deg + random.uniform(-10.0, 10.0),
            self.EYE_V_MIN_DEG,
            self.EYE_V_MAX_DEG,
        )
        self._publish_all_joints(0.3)
        time.sleep(0.3 + 0.05)

        self.eye_h_deg = orig_h
        self.eye_v_deg = orig_v
        self._publish_all_joints(delay)

    def _execute_eyeroll(self, step_duration: float = 0.15):
        self._init_joint_state()

        orig_h = self.eye_h_deg
        orig_v = self.eye_v_deg

        h_range = (self.EYE_H_MAX_DEG - self.EYE_H_MIN_DEG) / 2
        v_range = (self.EYE_V_MAX_DEG - self.EYE_V_MIN_DEG) / 2
        home_h = self.EYE_H_HOME_DEG
        home_v = self.EYE_V_HOME_DEG

        roll = [
            (home_h, home_v - v_range * 0.6),
            (home_h + h_range * 0.6, home_v),
            (home_h, home_v + v_range * 0.6),
            (home_h - h_range * 0.6, home_v),
        ]

        for h, v in roll:
            self.eye_h_deg = self._clamp(h, self.EYE_H_MIN_DEG, self.EYE_H_MAX_DEG)
            self.eye_v_deg = self._clamp(v, self.EYE_V_MIN_DEG, self.EYE_V_MAX_DEG)
            self._publish_all_joints(step_duration)
            time.sleep(step_duration + 0.03)

        self.eye_h_deg = orig_h
        self.eye_v_deg = orig_v
        self._publish_all_joints(step_duration)


def main(args=None):
    run_node(HeadGestureNode)


if __name__ == "__main__":
    main()
