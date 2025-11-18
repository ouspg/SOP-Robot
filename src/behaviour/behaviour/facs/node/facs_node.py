import math
import random
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from interface.msg import FacsAU

from core.util import run_node


class ActiveAU:
    """
    Tracks a single AU's current activation state.

    intensity: 0.0 (neutral) to 1.0 (full activation)
    priority:  higher value wins when multiple AUs compete
    """

    def __init__(self, au_id: int, intensity: float, priority: int, duration_ns: int):
        self.au_id = au_id
        self.intensity = intensity
        self.priority = priority
        self.duration_ns = duration_ns


class FACSNode(Node):
    """
    FACS expression manager for i2head.

    Receives Action Unit activations or named expressions,
    blends them additively, and publishes servo positions
    as JointTrajectory to /i2head/joint_commands.

    All AU→servo mappings come from the YAML config file —
    no servo names are hardcoded in this node.
    """

    def __init__(self):
        super().__init__("facs")

        self.declare_parameter("facs_config", "")
        self.declare_parameter("au_topic", "/facs/au")
        self.declare_parameter("expression_topic", "/facs/expression")
        self.declare_parameter("command_topic", "/i2head/joint_commands")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("neutral_after_sec", 3.0)
        self.declare_parameter("idle_expression_chance", 0.0)
        self.declare_parameter("idle_interval_sec", 8.0)

        config_path = self.get_parameter("facs_config").value
        au_topic = self.get_parameter("au_topic").value
        expr_topic = self.get_parameter("expression_topic").value
        cmd_topic = self.get_parameter("command_topic").value
        self.neutral_after_sec = self.get_parameter("neutral_after_sec").value
        self.idle_chance = self.get_parameter("idle_expression_chance").value
        self.idle_interval = self.get_parameter("idle_interval_sec").value

        # AU definitions: au_id → {servo_name: {neutral, activated}}
        self.au_defs: Dict[int, dict] = {}
        # Expression definitions: name → {aus: {au_id: intensity}, duration, sequence}
        self.expr_defs: Dict[str, dict] = {}
        # Currently active AUs (au_id → ActiveAU)
        self.active_aus: Dict[int, ActiveAU] = {}
        self.last_expression_time = 0.0
        # Cached servo angles for smooth (damped) transitions
        self.current_angles: Dict[str, float] = {}
        self._neutral_timer = None
        self._seq_timer = None
        self._seq_steps = []
        self._seq_index = 0
        self._seq_name = ""

        self._load_config(config_path)

        self.cmd_pub = self.create_publisher(JointTrajectory, cmd_topic, 10)
        self.create_subscription(FacsAU, au_topic, self.au_callback, 10)
        self.create_subscription(String, expr_topic, self.expression_callback, 10)

        if self.idle_chance > 0:
            self.create_timer(self.idle_interval, self._idle_timer_callback)

        expr_list = ", ".join(sorted(self.expr_defs.keys()))
        self.get_logger().info(
            f"Node [facs] initialized. {len(self.au_defs)} AUs, "
            f"{len(self.expr_defs)} expressions: [{expr_list}]"
        )
        self.get_logger().info(
            f"  Sub: {au_topic} (interface/msg/FacsAU)"
        )
        self.get_logger().info(
            f"  Sub: {expr_topic} (std_msgs/String)"
        )
        self.get_logger().info(
            f"  Pub: {cmd_topic} (trajectory_msgs/JointTrajectory)"
        )

    def _load_config(self, path: str):
        """Load AU and expression definitions from a YAML file."""
        if not path:
            self.get_logger().warn("No facs_config parameter set — no AUs loaded")
            return

        try:
            import yaml
            import os
        except ImportError:
            self.get_logger().error("yaml not available, cannot load FACS config")
            return

        if not os.path.isfile(path):
            self.get_logger().warn(f"FACS config not found: {path}")
            return

        with open(path) as f:
            data = yaml.safe_load(f)

        facs = data.get("facs", {})

        for key, cfg in facs.get("action_units", {}).items():
            if key.startswith("au_"):
                au_id = int(key[3:])
                self.au_defs[au_id] = cfg

        for name, cfg in facs.get("expressions", {}).items():
            self.expr_defs[name] = cfg

        self.get_logger().info(
            f"Loaded {len(self.au_defs)} AUs, {len(self.expr_defs)} expressions "
            f"from {path}"
        )

    def au_callback(self, msg: FacsAU):
        """
        Handle a single AU activation command.

        Intensity 0.0 removes the AU; otherwise it is stored
        and blended with any other active AUs.
        """
        if msg.au_id not in self.au_defs:
            self.get_logger().warn(f"Unknown AU: {msg.au_id}")
            return

        self.get_logger().info(
            f"AU {msg.au_id}: intensity={msg.intensity:.2f}, priority={msg.priority}"
        )

        if msg.intensity <= 0.0:
            self.active_aus.pop(msg.au_id, None)
        else:
            self.active_aus[msg.au_id] = ActiveAU(
                msg.au_id, min(1.0, msg.intensity), msg.priority, msg.duration_ns
            )

        self._blend_and_publish()

    def expression_callback(self, msg: String):
        """
        Handle a named expression (e.g. "smile", "blink").

        Expressions can be static (set of AUs) or sequenced
        (list of timed steps, e.g. blink: close → open).
        """
        name = msg.data.strip().lower()
        if name not in self.expr_defs:
            self.get_logger().warn(f"Unknown expression: '{name}'")
            return

        self.get_logger().info(f"Expression begin: {name}")
        expr = self.expr_defs[name]

        if "sequence" in expr:
            self._execute_sequence(name, expr["sequence"])
        else:
            self._apply_expression_aus(expr.get("aus", {}), expr.get("duration", 0.3))
            self._schedule_neutral()
            self.get_logger().info(f"Expression end: {name}")

    def _apply_expression_aus(self, au_map: Dict[int, float], duration_sec: float):
        """Clear all active AUs and apply a new set (used by expressions)."""
        self.active_aus.clear()
        duration_ns = int(duration_sec * 1e9)
        for au_id, intensity in au_map.items():
            if au_id in self.au_defs and intensity > 0:
                self.active_aus[au_id] = ActiveAU(au_id, intensity, 2, duration_ns)
        self._blend_and_publish()

    def _execute_sequence(self, name: str, steps: list):
        """Start a multi-step expression sequence (e.g. blink = close + open)."""
        self._seq_timer = None
        self._seq_name = name
        self._seq_steps = list(steps)
        self._seq_index = 0
        self._execute_seq_step()

    def _execute_seq_step(self):
        """Execute one step of a sequence, schedule the next if any."""
        if self._seq_index >= len(self._seq_steps):
            self.get_logger().info(f"Expression end: {self._seq_name}")
            self._schedule_neutral()
            return

        step = self._seq_steps[self._seq_index]
        dur = step.get("duration", 0.15)
        self._apply_expression_aus(step.get("aus", {}), dur)
        self._seq_index += 1

        if self._seq_index < len(self._seq_steps):
            if self._seq_timer:
                self._seq_timer.cancel()
            self._seq_timer = self.create_timer(dur + 0.05, self._execute_seq_step)

    def _blend_and_publish(self):
        """
        Blend all active AUs into servo angles and publish JointTrajectory.

        Blending rules:
        - AUs are grouped by priority; only the highest-priority group is used.
        - Within that group, AU effects are summed additively starting from neutral.
        - Servo angles are damped (50% blend with previous value) to avoid snaps.
        """
        if not self.active_aus:
            return

        # Group AUs by priority
        priority_groups: Dict[int, List[ActiveAU]] = {}
        for au in self.active_aus.values():
            priority_groups.setdefault(au.priority, []).append(au)

        # Only the highest-priority level is expressed
        top_priority = max(priority_groups.keys())
        blended: Dict[str, float] = {}

        for au in priority_groups[top_priority]:
            cfg = self.au_defs.get(au.au_id)
            if not cfg:
                continue
            for srv, scfg in cfg.get("servos", {}).items():
                neutral = scfg.get("neutral", 90.0)
                activated = scfg.get("activated", neutral)
                if srv not in blended:
                    blended[srv] = neutral
                blended[srv] += (activated - neutral) * au.intensity

        # Dampen transitions to avoid abrupt servo snaps
        for name in list(blended.keys()):
            current = self.current_angles.get(name, blended[name])
            diff = blended[name] - current
            blended[name] = current + diff * 0.5
            self.current_angles[name] = blended[name]

        if not blended:
            return

        msg = JointTrajectory()
        msg.joint_names = list(blended.keys())
        point = JointTrajectoryPoint()
        point.positions = [blended[j] for j in msg.joint_names]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points.append(point)
        self.cmd_pub.publish(msg)

    def _schedule_neutral(self):
        """Set a timer to return to neutral expression after a delay."""
        self.last_expression_time = self.get_clock().now().nanoseconds / 1e9
        if self.neutral_after_sec <= 0:
            return
        if self._neutral_timer:
            self._neutral_timer.cancel()
        self._neutral_timer = self.create_timer(
            self.neutral_after_sec, self._return_to_neutral
        )

    def _return_to_neutral(self):
        """Clear all AUs and publish neutral pose."""
        if self._neutral_timer:
            self._neutral_timer.cancel()
            self._neutral_timer = None
        self.active_aus.clear()
        self._blend_and_publish()

    def _idle_timer_callback(self):
        """Periodically trigger a random expression when no AU is active."""
        if self.active_aus:
            return
        if random.random() >= self.idle_chance:
            return

        candidates = [n for n in self.expr_defs if n not in ("neutral", "blink")]
        if not candidates:
            return

        chosen = random.choice(candidates)
        self.get_logger().info(f"Idle expression: {chosen}")
        msg = String()
        msg.data = chosen
        self.expression_callback(msg)


def main(args=None):
    run_node(FACSNode)


if __name__ == "__main__":
    main()
