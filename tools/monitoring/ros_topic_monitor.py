#!/usr/bin/env python3
"""ROS 2 topic performance monitor and graph overview generator."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
import html
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPORT_ROOT = PROJECT_ROOT / "reports"
ROBOT_STACK_CONFIG_PATH = PROJECT_ROOT / "config" / "robot_stack.yaml"
ROBOT_STACK_DEFAULTS_PATH = PROJECT_ROOT / "src" / "robot" / "config" / "robot_stack.defaults.yaml"
SYSTEM_TOPICS = {"/parameter_events", "/rosout"}
DEPRECATED_CONTROLLER_STATE_SUFFIX = "_controller/state"
DEPRECATED_CONTROLLER_STATE_TYPE = "control_msgs/msg/JointTrajectoryControllerState"
HELPER_NODE_NAMES = {
    "/sop_robot_topic_monitor",
    "/sop_robot_topic_graph",
    "/sop_robot_topic_overview",
}
DEFAULT_SAMPLE_SECONDS = 10.0
DEFAULT_PRINT_PERIOD_SECONDS = 1.0


@dataclass
class TopicMetric:
    topic: str
    type_name: str
    expected_hz: float | None = None
    count: int = 0
    first_received: float | None = None
    last_received: float | None = None
    last_report_count: int = 0
    last_report_time: float | None = None
    intervals: deque[float] = field(default_factory=lambda: deque(maxlen=512))
    latencies: deque[float] = field(default_factory=lambda: deque(maxlen=512))
    byte_sizes: deque[int] = field(default_factory=lambda: deque(maxlen=512))
    total_bytes: int = 0

    def observe(self, msg: Any, received_at: float) -> None:
        if self.last_received is not None:
            self.intervals.append(received_at - self.last_received)
        self.last_received = received_at
        if self.first_received is None:
            self.first_received = received_at
        self.count += 1

        latency = message_latency_seconds(msg, received_at)
        if latency is not None:
            self.latencies.append(latency)

        try:
            size = len(serialize_message(msg))
        except Exception:
            size = 0
        self.byte_sizes.append(size)
        self.total_bytes += size

    def recent_hz(self, now: float) -> float:
        if self.last_report_time is None:
            self.last_report_time = now
            self.last_report_count = self.count
            return 0.0
        elapsed = max(now - self.last_report_time, 1e-9)
        count_delta = self.count - self.last_report_count
        self.last_report_time = now
        self.last_report_count = self.count
        return count_delta / elapsed

    def summary(self, now: float, publishers: int = 0, subscribers: int = 0) -> dict[str, Any]:
        elapsed = max(now - (self.first_received or now), 1e-9)
        avg_hz = self.count / elapsed if self.count else 0.0
        bytes_per_second = (self.total_bytes / elapsed) if self.count else 0.0
        interval_avg = statistics.fmean(self.intervals) if self.intervals else None
        jitter_ms = stdev_ms(self.intervals)
        latency_avg = statistics.fmean(self.latencies) if self.latencies else None
        avg_bytes = statistics.fmean(self.byte_sizes) if self.byte_sizes else 0.0
        status = topic_status(avg_hz, self.expected_hz, self.count)

        return {
            "topic": self.topic,
            "type": self.type_name,
            "status": status,
            "count": self.count,
            "avg_hz": round(avg_hz, 3),
            "expected_hz": self.expected_hz,
            "expected_ratio": round(avg_hz / self.expected_hz, 3)
            if self.expected_hz and self.expected_hz > 0
            else None,
            "avg_period_ms": round(interval_avg * 1000.0, 3) if interval_avg else None,
            "p95_period_ms": percentile_ms(self.intervals, 95),
            "jitter_ms": jitter_ms,
            "avg_latency_ms": round(latency_avg * 1000.0, 3) if latency_avg is not None else None,
            "p95_latency_ms": percentile_ms(self.latencies, 95),
            "max_latency_ms": round(max(self.latencies) * 1000.0, 3) if self.latencies else None,
            "avg_bytes": round(avg_bytes, 1),
            "max_bytes": max(self.byte_sizes) if self.byte_sizes else 0,
            "kib_per_s": round(bytes_per_second / 1024.0, 3),
            "mib_per_s": round(bytes_per_second / (1024.0 * 1024.0), 3),
            "kbps": round((bytes_per_second * 8.0) / 1000.0, 3),
            "last_age_ms": round((now - self.last_received) * 1000.0, 3)
            if self.last_received
            else None,
            "publishers": publishers,
            "subscribers": subscribers,
        }


@dataclass(frozen=True)
class GraphSnapshot:
    nodes: list[str]
    topics: dict[str, list[str]]
    publishers: dict[str, list[str]]
    subscribers: dict[str, list[str]]


def percentile_ms(values: deque[float], percentile: int) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(math.ceil((percentile / 100.0) * len(ordered)) - 1, len(ordered) - 1)
    return round(ordered[index] * 1000.0, 3)


def stdev_ms(values: deque[float]) -> float | None:
    if len(values) < 2:
        return None
    return round(statistics.pstdev(values) * 1000.0, 3)


def topic_status(avg_hz: float, expected_hz: float | None, count: int) -> str:
    if count == 0:
        return "no_messages"
    if expected_hz is None or expected_hz <= 0:
        return "observed"
    ratio = avg_hz / expected_hz
    if ratio >= 0.85:
        return "ok"
    if ratio >= 0.60:
        return "slow"
    return "bad"


def message_latency_seconds(msg: Any, received_at: float) -> float | None:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    stamped_at = float(sec) + (float(nanosec) / 1_000_000_000.0)
    latency = received_at - stamped_at
    if latency < -1.0:
        return None
    return latency


def is_hidden_name(name: str) -> bool:
    return any(part.startswith("_") for part in name.split("/") if part)


def full_node_name(name: str, namespace: str) -> str:
    if namespace in {"", "/"}:
        return f"/{name}"
    return f"{namespace.rstrip('/')}/{name}"


def parse_expectations(values: list[str]) -> dict[str, float]:
    expectations: dict[str, float] = {}
    for value in values:
        if "=" not in value:
            raise SystemExit(f"Expected topic rate must be TOPIC=HZ, got {value!r}.")
        topic, raw_hz = value.split("=", 1)
        if not topic.startswith("/"):
            topic = f"/{topic}"
        try:
            expectations[topic] = float(raw_hz)
        except ValueError as exc:
            raise SystemExit(f"Invalid expected Hz in {value!r}.") from exc
    return expectations


def positive_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None


def positive_int(value: Any) -> int | None:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number > 0 else None


def load_robot_common_config() -> dict[str, Any]:
    for path in (ROBOT_STACK_CONFIG_PATH, ROBOT_STACK_DEFAULTS_PATH):
        if not path.exists():
            continue
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            continue
        common_config = data.get("common")
        if isinstance(common_config, dict):
            return common_config
    return {}


def default_robot_expectations() -> dict[str, float]:
    common_config = load_robot_common_config()
    image_hz = positive_float(common_config.get("webcam_fps"))
    face_image_hz = positive_float(common_config.get("face_image_publish_fps")) or image_hz
    publish_every = positive_int(common_config.get("face_image_publish_every_n_frames")) or 1
    max_processing_hz = positive_float(common_config.get("face_tracker_max_processing_fps"))

    if face_image_hz is not None:
        face_image_hz /= publish_every
    if face_image_hz is not None and max_processing_hz is not None:
        face_image_hz = min(face_image_hz, max_processing_hz)

    expectations = {
        "/joint_states": 20.0,
        "/voice_chatbot/status": 0.2,
    }
    if image_hz is not None:
        expectations["/image_raw"] = image_hz
    if face_image_hz is not None:
        expectations["/face_tracker/image_face"] = face_image_hz
    return expectations


def topic_allowed(topic: str, include_hidden: bool, include_system: bool) -> bool:
    if not include_system and topic in SYSTEM_TOPICS:
        return False
    if not include_hidden and is_hidden_name(topic):
        return False
    return True


def is_deprecated_controller_state_topic(topic_name: str, type_names: list[str] | tuple[str, ...]) -> bool:
    return (
        topic_name.endswith(DEPRECATED_CONTROLLER_STATE_SUFFIX)
        and DEPRECATED_CONTROLLER_STATE_TYPE in type_names
    )


def select_topics(
    node: Node,
    explicit_topics: list[str],
    include_hidden: bool,
    include_system: bool,
) -> dict[str, str]:
    topic_map: dict[str, str] = {}
    for topic, type_names in node.get_topic_names_and_types():
        if is_deprecated_controller_state_topic(topic, type_names):
            continue
        if topic_allowed(topic, include_hidden, include_system):
            topic_map[topic] = type_names[0] if type_names else ""

    if explicit_topics:
        selected: dict[str, str] = {}
        for topic in explicit_topics:
            normalized = topic if topic.startswith("/") else f"/{topic}"
            selected[normalized] = topic_map.get(normalized, "")
        return selected
    return dict(sorted(topic_map.items()))


def capture_graph(node: Node, include_hidden: bool, include_system: bool) -> GraphSnapshot:
    nodes: list[str] = []
    publishers: dict[str, set[str]] = defaultdict(set)
    subscribers: dict[str, set[str]] = defaultdict(set)
    topics: dict[str, list[str]] = {}

    for topic, type_names in node.get_topic_names_and_types():
        if is_deprecated_controller_state_topic(topic, type_names):
            continue
        if topic_allowed(topic, include_hidden, include_system):
            topics[topic] = type_names

    for name, namespace in node.get_node_names_and_namespaces():
        fqn = full_node_name(name, namespace)
        if fqn in HELPER_NODE_NAMES:
            continue
        if not include_hidden and is_hidden_name(fqn):
            continue
        nodes.append(fqn)

        for topic, _type_names in node.get_publisher_names_and_types_by_node(name, namespace):
            if topic in topics:
                publishers[topic].add(fqn)
        for topic, _type_names in node.get_subscriber_names_and_types_by_node(name, namespace):
            if topic in topics:
                subscribers[topic].add(fqn)

    return GraphSnapshot(
        nodes=sorted(nodes),
        topics=dict(sorted(topics.items())),
        publishers={topic: sorted(values) for topic, values in sorted(publishers.items())},
        subscribers={topic: sorted(values) for topic, values in sorted(subscribers.items())},
    )


class TopicMonitor:
    def __init__(
        self,
        node: Node,
        topics: dict[str, str],
        expectations: dict[str, float],
    ) -> None:
        self.node = node
        self.metrics = {
            topic: TopicMetric(
                topic=topic,
                type_name=type_name,
                expected_hz=expectations.get(topic),
            )
            for topic, type_name in topics.items()
        }
        self._subscriptions: list[Any] = []
        self.skipped_topics: dict[str, str] = {}

    def start(self) -> None:
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for topic, metric in self.metrics.items():
            if not metric.type_name:
                self.skipped_topics[topic] = "topic type was not discoverable"
                continue
            try:
                message_type = get_message(metric.type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.skipped_topics[topic] = f"cannot import {metric.type_name}: {exc}"
                continue

            def callback(msg: Any, selected_topic: str = topic) -> None:
                self.metrics[selected_topic].observe(msg, time.time())

            self._subscriptions.append(
                self.node.create_subscription(message_type, topic, callback, qos)
            )

    def summaries(self, graph: GraphSnapshot | None = None) -> list[dict[str, Any]]:
        now = time.time()
        rows = []
        for topic, metric in sorted(self.metrics.items()):
            rows.append(
                metric.summary(
                    now,
                    publishers=len(graph.publishers.get(topic, [])) if graph else 0,
                    subscribers=len(graph.subscribers.get(topic, [])) if graph else 0,
                )
            )
        return rows


def print_monitor_table(monitor: TopicMonitor, graph: GraphSnapshot | None = None) -> None:
    now = time.time()
    rows = []
    for topic, metric in sorted(monitor.metrics.items()):
        summary = metric.summary(
            now,
            publishers=len(graph.publishers.get(topic, [])) if graph else 0,
            subscribers=len(graph.subscribers.get(topic, [])) if graph else 0,
        )
        summary["recent_hz"] = round(metric.recent_hz(now), 3)
        rows.append(summary)

    columns = (
        ("status", "status", 11),
        ("topic", "topic", 36),
        ("recent_hz", "recent_hz", 9),
        ("avg_hz", "avg_hz", 8),
        ("expected_hz", "expected", 8),
        ("p95_latency_ms", "p95_laten", 9),
        ("p95_period_ms", "p95_perio", 9),
        ("mib_per_s", "MiB/s", 8),
        ("pub/sub", "pub/sub", 7),
    )
    header = "  ".join(label[:width].ljust(width) for _key, label, width in columns)
    print(f"\n{datetime.now().strftime('%H:%M:%S')} topic performance")
    print(header)
    print("-" * len(header))
    for row in rows:
        pub_sub = f"{row['publishers']}/{row['subscribers']}"
        values = {
            **row,
            "pub/sub": pub_sub,
            "expected_hz": "" if row["expected_hz"] is None else row["expected_hz"],
            "p95_latency_ms": "" if row["p95_latency_ms"] is None else row["p95_latency_ms"],
            "p95_period_ms": "" if row["p95_period_ms"] is None else row["p95_period_ms"],
        }
        print(
            "  ".join(
                str(values[key])[:width].ljust(width)
                for key, _label, width in columns
            )
        )
    if monitor.skipped_topics:
        print("\nSkipped topics:")
        for topic, reason in sorted(monitor.skipped_topics.items()):
            print(f"  {topic}: {reason}")


def spin_for_duration(
    node: Node,
    monitor: TopicMonitor,
    duration: float,
    print_period: float | None,
    include_hidden: bool,
    include_system: bool,
) -> GraphSnapshot:
    deadline = time.monotonic() + duration if duration > 0 else None
    next_print = time.monotonic() + (print_period or 0.0)
    graph = capture_graph(node, include_hidden, include_system)
    while deadline is None or time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if print_period and time.monotonic() >= next_print:
            graph = capture_graph(node, include_hidden, include_system)
            print_monitor_table(monitor, graph)
            next_print = time.monotonic() + print_period
    return capture_graph(node, include_hidden, include_system)


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def csv_escape(value: Any) -> str:
    text = "" if value is None else str(value)
    if any(char in text for char in [",", '"', "\n"]):
        return '"' + text.replace('"', '""') + '"'
    return text


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    field_names = list(rows[0])
    lines = [",".join(field_names)]
    lines.extend(
        ",".join(csv_escape(row.get(field_name)) for field_name in field_names)
        for row in rows
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def dot_id(value: str) -> str:
    safe = "".join(char if char.isalnum() else "_" for char in value)
    return safe.strip("_") or "root"


def render_dot(graph: GraphSnapshot) -> str:
    lines = [
        "digraph ros_topics {",
        "  rankdir=LR;",
        '  node [fontname="Arial", fontsize=10];',
    ]
    for node_name in graph.nodes:
        lines.append(f'  "{dot_id("node_" + node_name)}" [label="{node_name}", shape=box];')
    for topic, type_names in graph.topics.items():
        label = f"{topic}\\n{', '.join(type_names)}"
        lines.append(f'  "{dot_id("topic_" + topic)}" [label="{label}", shape=ellipse];')
        for publisher in graph.publishers.get(topic, []):
            lines.append(
                f'  "{dot_id("node_" + publisher)}" -> "{dot_id("topic_" + topic)}";'
            )
        for subscriber in graph.subscribers.get(topic, []):
            lines.append(
                f'  "{dot_id("topic_" + topic)}" -> "{dot_id("node_" + subscriber)}";'
            )
    lines.append("}")
    return "\n".join(lines) + "\n"


def metric_by_topic(metrics: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {row["topic"]: row for row in metrics}


def render_html(graph: GraphSnapshot, metrics: list[dict[str, Any]], generated_at: str) -> str:
    metrics_by_topic = metric_by_topic(metrics)
    rows = []
    svg_rows = []
    row_height = 96
    width = 1320
    y = 52
    for topic in sorted(graph.topics):
        publishers = graph.publishers.get(topic, [])
        subscribers = graph.subscribers.get(topic, [])
        metric = metrics_by_topic.get(topic, {})
        type_text = ", ".join(graph.topics.get(topic, []))
        status = str(metric.get("status", "not_sampled"))
        perf = topic_perf_label(metric)
        topic_html = html.escape(topic)
        data_text = " ".join([topic, type_text, " ".join(publishers), " ".join(subscribers)])
        svg_rows.append(
            render_svg_topic_row(topic, publishers, subscribers, type_text, metric, y, data_text)
        )
        rows.append(
            "<tr data-filter=\"{filter_text}\">"
            "<td><span class=\"status {status_class}\">{status}</span></td>"
            "<td><code>{topic}</code><div class=\"muted\">{type_text}</div></td>"
            "<td>{pubs}</td><td>{subs}</td><td>{perf}</td></tr>".format(
                filter_text=html.escape(data_text.lower(), quote=True),
                status_class=html.escape(status),
                status=html.escape(status),
                topic=topic_html,
                type_text=html.escape(type_text),
                pubs="<br>".join(html.escape(value) for value in publishers) or "<span class=\"muted\">none</span>",
                subs="<br>".join(html.escape(value) for value in subscribers) or "<span class=\"muted\">none</span>",
                perf=html.escape(perf),
            )
        )
        y += row_height

    svg_height = max(y + 32, 220)
    summary = {
        "nodes": len(graph.nodes),
        "topics": len(graph.topics),
        "edges": sum(len(v) for v in graph.publishers.values())
        + sum(len(v) for v in graph.subscribers.values()),
        "sampled_topics": len(metrics),
    }
    payload = json.dumps(
        {
            "generated_at": generated_at,
            "summary": summary,
            "graph": asdict(graph),
            "metrics": metrics,
        },
        indent=2,
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SOP-Robot ROS Topic Overview</title>
  <style>
    :root {{
      --bg: #10131a;
      --panel: #171b24;
      --ink: #eef2ff;
      --muted: #9aa4b2;
      --line: #364052;
      --topic: #2b6cb0;
      --node: #234e52;
      --ok: #2f855a;
      --slow: #b7791f;
      --bad: #c53030;
    }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--ink);
      font-family: Arial, sans-serif;
      font-size: 14px;
    }}
    header, main {{ max-width: 1480px; margin: 0 auto; padding: 18px 22px; }}
    h1 {{ margin: 0 0 8px; font-size: 24px; }}
    .muted {{ color: var(--muted); font-size: 12px; }}
    .stats {{ display: flex; gap: 12px; flex-wrap: wrap; margin-top: 14px; }}
    .stat {{ background: var(--panel); border: 1px solid var(--line); padding: 10px 12px; border-radius: 6px; }}
    input {{
      width: min(720px, calc(100vw - 48px));
      margin: 10px 0 16px;
      padding: 10px 12px;
      background: #0b0e14;
      color: var(--ink);
      border: 1px solid var(--line);
      border-radius: 4px;
    }}
    section {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
      margin-bottom: 18px;
      overflow: auto;
    }}
    .section-title {{ padding: 12px 14px; border-bottom: 1px solid var(--line); font-weight: 700; }}
    svg {{ display: block; min-width: {width}px; background: #0f141d; }}
    .box {{ fill: var(--node); stroke: #6ee7b7; stroke-width: 1; rx: 5; }}
    .topic {{ fill: var(--topic); stroke: #90cdf4; stroke-width: 1; rx: 18; }}
    .edge {{ stroke: #718096; stroke-width: 1.3; fill: none; marker-end: url(#arrow); }}
    .rowline {{ stroke: #252c3a; stroke-width: 1; }}
    text {{ fill: var(--ink); font-size: 12px; }}
    .small {{ fill: var(--muted); font-size: 11px; }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{ padding: 10px 12px; border-bottom: 1px solid var(--line); vertical-align: top; text-align: left; }}
    th {{ color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .04em; }}
    code {{ color: #bee3f8; }}
    .status {{ border-radius: 999px; padding: 3px 8px; background: #2d3748; font-size: 12px; }}
    .status.ok, .status.observed {{ background: var(--ok); }}
    .status.slow {{ background: var(--slow); }}
    .status.bad, .status.no_messages {{ background: var(--bad); }}
    details {{ margin: 18px 0; }}
    pre {{ overflow: auto; background: #0b0e14; border: 1px solid var(--line); padding: 12px; border-radius: 6px; }}
  </style>
</head>
<body>
  <header>
    <h1>SOP-Robot ROS Topic Overview</h1>
    <div class="muted">Generated {html.escape(generated_at)}</div>
    <div class="stats">
      <div class="stat"><strong>{summary["nodes"]}</strong><div class="muted">nodes</div></div>
      <div class="stat"><strong>{summary["topics"]}</strong><div class="muted">topics</div></div>
      <div class="stat"><strong>{summary["edges"]}</strong><div class="muted">pub/sub edges</div></div>
      <div class="stat"><strong>{summary["sampled_topics"]}</strong><div class="muted">sampled topics</div></div>
    </div>
  </header>
  <main>
    <input id="filter" placeholder="Filter by topic, node, or type">
    <section>
      <div class="section-title">Topic Relationships</div>
      <svg viewBox="0 0 {width} {svg_height}" height="{svg_height}" role="img">
        <defs>
          <marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
            <path d="M0,0 L8,4 L0,8 z" fill="#718096"></path>
          </marker>
        </defs>
        <text x="56" y="26" class="small">Publishers</text>
        <text x="568" y="26" class="small">Topics</text>
        <text x="1030" y="26" class="small">Subscribers</text>
        {''.join(svg_rows)}
      </svg>
    </section>
    <section>
      <div class="section-title">Performance Details</div>
      <table>
        <thead>
          <tr><th>Status</th><th>Topic</th><th>Publishers</th><th>Subscribers</th><th>Performance</th></tr>
        </thead>
        <tbody>
          {''.join(rows)}
        </tbody>
      </table>
    </section>
    <details>
      <summary>Raw JSON</summary>
      <pre>{html.escape(payload)}</pre>
    </details>
  </main>
  <script>
    const input = document.getElementById('filter');
    input.addEventListener('input', () => {{
      const query = input.value.trim().toLowerCase();
      document.querySelectorAll('[data-filter]').forEach((el) => {{
        el.style.display = !query || el.dataset.filter.includes(query) ? '' : 'none';
      }});
    }});
  </script>
</body>
</html>
"""


def render_svg_topic_row(
    topic: str,
    publishers: list[str],
    subscribers: list[str],
    type_text: str,
    metric: dict[str, Any],
    y: int,
    filter_text: str,
) -> str:
    row_parts = [f'<g data-filter="{html.escape(filter_text.lower(), quote=True)}">']
    row_parts.append(f'<line class="rowline" x1="36" y1="{y + 50}" x2="1284" y2="{y + 50}"/>')
    row_parts.append(render_svg_box(500, y + 12, 330, 46, topic, type_text, "topic"))
    row_parts.append(
        f'<text x="512" y="{y + 74}" class="small">{html.escape(topic_perf_label(metric))}</text>'
    )

    for index, publisher in enumerate(publishers[:3] or ["none"]):
        box_y = y + 4 + (index * 26)
        if publisher == "none":
            row_parts.append(f'<text x="58" y="{box_y + 18}" class="small">no publisher</text>')
        else:
            row_parts.append(render_svg_box(46, box_y, 310, 22, publisher, "", "box"))
            row_parts.append(
                f'<path class="edge" d="M356,{box_y + 11} C420,{box_y + 11} 430,{y + 35} 500,{y + 35}"/>'
            )
    if len(publishers) > 3:
        row_parts.append(f'<text x="58" y="{y + 90}" class="small">+{len(publishers) - 3} more</text>')

    for index, subscriber in enumerate(subscribers[:3] or ["none"]):
        box_y = y + 4 + (index * 26)
        if subscriber == "none":
            row_parts.append(f'<text x="1010" y="{box_y + 18}" class="small">no subscriber</text>')
        else:
            row_parts.append(
                f'<path class="edge" d="M830,{y + 35} C900,{y + 35} 920,{box_y + 11} 982,{box_y + 11}"/>'
            )
            row_parts.append(render_svg_box(982, box_y, 310, 22, subscriber, "", "box"))
    if len(subscribers) > 3:
        row_parts.append(
            f'<text x="1010" y="{y + 90}" class="small">+{len(subscribers) - 3} more</text>'
        )
    row_parts.append("</g>")
    return "".join(row_parts)


def render_svg_box(
    x: int,
    y: int,
    width: int,
    height: int,
    title: str,
    subtitle: str,
    css_class: str,
) -> str:
    safe_title = truncate_svg_text(title)
    safe_subtitle = truncate_svg_text(subtitle, 58)
    parts = [
        f'<rect class="{css_class}" x="{x}" y="{y}" width="{width}" height="{height}" rx="5"/>',
        f'<text x="{x + 10}" y="{y + 16}">{html.escape(safe_title)}</text>',
    ]
    if subtitle:
        parts.append(f'<text x="{x + 10}" y="{y + 34}" class="small">{html.escape(safe_subtitle)}</text>')
    return "".join(parts)


def truncate_svg_text(value: str, max_chars: int = 46) -> str:
    return value if len(value) <= max_chars else value[: max_chars - 1] + "..."


def topic_perf_label(metric: dict[str, Any]) -> str:
    if not metric:
        return "not sampled"
    pieces = [
        f"{metric.get('avg_hz', 0.0)} Hz",
        f"{metric.get('mib_per_s', 0.0)} MiB/s",
    ]
    if metric.get("p95_latency_ms") is not None:
        pieces.append(f"p95 latency {metric['p95_latency_ms']} ms")
    if metric.get("p95_period_ms") is not None:
        pieces.append(f"p95 period {metric['p95_period_ms']} ms")
    if metric.get("expected_hz") is not None:
        pieces.append(f"expected {metric['expected_hz']} Hz")
    return " | ".join(pieces)


def timestamped_path(prefix: str, suffix: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return REPORT_ROOT / f"{prefix}_{stamp}.{suffix}"


def build_payload(graph: GraphSnapshot, metrics: list[dict[str, Any]]) -> dict[str, Any]:
    generated_at = datetime.now().isoformat(timespec="seconds")
    return {
        "generated_at": generated_at,
        "graph": asdict(graph),
        "metrics": metrics,
    }


def run_monitor(args: argparse.Namespace) -> int:
    rclpy.init()
    node = Node("sop_robot_topic_monitor")
    try:
        expectations = default_robot_expectations() if args.profile == "robot" else {}
        expectations.update(parse_expectations(args.expect))
        topics = select_topics(node, args.topic, args.include_hidden, args.include_system)
        monitor = TopicMonitor(node, topics, expectations)
        monitor.start()
        print(f"Monitoring {len(monitor.metrics) - len(monitor.skipped_topics)} topics.")
        graph = spin_for_duration(
            node,
            monitor,
            duration=args.duration,
            print_period=args.period,
            include_hidden=args.include_hidden,
            include_system=args.include_system,
        )
        metrics = monitor.summaries(graph)
        if args.output_json:
            write_json(Path(args.output_json), build_payload(graph, metrics))
        if args.output_csv:
            write_csv(Path(args.output_csv), metrics)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_graph(args: argparse.Namespace) -> int:
    rclpy.init()
    node = Node("sop_robot_topic_graph")
    try:
        deadline = time.monotonic() + args.discovery_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        graph = capture_graph(node, args.include_hidden, args.include_system)
        metrics: list[dict[str, Any]] = []
        payload = build_payload(graph, metrics)
        html_path = Path(args.output_html) if args.output_html else timestamped_path("ros_topic_graph", "html")
        json_path = Path(args.output_json) if args.output_json else html_path.with_suffix(".json")
        dot_path = Path(args.output_dot) if args.output_dot else html_path.with_suffix(".dot")
        write_json(json_path, payload)
        dot_path.parent.mkdir(parents=True, exist_ok=True)
        dot_path.write_text(render_dot(graph), encoding="utf-8")
        html_path.parent.mkdir(parents=True, exist_ok=True)
        html_path.write_text(render_html(graph, metrics, payload["generated_at"]), encoding="utf-8")
        print(f"HTML: {html_path}")
        print(f"JSON: {json_path}")
        print(f"DOT:  {dot_path}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_overview(args: argparse.Namespace) -> int:
    rclpy.init()
    node = Node("sop_robot_topic_overview")
    try:
        expectations = default_robot_expectations() if args.profile == "robot" else {}
        expectations.update(parse_expectations(args.expect))
        topics = select_topics(node, args.topic, args.include_hidden, args.include_system)
        monitor = TopicMonitor(node, topics, expectations)
        monitor.start()
        print(f"Sampling {len(monitor.metrics) - len(monitor.skipped_topics)} topics for {args.duration:.1f}s.")
        graph = spin_for_duration(
            node,
            monitor,
            duration=args.duration,
            print_period=args.period if args.print else None,
            include_hidden=args.include_hidden,
            include_system=args.include_system,
        )
        metrics = monitor.summaries(graph)
        payload = build_payload(graph, metrics)
        html_path = Path(args.output_html) if args.output_html else timestamped_path("ros_topic_overview", "html")
        json_path = Path(args.output_json) if args.output_json else html_path.with_suffix(".json")
        csv_path = Path(args.output_csv) if args.output_csv else html_path.with_suffix(".csv")
        dot_path = Path(args.output_dot) if args.output_dot else html_path.with_suffix(".dot")
        write_json(json_path, payload)
        write_csv(csv_path, metrics)
        dot_path.parent.mkdir(parents=True, exist_ok=True)
        dot_path.write_text(render_dot(graph), encoding="utf-8")
        html_path.parent.mkdir(parents=True, exist_ok=True)
        html_path.write_text(render_html(graph, metrics, payload["generated_at"]), encoding="utf-8")
        print(f"HTML: {html_path}")
        print(f"JSON: {json_path}")
        print(f"CSV:  {csv_path}")
        print(f"DOT:  {dot_path}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        help="Topic to monitor. May be repeated. Defaults to all non-system topics.",
    )
    parser.add_argument(
        "--expect",
        action="append",
        default=[],
        metavar="TOPIC=HZ",
        help="Expected topic rate used for ok/slow/bad status. May be repeated.",
    )
    parser.add_argument(
        "--profile",
        choices=("none", "robot"),
        default="robot",
        help="Expectation profile. 'robot' adds SOP-Robot defaults for key topics.",
    )
    parser.add_argument("--include-hidden", action="store_true", help="Include hidden ROS topics/nodes.")
    parser.add_argument("--include-system", action="store_true", help="Include /rosout and /parameter_events.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    monitor_parser = subparsers.add_parser("monitor", help="Print a live topic performance table.")
    add_common_args(monitor_parser)
    monitor_parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means until Ctrl-C.")
    monitor_parser.add_argument("--period", type=float, default=DEFAULT_PRINT_PERIOD_SECONDS, help="Print period in seconds.")
    monitor_parser.add_argument("--output-json", default="", help="Optional JSON summary path.")
    monitor_parser.add_argument("--output-csv", default="", help="Optional CSV summary path.")

    graph_parser = subparsers.add_parser("graph", help="Generate a static topic relationship graph.")
    graph_parser.add_argument("--discovery-seconds", type=float, default=2.0, help="Seconds to wait for graph discovery.")
    graph_parser.add_argument("--include-hidden", action="store_true", help="Include hidden ROS topics/nodes.")
    graph_parser.add_argument("--include-system", action="store_true", help="Include /rosout and /parameter_events.")
    graph_parser.add_argument("--output-html", default="", help="HTML output path.")
    graph_parser.add_argument("--output-json", default="", help="JSON output path.")
    graph_parser.add_argument("--output-dot", default="", help="Graphviz DOT output path.")

    overview_parser = subparsers.add_parser("overview", help="Sample topic performance and render an HTML graph report.")
    add_common_args(overview_parser)
    overview_parser.add_argument("--duration", type=float, default=DEFAULT_SAMPLE_SECONDS, help="Seconds to sample topics.")
    overview_parser.add_argument("--period", type=float, default=DEFAULT_PRINT_PERIOD_SECONDS, help="Print period in seconds when --print is used.")
    overview_parser.add_argument("--print", action="store_true", help="Print live tables while sampling.")
    overview_parser.add_argument("--output-html", default="", help="HTML output path.")
    overview_parser.add_argument("--output-json", default="", help="JSON output path.")
    overview_parser.add_argument("--output-csv", default="", help="CSV output path.")
    overview_parser.add_argument("--output-dot", default="", help="Graphviz DOT output path.")

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "monitor":
        return run_monitor(args)
    if args.command == "graph":
        return run_graph(args)
    if args.command == "overview":
        return run_overview(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    sys.exit(main())
