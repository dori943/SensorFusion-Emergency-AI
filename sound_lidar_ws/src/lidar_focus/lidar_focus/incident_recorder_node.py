"""Keep a rolling visualization buffer and save confirmed fall incidents."""

from __future__ import annotations

import base64
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


CONFIRMED_LEVELS = {"confirmed", "critical", "emergency", "danger", "fall"}


class IncidentRecorderNode(Node):
    def __init__(self):
        super().__init__("incident_recorder_node")
        self.declare_parameter("cloud_topic", "/viz/scene_cloud")
        self.declare_parameter("meta_topic", "/viz/scene_meta")
        self.declare_parameter("pre_event_sec", 30.0)
        self.declare_parameter("post_event_sec", 10.0)
        self.declare_parameter("record_fps", 5.0)
        self.declare_parameter("incident_cooldown_sec", 60.0)
        self.declare_parameter(
            "output_dir", "~/Sensor-Fusion-Home-CCTV/data/incidents"
        )

        self.pre_event_sec = float(self.get_parameter("pre_event_sec").value)
        self.post_event_sec = float(self.get_parameter("post_event_sec").value)
        record_fps = float(self.get_parameter("record_fps").value)
        if record_fps <= 0:
            raise ValueError("record_fps must be greater than zero")
        self.record_interval = 1.0 / record_fps
        self.incident_cooldown_sec = float(
            self.get_parameter("incident_cooldown_sec").value
        )
        self.output_dir = Path(
            os.path.expanduser(self.get_parameter("output_dir").value)
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.frames = deque()
        self.latest_meta = None
        self.last_frame_time = float("-inf")
        self.last_trigger_time = float("-inf")
        self.active_incident = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._on_cloud,
            qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("meta_topic").value,
            self._on_meta,
            qos,
        )
        self.create_timer(0.2, self._finish_if_due)
        self.get_logger().info(
            f"사고 기록 준비 | 이전 {self.pre_event_sec:.0f}초 + "
            f"이후 {self.post_event_sec:.0f}초 | {self.output_dir}"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _alert_from_meta(meta):
        if meta.get("event_type") == "fall_alert":
            return meta
        return meta.get("alert") or meta.get("fall")

    @staticmethod
    def _is_confirmed(alert):
        if not isinstance(alert, dict):
            return False
        if str(alert.get("state_code", "")) == "2":
            return True
        level = str(alert.get("level", "")).lower()
        severity = str(alert.get("severity", "")).lower()
        state = str(alert.get("state", "")).lower()
        return bool({level, severity, state} & CONFIRMED_LEVELS)

    def _on_meta(self, msg):
        try:
            meta = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning("/viz/scene_meta JSON 파싱 실패")
            return
        self.latest_meta = meta

        alert = self._alert_from_meta(meta)
        now = self._now()
        cooldown_elapsed = now - self.last_trigger_time >= self.incident_cooldown_sec
        if (
            self._is_confirmed(alert)
            and self.active_incident is None
            and cooldown_elapsed
        ):
            self.last_trigger_time = now
            self.active_incident = {
                "trigger_time": now,
                "deadline": now + self.post_event_sec,
                "case_id": alert.get("case_id"),
                "track_id": alert.get("track_id"),
                "frames": list(self.frames),
            }
            self.get_logger().warning(
                f"낙상 확정: 최근 {self.pre_event_sec:.0f}초 보존, "
                f"이후 {self.post_event_sec:.0f}초 기록 시작"
            )

    @staticmethod
    def _snapshot_cloud(msg):
        return {
            "height": msg.height,
            "width": msg.width,
            "fields": [
                {
                    "name": field.name,
                    "offset": field.offset,
                    "datatype": field.datatype,
                    "count": field.count,
                }
                for field in msg.fields
            ],
            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "data": bytes(msg.data),
            "is_dense": msg.is_dense,
        }

    def _on_cloud(self, msg):
        now = self._now()
        if now - self.last_frame_time < self.record_interval:
            return
        self.last_frame_time = now
        frame = {
            "time": now,
            "cloud": self._snapshot_cloud(msg),
            "meta": self.latest_meta,
        }
        self.frames.append(frame)
        cutoff = now - self.pre_event_sec
        while self.frames and self.frames[0]["time"] < cutoff:
            self.frames.popleft()
        if self.active_incident is not None:
            self.active_incident["frames"].append(frame)
        self._finish_if_due()

    def _finish_if_due(self):
        incident = self.active_incident
        if incident is None or self._now() < incident["deadline"]:
            return
        self.active_incident = None
        threading.Thread(
            target=self._write_incident,
            args=(incident,),
            daemon=True,
        ).start()

    def _write_incident(self, incident):
        created_at = datetime.now().astimezone()
        case_suffix = (
            f"_case-{incident['case_id']}" if incident["case_id"] is not None else ""
        )
        filename = f"fall_{created_at.strftime('%Y%m%d_%H%M%S')}{case_suffix}.json"
        output_path = self.output_dir / filename
        trigger_time = incident["trigger_time"]

        with output_path.open("w", encoding="utf-8") as output:
            header = {
                "schema_version": 1,
                "created_at": created_at.isoformat(),
                "track_id": incident["track_id"],
                "pre_event_sec": self.pre_event_sec,
                "post_event_sec": self.post_event_sec,
            }
            output.write("{")
            output.write(",".join(
                f"{json.dumps(key)}:{json.dumps(value, ensure_ascii=False)}"
                for key, value in header.items()
            ))
            output.write(',"frames":[')
            for index, frame in enumerate(incident["frames"]):
                if index:
                    output.write(",")
                cloud = dict(frame["cloud"])
                cloud["data"] = base64.b64encode(cloud["data"]).decode("ascii")
                json.dump(
                    {
                        "time": frame["time"] - trigger_time,
                        "cloud": cloud,
                        "meta": frame["meta"],
                    },
                    output,
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
            output.write("]}")

        self._update_index(filename, header)
        self.get_logger().info(
            f"사고 클립 저장 완료: {output_path} "
            f"({len(incident['frames'])} frames)"
        )

    def _update_index(self, filename, header):
        index_path = self.output_dir / "index.json"
        try:
            with index_path.open("r", encoding="utf-8") as source:
                entries = json.load(source)
        except (FileNotFoundError, json.JSONDecodeError):
            entries = []
        entries.insert(0, {"file": filename, **header})
        temporary_path = index_path.with_suffix(".tmp")
        with temporary_path.open("w", encoding="utf-8") as output:
            json.dump(entries, output, ensure_ascii=False, separators=(",", ":"))
        os.replace(temporary_path, index_path)


def main(args=None):
    rclpy.init(args=args)
    node = IncidentRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
