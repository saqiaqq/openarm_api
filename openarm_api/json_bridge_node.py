"""json_bridge_node -- the single JSON entry point for the upper computer / LLM.

Architecture
------------
                +---------------------+
   JSON  ---->  |  json_bridge_node   |  --action--> /openarm/pick_place
   (srv  /      |  (this file)        |  --srv-----> /openarm/stop
    HTTP /      |                     |  --srv-----> /openarm/goto_home
    WS)         |                     |  --srv-----> /openarm/gripper
                +----------+----------+
                           |
                           v   `/openarm/skill/event` (std_msgs/String JSON)

Public surface
--------------
- ROS2 service:      /openarm/command   [openarm_skills/srv/StringCommand]
- ROS2 topic (out):  /openarm/skill/event   (std_msgs/String, JSON payload)
- Optional HTTP:     POST /command, GET /status, WS /events
                     (enabled with parameter `enable_http: true`)

Wire format -- request envelope
-------------------------------
    {
      "cmd_id": "<uuid>",                            # required
      "cmd_type": "pick_place|pick|place|home|stop|gripper|get_status",
      "arm": "left|right|both",
      "pose_source": "camera|upper_computer",
      "params": { ... per-cmd, see schemas/ ... },
      "timeout_s": 30
    }

Wire format -- response envelope
--------------------------------
    {
      "cmd_id": "<uuid>",
      "success": true,
      "result_code": 0,                              # see schemas/error_codes.json
      "result_name": "OK",
      "status": "done|error|stopped",
      "phase": "place.retreat",
      "message": "...",
      "perceived": {
         "grasp_pose": { "xyz":[...], "rpy":[...], "frame":"base_link" },
         "place_pose": { "xyz":[...], "rpy":[...], "frame":"base_link" }
      },
      "timestamp": "2026-04-30T11:00:00Z"
    }

Asynchronous status events use the same outer shape minus `result_code`,
plus `progress` (0~1) and the streaming `status` / `phase` fields.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from collections import OrderedDict
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion

from openarm_skills.action import PickPlace
from openarm_skills.srv import (
    Stop as StopSrv,
    GotoHome as GotoHomeSrv,
    Gripper as GripperSrv,
    StringCommand,
)

from . import error_codes as ec
from .validators import EnvelopeValidator


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _quat_to_rpy(q: Quaternion) -> Tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _pose_dict_to_msg(d: Optional[dict]) -> Pose:
    p = Pose()
    if not d:
        return p
    x, y, z = d.get("xyz", [0.0, 0.0, 0.0])
    r, pi, ya = d.get("rpy", [0.0, 0.0, 0.0])
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = _rpy_to_quat(float(r), float(pi), float(ya))
    return p


def _pose_msg_to_dict(p: Pose, frame: str = "base_link") -> dict:
    rpy = _quat_to_rpy(p.orientation)
    return {
        "xyz": [p.position.x, p.position.y, p.position.z],
        "rpy": list(rpy),
        "frame": frame,
    }


def _envelope(cmd_id: str, **fields) -> dict:
    out = {"cmd_id": cmd_id, "timestamp": _now_iso()}
    out.update(fields)
    if "result_code" in fields:
        out["result_name"] = ec.name_of(int(fields["result_code"]))
    return out


# ---------------------------------------------------------------------------
# LRU cache for cmd_id idempotency
# ---------------------------------------------------------------------------
class _LRU(OrderedDict):
    def __init__(self, capacity: int):
        super().__init__()
        self._cap = capacity

    def put(self, k, v):
        if k in self:
            self.move_to_end(k)
        self[k] = v
        if len(self) > self._cap:
            self.popitem(last=False)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class JsonBridgeNode(Node):
    """Single ROS2 entry point that translates JSON to ROS2 actions/services."""

    def __init__(self):
        super().__init__("openarm_api")

        # ---- parameters ----------------------------------------------------
        self.declare_parameter("enable_http", False)
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("dedup_cache_size", 256)
        self.declare_parameter("default_timeout_s", 60.0)

        cache_size = int(self.get_parameter("dedup_cache_size").value)
        self._cache: _LRU = _LRU(cache_size)
        self._cache_lock = threading.Lock()

        self._validator = EnvelopeValidator()

        # ---- ROS2 surface --------------------------------------------------
        self._event_pub = self.create_publisher(String, "/openarm/skill/event", 32)
        self._command_srv = self.create_service(
            StringCommand, "/openarm/command", self._on_command_srv)

        self._pick_place_client = ActionClient(self, PickPlace, "/openarm/pick_place")
        self._stop_client = self.create_client(StopSrv, "/openarm/stop")
        self._home_client = self.create_client(GotoHomeSrv, "/openarm/goto_home")
        self._gripper_client = self.create_client(GripperSrv, "/openarm/gripper")

        # ---- websocket subscribers (for HTTP gateway) ---------------------
        self._ws_subs: list[Callable[[str], None]] = []
        self._ws_lock = threading.Lock()

        # ---- optional HTTP gateway ----------------------------------------
        if bool(self.get_parameter("enable_http").value):
            self._start_http_gateway()

        self.get_logger().info(
            "openarm_api JSON bridge ready. service=/openarm/command, "
            "event topic=/openarm/skill/event")

    # ======================================================================
    # ROS2 service handler
    # ======================================================================
    def _on_command_srv(self, req: StringCommand.Request,
                        resp: StringCommand.Response) -> StringCommand.Response:
        resp.response_json = self.handle_json(req.request_json)
        return resp

    # ======================================================================
    # Public dispatch (also used by the HTTP gateway)
    # ======================================================================
    def handle_json(self, raw: str) -> str:
        try:
            envelope = json.loads(raw) if raw else {}
        except Exception as e:
            return json.dumps(_envelope(
                "", success=False, result_code=ec.BAD_REQUEST,
                status="error", message=f"invalid JSON: {e}"))

        cmd_id = envelope.get("cmd_id") or str(uuid.uuid4())

        # idempotency cache check
        with self._cache_lock:
            if cmd_id in self._cache:
                return self._cache[cmd_id]

        ok, err = self._validator.validate(envelope)
        if not ok:
            out = json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.BAD_REQUEST,
                status="error", message=err))
            self._remember(cmd_id, out)
            return out

        cmd = envelope["cmd_type"]
        try:
            if cmd in ("pick_place", "pick", "place"):
                out = self._do_pick_place(envelope)
            elif cmd == "home":
                out = self._do_home(envelope)
            elif cmd == "stop":
                out = self._do_stop(envelope)
            elif cmd == "gripper":
                out = self._do_gripper(envelope)
            elif cmd == "get_status":
                out = json.dumps(_envelope(
                    cmd_id, success=True, result_code=ec.OK,
                    status="idle", message="bridge alive"))
            else:
                out = json.dumps(_envelope(
                    cmd_id, success=False, result_code=ec.UNSUPPORTED_CMD,
                    status="error", message=f"unknown cmd_type '{cmd}'"))
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f"unhandled exception: {e!r}")
            out = json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.INTERNAL_ERROR,
                status="error", message=f"internal error: {e}"))

        self._remember(cmd_id, out)
        return out

    def _remember(self, cmd_id: str, out: str) -> None:
        if not cmd_id:
            return
        with self._cache_lock:
            self._cache.put(cmd_id, out)

    # ======================================================================
    # cmd_type implementations
    # ======================================================================
    def _do_pick_place(self, env: dict) -> str:
        cmd_id = env["cmd_id"]
        params = env.get("params", {}) or {}

        if not self._pick_place_client.wait_for_server(timeout_sec=2.0):
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.INTERNAL_ERROR,
                status="error", message="skill server action /openarm/pick_place unavailable"))

        goal = PickPlace.Goal()
        goal.cmd_id = cmd_id
        goal.arm = env.get("arm", "right")
        goal.pose_source = env.get("pose_source", "upper_computer")
        goal.target_name = params.get("target_name", "")
        goal.target_index = int(params.get("target_index", 0))
        goal.grasp_pose = _pose_dict_to_msg(params.get("grasp_pose"))
        goal.place_pose = _pose_dict_to_msg(params.get("place_pose"))
        goal.approach_offset_m = float(params.get("approach_offset_m", 0.05))
        goal.retreat_offset_m  = float(params.get("retreat_offset_m",  0.05))
        goal.speed_scale       = float(params.get("speed_scale",       0.10))
        goal.target_radius     = float(params.get("target_radius",     0.0))
        goal.gripper_force     = float(params.get("gripper_force",     0.0))
        goal.gripper_speed     = float(params.get("gripper_speed",     0.0))
        goal.timeout_s         = float(env.get("timeout_s",
                                              self.get_parameter("default_timeout_s").value))

        # Action call with feedback rebroadcast.
        send_future = self._pick_place_client.send_goal_async(
            goal, feedback_callback=lambda fb: self._rebroadcast_feedback(cmd_id, fb))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.BAD_REQUEST,
                status="error", message="goal rejected by skill server"))

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=goal.timeout_s + 30.0)
        wrap = result_future.result()
        if wrap is None:
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.ACTION_TIMEOUT,
                status="error", message="action result timeout"))
        r = wrap.result

        out: dict = _envelope(
            cmd_id,
            success=bool(r.success),
            result_code=int(r.result_code),
            status=r.status or ("done" if r.success else "error"),
            message=r.message,
        )
        # surface camera-perceived poses so the upper computer can verify them
        if env.get("pose_source") == "camera":
            out["perceived"] = {
                "grasp_pose": _pose_msg_to_dict(r.perceived_grasp_pose),
                "place_pose": _pose_msg_to_dict(r.perceived_place_pose),
            }
        self._publish_event(out)
        return json.dumps(out)

    def _do_home(self, env: dict) -> str:
        cmd_id = env["cmd_id"]
        if not self._home_client.wait_for_service(timeout_sec=2.0):
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.INTERNAL_ERROR,
                status="error", message="goto_home service unavailable"))
        req = GotoHomeSrv.Request()
        req.arm = env.get("arm", "both")
        req.speed_scale = float((env.get("params") or {}).get("speed_scale", 0.30))
        fut = self._home_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        r = fut.result()
        return json.dumps(_envelope(
            cmd_id,
            success=bool(r and r.success),
            result_code=int(r.result_code) if r else ec.ACTION_TIMEOUT,
            status="done" if (r and r.success) else "error",
            message=(r.message if r else "home call timeout"),
        ))

    def _do_stop(self, env: dict) -> str:
        cmd_id = env["cmd_id"]
        if not self._stop_client.wait_for_service(timeout_sec=1.0):
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.INTERNAL_ERROR,
                status="error", message="stop service unavailable"))
        req = StopSrv.Request()
        req.cmd_id = cmd_id
        fut = self._stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        r = fut.result()
        return json.dumps(_envelope(
            cmd_id,
            success=bool(r and r.success),
            result_code=ec.STOPPED_BY_USER if (r and r.success) else ec.INTERNAL_ERROR,
            status="stopped" if (r and r.success) else "error",
            message=(r.message if r else "stop call timeout"),
        ))

    def _do_gripper(self, env: dict) -> str:
        cmd_id = env["cmd_id"]
        params = env.get("params", {}) or {}
        if not self._gripper_client.wait_for_service(timeout_sec=2.0):
            return json.dumps(_envelope(
                cmd_id, success=False, result_code=ec.INTERNAL_ERROR,
                status="error", message="gripper service unavailable"))
        req = GripperSrv.Request()
        req.arm = env.get("arm", "right")
        req.action = params.get("action", "open")
        req.position = float(params.get("position", 0.0))
        req.force = float(params.get("force", 0.0))
        req.speed = float(params.get("speed", 0.0))
        fut = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        return json.dumps(_envelope(
            cmd_id,
            success=bool(r and r.success),
            result_code=int(r.result_code) if r else ec.ACTION_TIMEOUT,
            status="done" if (r and r.success) else "error",
            message=(r.message if r else "gripper call timeout"),
        ))

    # ======================================================================
    # Event broadcasting (action feedback -> JSON topic + WS)
    # ======================================================================
    def _rebroadcast_feedback(self, cmd_id: str, fb_msg) -> None:
        fb = fb_msg.feedback
        evt = _envelope(
            cmd_id,
            status=fb.status,
            phase=fb.phase,
            progress=float(fb.progress),
            message=fb.message,
        )
        self._publish_event(evt)

    def _publish_event(self, evt: dict) -> None:
        msg = String()
        msg.data = json.dumps(evt)
        self._event_pub.publish(msg)
        with self._ws_lock:
            for cb in list(self._ws_subs):
                try:
                    cb(msg.data)
                except Exception:
                    pass

    # ======================================================================
    # Optional HTTP / WebSocket gateway (FastAPI + uvicorn)
    # ======================================================================
    def _start_http_gateway(self) -> None:
        try:
            from .http_gateway import run_http_gateway
        except ImportError as e:
            self.get_logger().error(
                f"enable_http=true but FastAPI/uvicorn not installed: {e}")
            return
        host = self.get_parameter("http_host").value
        port = int(self.get_parameter("http_port").value)
        threading.Thread(
            target=run_http_gateway,
            kwargs=dict(node=self, host=host, port=port),
            daemon=True,
        ).start()
        self.get_logger().info(f"HTTP gateway listening on {host}:{port}")

    # API used by http_gateway.py
    def add_ws_subscriber(self, callback: Callable[[str], None]) -> None:
        with self._ws_lock:
            self._ws_subs.append(callback)

    def remove_ws_subscriber(self, callback: Callable[[str], None]) -> None:
        with self._ws_lock:
            if callback in self._ws_subs:
                self._ws_subs.remove(callback)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = JsonBridgeNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
