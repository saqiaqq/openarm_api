# OpenArm JSON API reference

This document is the canonical reference for upper-computer / LLM clients.
The skill server (`openarm_skills`) implements PRD tasks 1-5; this gateway
wraps it in a single JSON entry point so the caller never has to deal with
ROS2 actions or services directly.

---

## Endpoints

| Channel    | Path                          | Type                                       |
|------------|-------------------------------|--------------------------------------------|
| ROS2 srv   | `/openarm/command`            | `openarm_skills/srv/StringCommand`         |
| ROS2 topic | `/openarm/skill/event`        | `std_msgs/String` (JSON payload)           |
| HTTP       | `POST http://<host>:<port>/command` | body: envelope JSON, returns response JSON |
| HTTP       | `GET  http://<host>:<port>/status`  | liveness probe                       |
| WebSocket  | `ws://<host>:<port>/events`   | server pushes every event JSON             |

Enable the HTTP/WS surface with `enable_http:=true` on the launch file or in
`config/api_config.yaml`.

---

## Request envelope

```jsonc
{
  "cmd_id":     "<uuid>",                       // required, idempotent key
  "cmd_type":   "pick_place|pick|place|home|stop|gripper|get_status",
  "arm":        "left|right|both",
  "pose_source":"camera|upper_computer",        // pick_place only
  "params":     { /* per-cmd, see below */ },
  "timeout_s":  30
}
```

### `cmd_type = pick_place`

```jsonc
{
  "cmd_id": "550e8400-e29b-41d4-a716-446655440000",
  "cmd_type": "pick_place",
  "arm": "right",
  "pose_source": "upper_computer",
  "params": {
    "grasp_pose":  { "xyz": [0.42, 0.10, 0.20], "rpy": [0, 1.5708, 0], "frame": "base_link" },
    "place_pose":  { "xyz": [0.30,-0.20, 0.20], "rpy": [0, 1.5708, 0], "frame": "base_link" },
    "approach_offset_m": 0.05,
    "retreat_offset_m":  0.05,
    "speed_scale":       0.10,
    "target_radius":     0.04,
    "gripper_force":     8.0,
    "gripper_speed":     0.3
  },
  "timeout_s": 30
}
```

Camera mode (`pose_source = "camera"`):

```jsonc
{
  "cmd_id": "...",
  "cmd_type": "pick_place",
  "arm": "right",
  "pose_source": "camera",
  "params": { "target_name": "cup", "target_index": 0 }
}
```

### `cmd_type = home`

```jsonc
{ "cmd_id":"...", "cmd_type":"home", "arm":"both",
  "params": { "speed_scale": 0.3 } }
```

### `cmd_type = gripper`

```jsonc
{ "cmd_id":"...", "cmd_type":"gripper", "arm":"right",
  "params": { "action": "grasp", "force": 8.0, "speed": 0.5 } } // speed 越小越慢
```

### `cmd_type = stop`

```jsonc
{ "cmd_id":"...", "cmd_type":"stop" }
```

### `cmd_type = get_status`

Liveness check that travels through the same pipe as a real command.

---

## Response envelope (synchronous final result)

```jsonc
{
  "cmd_id":      "550e8400-...",
  "success":     true,
  "result_code": 0,
  "result_name": "OK",
  "status":      "done",                      // "done" | "error" | "stopped"
  "phase":       "place.retreat",             // optional fine-grained tag
  "message":     "pick_and_place done",
  "perceived":   {                            // only when pose_source == camera
    "grasp_pose": { "xyz":[...], "rpy":[...], "frame":"base_link" },
    "place_pose": { "xyz":[...], "rpy":[...], "frame":"base_link" }
  },
  "timestamp":   "2026-04-30T11:00:00Z"
}
```

## Async event envelope (WebSocket / `/openarm/skill/event` topic)

```jsonc
{
  "cmd_id":   "550e8400-...",
  "status":   "perceiving|grasping|transporting|placing|done|error|stopped",
  "phase":    "pick.descend",
  "progress": 0.55,
  "message":  "descending to grasp",
  "timestamp":"2026-04-30T11:00:01Z"
}
```

The final synchronous response is also republished as an event, so a UI
listening on the WebSocket sees the full lifecycle in chronological order.

---

## Error codes

The full table lives in [`schemas/error_codes.json`](../openarm_api/schemas/error_codes.json).

| code | name                  | category    | when it happens / how to react |
|-----:|-----------------------|-------------|--------------------------------|
|    0 | `OK`                  | ok          | success |
| 1001 | `PLAN_FAILED`         | motion      | MoveIt plan/Cartesian fraction below threshold (after retries). Try a different `grasp_pose`, lower `speed_scale`, or move obstructions. |
| 1002 | `ACTION_TIMEOUT`      | motion      | A step exceeded `step_timeout_s`. Inspect controllers / hardware. |
| 1003 | `GRIP_NOT_HELD`       | motion      | Finger position indicates an empty grasp after the auto-retry. Check object position / gripper calibration. |
| 1004 | `EXECUTE_FAILED`      | motion      | Controller refused the trajectory. Look at `ros2_control` logs. |
| 2001 | `CAMERA_NO_CLOUD`     | perception  | Perception service unreachable / no point cloud. Check Orbbec topic. |
| 2002 | `CAMERA_NO_TARGET`    | perception  | Object not detected. Adjust scene / change `target_name`. |
| 2003 | `CAMERA_OUT_OF_RANGE` | perception  | Pose outside the workspace bounds. Reposition the object or relax the bounds in `skills.yaml`. |
| 2004 | `PERCEPTION_TIMEOUT`  | perception  | Perception call exceeded `perception_timeout_s`. |
| 3001 | `BAD_REQUEST`         | protocol    | JSON Schema validation failed. The `message` field always carries the offending field. **Do not retry without modifying the input.** |
| 3002 | `UNSUPPORTED_CMD`     | protocol    | Unknown `cmd_type`. |
| 9001 | `STOPPED_BY_USER`     | lifecycle   | Aborted via `/openarm/stop` or action cancel. |
| 9002 | `INTERNAL_ERROR`      | lifecycle   | Unhandled exception. File a bug with the message. |

LLM-friendly retry hints:

- `1xxx` (motion): safe to retry once with the same arguments.
- `2001` / `2004` / `1002`: retry with backoff, then surface to the user.
- `2002` / `2003`: do **not** retry blindly; ask the user / re-plan.
- `3xxx`: never retry without modifying the request.
- `9001`: the user pressed stop; do not auto-resume.

---

## Calling samples

### From a ROS2 client (Python)

```python
import json, uuid, rclpy
from rclpy.node import Node
from openarm_skills.srv import StringCommand

rclpy.init()
node = Node("demo")
cli = node.create_client(StringCommand, "/openarm/command")
cli.wait_for_service()
req = StringCommand.Request()
req.request_json = json.dumps({
    "cmd_id":     str(uuid.uuid4()),
    "cmd_type":   "pick_place",
    "arm":        "right",
    "pose_source":"upper_computer",
    "params": {
        "grasp_pose": {"xyz":[0.42,0.10,0.20], "rpy":[0,1.5708,0]},
        "place_pose": {"xyz":[0.30,-0.20,0.20],"rpy":[0,1.5708,0]},
        "speed_scale": 0.10,
    },
})
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut)
print(json.loads(fut.result().response_json))
```

### From a non-ROS2 client (HTTP)

```bash
curl -X POST http://robot.local:8080/command \
     -H 'content-type: application/json' \
     -d '{"cmd_id":"abc","cmd_type":"home","arm":"both","params":{"speed_scale":0.3}}'
```

### Listening to live progress (WebSocket)

```python
import websocket, json
ws = websocket.WebSocket()
ws.connect("ws://robot.local:8080/events")
while True:
    print(json.loads(ws.recv()))
```
