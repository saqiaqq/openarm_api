# openarm_api

JSON gateway for the OpenArm skill layer. Provides a **single, schema-
validated entry point** that an upper computer (PC, PLC, or LLM agent) can
talk to without any ROS2 knowledge.

## Surfaces

| Channel        | Endpoint                                  |
|----------------|-------------------------------------------|
| ROS2 service   | `/openarm/command`                        |
| ROS2 topic out | `/openarm/skill/event` (JSON in `data`)   |
| HTTP (opt.)    | `POST /command`, `GET /status`            |
| WebSocket (opt)| `/events` (server-push of every event)    |

## Install

```bash
# from the workspace root
pip install jsonschema
# only if you want enable_http=true
pip install fastapi uvicorn websockets
colcon build --packages-select openarm_skills openarm_perception openarm_api
source install/setup.bash
```

## Run

```bash
# 1. MoveIt + controllers
ros2 launch openarm_bimanual_moveit_config demo.launch.py

# 2. (optional) perception stub for pose_source=camera
ros2 launch openarm_perception perception.launch.py

# 3. skill server
ros2 launch openarm_skills skills.launch.py

# 4. JSON gateway (ROS2 service only)
ros2 launch openarm_api api_gateway.launch.py

# 4. JSON gateway with HTTP/WS enabled
ros2 launch openarm_api api_gateway.launch.py enable_http:=true
```

## Quick test

```bash
ros2 service call /openarm/command openarm_skills/srv/StringCommand "{request_json: \
'{\"cmd_id\":\"t1\",\"cmd_type\":\"pick_place\",\"arm\":\"right\",\"pose_source\":\"upper_computer\",\
\"params\":{\"grasp_pose\":{\"xyz\":[0.42,0.10,0.20],\"rpy\":[0,1.5708,0]},\
\"place_pose\":{\"xyz\":[0.30,-0.20,0.20],\"rpy\":[0,1.5708,0]},\
\"speed_scale\":0.10}}'}"
```

## Documentation

- [`docs/api_reference.md`](docs/api_reference.md) -- full request/response
  spec, error code semantics, examples.
- [`docs/llm_tool_manifest.json`](docs/llm_tool_manifest.json) -- drop-in
  function-calling manifest for OpenAI / Anthropic-style LLM agents.
- [`openarm_api/schemas/`](openarm_api/schemas/) -- JSON Schemas used both
  for runtime validation and as the LLM tool definitions.

## Error codes

All `result_code` values are documented in detail in
[`docs/api_reference.md#error-codes`](docs/api_reference.md#error-codes) and
the canonical machine-readable list is
[`openarm_api/schemas/error_codes.json`](openarm_api/schemas/error_codes.json).
A short summary:

| range | category   | suggested LLM behaviour                  |
|-------|------------|------------------------------------------|
| 0     | success    | proceed                                  |
| 1xxx  | motion     | safe to retry once                       |
| 2xxx  | perception | retry only when transient (`2001/2004`)  |
| 3xxx  | protocol   | **never** retry without changing input   |
| 9xxx  | lifecycle  | surface to user; do not auto-resume      |
