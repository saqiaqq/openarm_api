"""Default gripper params when LLM / voice omits position, force, or speed."""

from __future__ import annotations

from typing import Any, Dict, Tuple

# Metres — aligned with openarm_skills/config/skills.yaml
GRIPPER_POSITION_BY_ACTION = {
    "open": 0.04,
    "close": 0.0,
    "half_close": 0.02,
    "grasp": 0.0,
}

DEFAULT_GRIPPER_FORCE = 1.0
DEFAULT_GRIPPER_SPEED = 0.5


def apply_gripper_param_defaults(params: Dict[str, Any] | None) -> Tuple[Dict[str, Any], str]:
    """
    Fill missing gripper fields. Returns (params, action).

    position: use action default when missing or <= 0
    force/speed: use defaults when missing or <= 0
    """
    p = dict(params or {})
    action = str(p.get("action", "open")).strip().lower()
    if action not in GRIPPER_POSITION_BY_ACTION:
        action = "open"

    pos = float(p.get("position", 0.0))
    if pos <= 0.0:
        pos = GRIPPER_POSITION_BY_ACTION[action]
    p["position"] = pos

    force = float(p.get("force", 0.0))
    if force <= 0.0:
        force = DEFAULT_GRIPPER_FORCE
    p["force"] = force

    speed = float(p.get("speed", 0.0))
    if speed <= 0.0:
        speed = DEFAULT_GRIPPER_SPEED
    p["speed"] = speed

    p["action"] = action
    return p, action
