"""JSON Schema loader + dispatcher for the API gateway.

We pre-load every schema at startup, build a `jsonschema.Validator` per
command type, and expose a single `validate(envelope)` entry point that the
bridge node calls before forwarding work to the skill server.

The schemas double as **LLM tool definitions** -- see
`docs/llm_tool_manifest.json`, which is auto-derivable from these files.
"""

from __future__ import annotations

import json
import os
from typing import Dict, Tuple

import jsonschema
from jsonschema import RefResolver


SCHEMA_DIR = os.path.join(os.path.dirname(__file__), "schemas")

# `cmd_type` -> filename mapping for the per-command params schemas.
_PARAM_SCHEMAS = {
    "pick_place": "pick_place.schema.json",
    "pick":       "pick_place.schema.json",   # share schema; pick uses same params
    "place":      "pick_place.schema.json",
    "home":       "home.schema.json",
    "hands_up":   "home.schema.json",
    "stop":       "stop.schema.json",
    "gripper":    "gripper.schema.json",
    "get_status": "stop.schema.json",          # empty params
}


def _load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


class EnvelopeValidator:
    """Validates the full JSON envelope (top + params)."""

    def __init__(self, schema_dir: str = SCHEMA_DIR):
        self._schema_dir = schema_dir
        self._envelope = _load_json(os.path.join(schema_dir, "envelope.schema.json"))
        self._store: Dict[str, dict] = {}
        for fn in os.listdir(schema_dir):
            if fn.endswith(".schema.json"):
                s = _load_json(os.path.join(schema_dir, fn))
                self._store[s.get("$id", f"openarm_api/{fn}")] = s
        self._params_validators: Dict[str, jsonschema.Validator] = {}
        resolver = RefResolver(base_uri="openarm_api/", referrer={}, store=self._store)
        for cmd, fn in _PARAM_SCHEMAS.items():
            schema = _load_json(os.path.join(schema_dir, fn))
            self._params_validators[cmd] = jsonschema.Draft7Validator(
                schema, resolver=resolver)
        self._envelope_validator = jsonschema.Draft7Validator(
            self._envelope, resolver=resolver)

    def validate(self, envelope: dict) -> Tuple[bool, str]:
        """Returns (ok, error_message). Empty error_message when ok."""
        errs = sorted(self._envelope_validator.iter_errors(envelope), key=lambda e: e.path)
        if errs:
            return False, f"envelope: {errs[0].message}"
        cmd = envelope.get("cmd_type", "")
        v = self._params_validators.get(cmd)
        if not v:
            return False, f"unsupported cmd_type '{cmd}'"
        params = envelope.get("params", {}) or {}
        perrs = sorted(v.iter_errors(params), key=lambda e: e.path)
        if perrs:
            return False, f"params.{cmd}: {perrs[0].message}"
        return True, ""

    @property
    def schemas(self) -> Dict[str, dict]:
        return self._store
