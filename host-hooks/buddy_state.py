#!/usr/bin/env python3
"""Claude Code hook that pushes session state to a StackChan running AugieChan.

Wire this in `~/.claude/settings.json` so it fires on the lifecycle events that
shape the buddy state machine:

    {
      "hooks": {
        "SessionStart":      [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py session_start"}]}],
        "Stop":              [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py stop"}]}],
        "PreToolUse":        [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py pre_tool"}]}],
        "Notification":      [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py notification"}]}],
        "PermissionRequest": [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py permission_request", "timeout": 35}]}]
      }
    }

The `permission_request` mode is the only one that blocks: it lights the
StackChan in `attention` state and short-polls `/pending` for up to
BUDDY_PERMISSION_TIMEOUT seconds (default 30). If you swipe forward on the
faceplate it returns `{"hookSpecificOutput":{"permissionDecision":"allow"}}` to
Claude Code; swipe backward returns `"deny"`. No gesture within the window =
no JSON output, and Claude Code falls back to its usual permission UI.

Each hook receives a JSON event on stdin; we map it to a buddy state and POST to
`/state`. Failures are silenced — the StackChan being offline must never block a
session.

Set STACKCHAN_BASE (default http://10.0.0.134) to point at your device.
"""
from __future__ import annotations

import json
import os
import secrets
import sys
import time
import urllib.error
import urllib.request

BASE = os.environ.get("STACKCHAN_BASE", "http://10.0.0.134").rstrip("/")
TIMEOUT = float(os.environ.get("STACKCHAN_TIMEOUT", "2"))
PERMISSION_TIMEOUT = float(os.environ.get("BUDDY_PERMISSION_TIMEOUT", "30"))
PERMISSION_POLL_INTERVAL = float(os.environ.get("BUDDY_PERMISSION_POLL_INTERVAL", "0.5"))


def post_state(state: str, prompt_id: str | None = None) -> dict | None:
    body: dict[str, str] = {"state": state}
    if prompt_id:
        body["prompt_id"] = prompt_id
    return _post_json("/state", body)


def _post_json(path: str, body: dict) -> dict | None:
    data = json.dumps(body).encode()
    req = urllib.request.Request(
        f"{BASE}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT) as resp:
            return json.loads(resp.read() or b"{}")
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError):
        return None


def _get_json(path: str) -> dict | None:
    try:
        with urllib.request.urlopen(f"{BASE}{path}", timeout=TIMEOUT) as resp:
            return json.loads(resp.read() or b"{}")
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError):
        return None


def handle_permission_request(payload: dict) -> int:
    """Display the prompt on the buddy and wait for a swipe.

    Emits a Claude-Code hook decision JSON if the user decides within the
    timeout. Otherwise emits nothing — Claude Code falls back to its usual
    permission UI.
    """
    prompt_id = f"req_{secrets.token_hex(4)}"
    if post_state("attention", prompt_id) is None:
        # Device unreachable — let Claude Code prompt normally.
        return 0
    deadline = time.monotonic() + PERMISSION_TIMEOUT
    decision = ""
    while time.monotonic() < deadline:
        body = _get_json("/pending")
        if body is None:
            break
        if body.get("decision"):
            decision = body["decision"]
            break
        if not body.get("pending"):
            # The device cleared the prompt without a decision (e.g. another
            # /state call came in). Bail out and let Claude Code prompt.
            break
        time.sleep(PERMISSION_POLL_INTERVAL)
    if decision == "once":
        out = {"hookSpecificOutput": {
            "hookEventName": "PermissionRequest",
            "permissionDecision": "allow",
            "permissionDecisionReason": "approved via StackChan faceplate swipe",
        }}
    elif decision == "deny":
        out = {"hookSpecificOutput": {
            "hookEventName": "PermissionRequest",
            "permissionDecision": "deny",
            "permissionDecisionReason": "denied via StackChan faceplate swipe",
        }}
    else:
        # No decision; emit nothing so Claude Code prompts normally.
        post_state("busy")
        return 0
    print(json.dumps(out))
    return 0


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        return 0
    event = argv[1]

    try:
        payload = json.loads(sys.stdin.read() or "{}")
    except json.JSONDecodeError:
        payload = {}

    if event == "session_start":
        post_state("busy")
    elif event == "stop":
        post_state("idle")
    elif event == "pre_tool":
        post_state("busy")
    elif event == "notification":
        msg = (payload.get("message") or "").lower()
        if "permission" in msg or "approve" in msg:
            post_state("attention", prompt_id=payload.get("session_id"))
        else:
            post_state("busy")
    elif event == "permission_request":
        return handle_permission_request(payload)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
