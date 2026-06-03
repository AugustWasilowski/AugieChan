#!/usr/bin/env python3
"""Claude Code hook that pushes session state to a StackChan running AugieChan.

Wire this in `~/.claude/settings.json` so it fires on the lifecycle events that
shape the buddy state machine:

    {
      "hooks": {
        "SessionStart":  [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py session_start"}]}],
        "Stop":          [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py stop"}]}],
        "PreToolUse":    [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py pre_tool"}]}],
        "Notification":  [{"hooks": [{"type": "command", "command": "/path/to/buddy_state.py notification"}]}]
      }
    }

Each hook receives a JSON event on stdin; we map it to a buddy state and POST to
`/state`. Failures are silenced — the StackChan being offline must never block a
session.

Set STACKCHAN_BASE (default http://10.0.0.134) to point at your device.
"""
from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request

BASE = os.environ.get("STACKCHAN_BASE", "http://10.0.0.134").rstrip("/")
TIMEOUT = float(os.environ.get("STACKCHAN_TIMEOUT", "2"))


def post_state(state: str, prompt_id: str | None = None) -> None:
    body = {"state": state}
    if prompt_id:
        body["prompt_id"] = prompt_id
    data = json.dumps(body).encode()
    req = urllib.request.Request(
        f"{BASE}/state",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT):
            pass
    except (urllib.error.URLError, TimeoutError, OSError):
        pass


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
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
