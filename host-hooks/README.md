# Host-side hooks

Scripts that hook Claude Code (or other client lifecycles) up to the StackChan's
buddy-mode endpoints.

## `buddy_state.py`

Pushes a buddy state to the firmware's `POST /state` endpoint based on Claude Code
lifecycle events. Inspired by [anthropics/claude-desktop-buddy], but goes over
WiFi/HTTP instead of BLE — no pairing needed.

State mapping:

| Event                | State       | What the StackChan does                              |
| -------------------- | ----------- | ---------------------------------------------------- |
| `SessionStart`       | `busy`      | Neutral face, dim blue ring                          |
| `PreToolUse`         | `busy`      | (same — keeps the buddy "alive" during long tool runs) |
| `Notification`       | `attention` | Doubt face, yellow ring, chase strip — if msg mentions a permission |
| `Stop`               | `idle`      | Neutral face, all LEDs off                           |
| `PermissionRequest`  | `attention` | Lights up + waits up to 30 s for a swipe. Forward swipe → emits `{"hookSpecificOutput":{"permissionDecision":"allow"}}`, backward swipe → `"deny"`. No swipe → no JSON (Claude Code prompts normally). |

Install:

```bash
chmod +x /path/to/host-hooks/buddy_state.py
```

Then add the hook block (see the docstring at the top of the script) to
`~/.claude/settings.json` and restart Claude Code.

Override the device URL with `STACKCHAN_BASE=http://10.0.0.x` in the hook
command if mDNS isn't resolving or the device IP changed.

Permission-flow tuning (env vars):
- `BUDDY_PERMISSION_TIMEOUT` — seconds to wait for a swipe (default 30).
- `BUDDY_PERMISSION_POLL_INTERVAL` — seconds between `/pending` polls (default 0.5).

Failures are swallowed — the StackChan being offline never blocks a session.

[anthropics/claude-desktop-buddy]: https://github.com/anthropics/claude-desktop-buddy
