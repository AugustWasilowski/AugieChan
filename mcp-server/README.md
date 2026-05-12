# `stackchan` MCP server

Wraps the firmware's HTTP API as [MCP](https://modelcontextprotocol.io) tools so a Claude Code (or any other MCP client) session can drive the robot.

## Tools exposed

| Tool | What it does |
| --- | --- |
| `status` | IP, MAC, RSSI, battery, current expression, servo angles, camera ready flag. |
| `set_face(expression)` | `neutral` / `happy` / `sad` / `angry` / `sleepy` / `doubt`. |
| `speak(text)` | Renders a speech bubble + animated mouth. |
| `move_servo(yaw?, pitch?)` | Absolute servo angles in degrees. |
| `home()` | Return to home position. |
| `stop_servo()` | Cancel in-flight motion. |
| `set_led(r, g, b)` | Onboard RGB LED ring. |

Optional face-tracker tools (`track_face_start` / `track_face_stop` / `track_face_status`) live in [`mcp_patch.py`](../tracker/mcp_patch.py) — append them to `stackchan_mcp.py` if you've also set up [`tracker/`](../tracker/).

## Install

```bash
cd mcp-server
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Configure

| Env var | Default | Meaning |
| --- | --- | --- |
| `STACKCHAN_BASE_URL` | `http://stackchan.local` | Firmware HTTP root. Use the device's IP if mDNS doesn't resolve on your LAN. |
| `STACKCHAN_TIMEOUT` | `10` | HTTP timeout in seconds. |

## Register with Claude Code

Add to `~/.claude.json` (or the equivalent for your client):

```jsonc
{
  "mcpServers": {
    "stackchan": {
      "command": "/absolute/path/to/AugieChan/mcp-server/.venv/bin/python",
      "args": ["/absolute/path/to/AugieChan/mcp-server/stackchan_mcp.py"],
      "env": { "STACKCHAN_BASE_URL": "http://10.0.0.134" }
    }
  }
}
```

Then restart Claude Code. Tools appear as `mcp__stackchan__status`, `mcp__stackchan__set_face`, etc.
