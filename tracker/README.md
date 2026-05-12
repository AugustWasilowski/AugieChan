# StackChan face-following tracker

Pulls MJPEG from the StackChan's camera (`http://<device>:81/stream`), finds the largest face per frame using OpenCV YuNet, and POSTs servo deltas to `http://<device>/servo` to keep the face centered. Bundled with the upstream YuNet model so there's nothing to download.

## Install

```bash
cd tracker
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run

Foreground (for testing/tuning):

```bash
STACKCHAN_BASE=http://<device-ip> .venv/bin/python tracker.py
# In another shell:
curl -X POST http://127.0.0.1:5051/track -d '{"on":true}'
curl http://127.0.0.1:5051/metrics
```

As a socket-activated systemd service — copy the units, edit paths, then enable:

```bash
sudo cp systemd/stackchan-tracker.socket  /etc/systemd/system/
sudo cp systemd/stackchan-tracker.service /etc/systemd/system/
# Edit the .service file so User=, WorkingDirectory=, and ExecStart= point at
# wherever you cloned this repo. STACKCHAN_BASE= sets the device URL.
sudoedit /etc/systemd/system/stackchan-tracker.service
sudo systemctl daemon-reload
sudo systemctl enable --now stackchan-tracker.socket
journalctl -u stackchan-tracker -f
```

Socket activation means the service is dormant until something hits `:5051`, then exits cleanly when you `POST /track {"on": false}` — no idle CPU when nobody's looking at the robot.

## Tuning

Environment variables (override defaults):

| Var | Default | Meaning |
| --- | --- | --- |
| `STACKCHAN_BASE` | `http://stackchan.local` | Device base URL. |
| `GAIN_YAW` | `0.05` | Degrees of yaw per pixel of horizontal error. |
| `GAIN_PITCH` | `0.04` | Same for pitch. |
| `DEADZONE` | `20` | Pixels of error tolerated before commanding (avoids jitter). |
| `MAX_STEP` | `8` | Max degrees per single command (anti-overshoot). |
| `COMMAND_HZ` | `8` | Control-loop rate ceiling. |
| `SERVO_SPEED` | `300` | Speed param sent to firmware (lower = faster). |
| `INVERT_X` / `INVERT_Y` | `0` | Set to `1` if servo moves the wrong way. |
| `TRACKER_PORT` | `5051` | HTTP control port (ignored under socket activation; systemd passes fd 3). |

`tracker.py` reads `LISTEN_FDS` on startup; if systemd is socket-activating it, it grabs the inherited listening socket instead of opening its own.

## Toggling from the MCP server

The companion [`mcp-server/`](../mcp-server/) exposes the firmware's HTTP API as MCP tools. Append [`mcp_patch.py`](mcp_patch.py) to `mcp-server/stackchan_mcp.py` to add three more tools — `track_face_start`, `track_face_stop`, `track_face_status` — that POST to this tracker's `/track` and `/metrics` endpoints.

## Files

- `tracker.py` — main service.
- `mcp_patch.py` — three additional MCP tools to glue this tracker into the MCP server.
- `requirements.txt` — `opencv-python-headless`, `numpy`, `requests`.
- `face_detection_yunet_2023mar.onnx` — the YuNet model from [opencv/opencv_zoo](https://github.com/opencv/opencv_zoo) (MIT).
- `systemd/stackchan-tracker.{socket,service}` — example units; **edit the absolute paths in the `.service` to match where you cloned this repo**.
