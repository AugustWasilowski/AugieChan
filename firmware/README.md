# StackChan firmware (`StackChanBridge`)

Arduino sketch for the M5StackChan (ESP32-CoreS3 + servo base + camera + touch faceplate). Exposes the robot's capabilities as a small HTTP API and streams MJPEG from the onboard GC0308 camera.

## What it serves

Control HTTP on **`:80`** (Arduino `WebServer`):

| Method & path | Body | Notes |
| --- | --- | --- |
| `GET /status` | — | IP, MAC, RSSI, battery, current expression, servo angles, camera ready flag. |
| `POST /face` | `{"expression": "happy"|"sad"|"angry"|"sleepy"|"doubt"|"neutral", "mouth": 0..1, "eyes": 0..1, "breath": 0..1, "gaze_v": -1..1, "gaze_h": -1..1}` | All fields optional. |
| `POST /speak` | `{"text": "...", "hold_ms": 2500}` | Renders a speech bubble and animates the mouth. |
| `POST /servo` | `{"yaw": int, "pitch": int, "look_x": -1..1, "look_y": -1..1, "spin": int, "speed": 500}` | Either explicit angles or normalized `look_*`. |
| `POST /servo/home` | `{"speed": 500}` | Returns head to home. |
| `POST /servo/stop` | — | Cancels in-flight motion. |
| `POST /led` | `{"r": 0..255, "g": 0..255, "b": 0..255, "index": 0..11}` | Omit `index` to set all 12 LEDs at once. |
| `POST /leds` | `{"r": 0..255, "g": 0..255, "b": 0..255, "brightness": 0..64}` | Solid fill of the Port C 30-LED strip. |
| `POST /leds/pixel` | `{"index": 0..29, "r": 0..255, "g": 0..255, "b": 0..255}` | Set one strip pixel. |
| `POST /leds/effect` | `{"name": "rainbow"|"breathe"|"chase"|"off", "r": 0..255, "g": 0..255, "b": 0..255, "brightness": 0..64}` | Run a strip animation. |
| `POST /leds/buffer` | `{"pixels": [[r,g,b], ...], "brightness": 0..64}` | Paint the strip in one shot; used as a progress bar. |
| `POST /state` | `{"state": "idle"|"busy"|"attention"|"celebrate"|"heart"|"dizzy"|"nap", "prompt_id": "..."}` | Composite buddy-style state — drives face + ring + strip atomically. `celebrate`/`heart`/`dizzy` auto-revert after 2–3 s. See `host-hooks/`. |
| `POST /heartbeat` | `{"total": int, "running": int, "waiting": int, "tokens": int, "tokens_today": int, "prompt": {"id", "tool", "hint"}}` | Mirrors the [claude-desktop-buddy] heartbeat shape. Firmware derives the state: `prompt` → `attention`, running/waiting → `busy`, crossing each 50K-token boundary → one-shot `celebrate`. Snapshot is echoed in `/status`. |
| `GET /pending` | — | `{pending, prompt_id, decision}`. Short-poll for permission-prompt resolution. A populated decision is consumed (cleared device-side) by reading it. |
| `POST /reset` | — | Acks, then reboots ~250 ms later. |

[claude-desktop-buddy]: https://github.com/anthropics/claude-desktop-buddy

Camera HTTP on **`:81`** (ESP-IDF `httpd`, runs in its own task so streaming doesn't block control):

| Method & path | Notes |
| --- | --- |
| `GET /capture` | Single JPEG. |
| `GET /stream` | `multipart/x-mixed-replace` MJPEG, ~10–15 FPS at 320×240. |

The split-port design is deliberate — Arduino `WebServer` is single-threaded, so an MJPEG stream would otherwise block servo/face calls for the entire duration of the stream.

## Outbound events

When the faceplate's touch sensor reports `wasClicked()` / swipe forward / swipe back, the firmware POSTs a JSON event to `N8N_EVENT_URL` (configured in `secrets.h`). Useful for hooking the robot into automations on your own n8n / Home Assistant / etc.

**Permission flow exception**: while a buddy permission prompt is pending (set via `POST /state {"state":"attention","prompt_id":...}` or via `POST /heartbeat` with a `prompt` field), faceplate swipes are consumed locally instead of forwarding:

| Gesture | Effect |
| --- | --- |
| Swipe forward | Resolves `decision: "once"` on the pending prompt, plays a quick `heart` celebration. |
| Swipe backward | Resolves `decision: "deny"` on the pending prompt. |
| Click | Still forwarded — no resolve. |

The host reads the resolution via `GET /pending`. See `host-hooks/buddy_state.py` for the polling implementation.

## Shake → dizzy

When `M5.Imu`'s accelerometer magnitude registers three sharp jolts (Δ ≥ 1.2 G each) inside a 700 ms window, the firmware drops into the `dizzy` state for ~2 s — doubt face + rainbow strip. Disabled while a permission prompt is pending so you can pick the buddy up to swipe without triggering false dizzies.

## Build & flash

Open `StackChanBridge/StackChanBridge.ino` in Arduino IDE (or `arduino-cli`).

**Board**: ESP32-CoreS3 (PSRAM enabled).

**Libraries** (Arduino Library Manager):

- `M5StackChan` (M5Stack)
- `M5Unified` (pulled in by M5StackChan)
- `m5stack-avatar`
- `ArduinoJson`

ESP32 camera + OTA + mDNS are bundled with the ESP32 Arduino core.

**Secrets**:

```bash
cp StackChanBridge/secrets.h.example StackChanBridge/secrets.h
# then edit StackChanBridge/secrets.h
```

`secrets.h` is gitignored.

**First flash**: USB-C, then upload. The device prints its IP on the screen.

**Subsequent flashes**: WiFi OTA — `arduino-cli upload --port <ip-or-hostname>.local …` and supply the `OTA_PASSWORD` from `secrets.h`.

## Files in this directory

- `StackChanBridge/StackChanBridge.ino` — the sketch you flash.
- `StackChanBridge/secrets.h.example` — template; copy to `secrets.h`.
- `StackChanBridge.ino.orig` — baseline before camera, OTA, `/reset`, and touch-event forwarding were added. Kept as a reference; not used during builds.
