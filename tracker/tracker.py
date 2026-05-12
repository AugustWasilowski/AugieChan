#!/usr/bin/env python3
"""StackChan face-following control loop.

Pulls MJPEG frames from the StackChan's camera HTTP endpoint, finds the
largest face in each frame, and nudges the head's yaw/pitch toward
centering the face. Runs as a long-lived service; tracking is gated by a
local HTTP toggle so an MCP tool (or curl) can flip "follow my face"
on and off.

  GET  /health                -> {"ok": true, "tracking": bool, ...}
  POST /track {"on": bool}    -> toggle tracking
  GET  /metrics               -> last frame stats (face xy, error, fps)

Why a separate Python service rather than baking detection into the
firmware? Iteration speed. P-controller gain tuning is empirical; reload
this script and you've got a new gain in two seconds, vs. a 30s flash
cycle. The same camera stream stays usable for Frigate / vision LLMs.
"""
from __future__ import annotations

import argparse
import io
import json
import logging
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional

import cv2
import numpy as np
import requests

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

STACKCHAN_BASE  = os.environ.get("STACKCHAN_BASE", "http://10.0.0.134")
STREAM_URL      = os.environ.get("STACKCHAN_STREAM", f"{STACKCHAN_BASE}:81/stream")
SERVO_URL       = os.environ.get("STACKCHAN_SERVO", f"{STACKCHAN_BASE}/servo")
CONTROL_PORT    = int(os.environ.get("TRACKER_PORT", "5051"))

# systemd socket activation: when this process is spawned by systemd's
# stackchan-tracker.socket unit, LISTEN_FDS=1 and fd 3 is an already-listening
# socket. We use it instead of binding our own port. On /track off, we exit
# so systemd can hibernate the unit until the next inbound connection.
SOCKET_ACTIVATED = (
    int(os.environ.get("LISTEN_FDS", "0")) > 0
    and int(os.environ.get("LISTEN_PID", "0")) == os.getpid()
)


def _shutdown_cleanup():
    """Park the StackChan to a clean idle state. Called on /track off, before
    the response is returned and before any os._exit() timer fires.

    Each leg is best-effort with a short timeout — the StackChan can be
    flaky after extended use and we don't want a transient network blip
    to stop us from shutting down.
    """
    import requests as _req  # local import — `requests` is already imported at module top
    for path, body in (
        ("/led",        {"r": 0, "g": 0, "b": 0}),
        ("/face",       {"expression": "neutral"}),
        ("/servo/home", {"speed": 80}),
    ):
        try:
            _req.post(STACKCHAN_BASE + path, json=body, timeout=2)
            log.info("shutdown cleanup: %s ok", path)
        except Exception as e:
            log.warning("shutdown cleanup: %s failed (%s)", path, e)

# Tuning. Servo travel is yaw ~[-90,90], pitch ~[-30,30] for the StackChan.
# We drive the head via M5StackChan.lookAtNormalized(x, y, speed) — the library
# author literally annotated this as "ideal for visual tracking (e.g. centering a
# face in a camera frame)". Inputs are normalized -1..+1; the library maps to the
# physical servo range and handles whatever closed-loop quirks exist internally.
# (We learned the hard way that moveYaw + rotateYaw are NOT what we want.)
# Tuning notes:
#   - GAIN low so each command is a small nudge — prevents blur-induced overshoot
#   - COMMAND_HZ = 2 so each ~500ms M5StackChan move completes before the next
#   - SETTLE_AFTER_CMD_MS skips detection for that many ms after a movement
#     command, letting the head come to rest before we trust the next frame
#   - FACE_EMA smooths the detected face position across frames so single-frame
#     bbox jitter doesn't translate into servo jitter
# Defaults below are the values that produced solid tracking in the 2026-05-09
# tuning session — see ~/.claude/projects/-home-mayorawesome/memory/stackchan_face_tracking.md
GAIN_X = float(os.environ.get("GAIN_X", "0.20"))
GAIN_Y = float(os.environ.get("GAIN_Y", "0.20"))
DEADZONE_PIX        = int(os.environ.get("DEADZONE", "30"))
COMMAND_HZ          = float(os.environ.get("COMMAND_HZ", "2.0"))
SETTLE_AFTER_CMD_MS = int(os.environ.get("SETTLE_AFTER_CMD_MS", "450"))
SERVO_SPEED         = int(os.environ.get("SERVO_SPEED", "500"))
# INVERT_X=0, INVERT_Y=1 are CORRECT for this StackChan + camera mirror combo
INVERT_X            = os.environ.get("INVERT_X", "0") == "1"
INVERT_Y            = os.environ.get("INVERT_Y", "1") == "1"
FACE_EMA            = float(os.environ.get("FACE_EMA", "0.75"))
TARGET_HYSTERESIS   = float(os.environ.get("TARGET_HYSTERESIS", "0.10"))

# Home pose driven on every False -> True tracking edge. lookAtNormalized is an
# *absolute* command, so the tracker's internal target_x/y baseline must match
# the firmware's actual servo position or our first command will jerk the head
# away from wherever it physically is. Solution: at engage, drive the head to
# a known pose AND set internal target to the same value.
#
# Defaults below were derived empirically 2026-05-09 from a 60-second
# convergence trace with the user sitting at their desk (StackChan between
# keyboard and monitor). The tracker oscillated around look ≈ (+0.07, -0.38)
# while keeping the user's face roughly centered in frame. Setting HOME there
# lets the tracker engage directly into "looking at you" pose, no overshoot.
# Override per-machine via env vars HOME_LOOK_X / HOME_LOOK_Y.
HOME_LOOK_X = float(os.environ.get("HOME_LOOK_X", "0.07"))
HOME_LOOK_Y = float(os.environ.get("HOME_LOOK_Y", "-0.38"))

# Idle scanning: when no face has been seen for SCAN_AFTER_S, slowly sweep
# the head left/right. Stops the moment a face is detected.
SCAN_ENABLED   = os.environ.get("SCAN_ENABLED", "1") == "1"
SCAN_AFTER_S   = float(os.environ.get("SCAN_AFTER_S", "4.0"))
SCAN_AMPL      = float(os.environ.get("SCAN_AMPL", "0.4"))
SCAN_PERIOD_S  = float(os.environ.get("SCAN_PERIOD_S", "10.0"))

# Detection hysteresis: require N consecutive detections before commanding.
# YuNet is far less noisy than Haar, but we still want belt-and-suspenders to
# prevent a single bad frame from slamming the head.
MIN_DETECT_STREAK      = int(os.environ.get("MIN_DETECT_STREAK", "2"))
MIN_FACE_CONF          = float(os.environ.get("MIN_FACE_CONF", "0.75"))
MIN_FACE_PIX           = int(os.environ.get("MIN_FACE_PIX", "32"))    # bbox shorter side must exceed this
YUNET_MODEL            = os.environ.get(
    "YUNET_MODEL",
    os.path.join(os.path.dirname(__file__), "face_detection_yunet_2023mar.onnx"),
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("tracker")


# ---------------------------------------------------------------------------
# Shared state (read by HTTP control, written by tracker thread)
# ---------------------------------------------------------------------------

@dataclass
class State:
    tracking: bool   = False        # toggled via /track
    quiet: bool      = False        # if True, suppress /speak + /face calls
                                    # (used when n8n drives presentation — the
                                    # "Announce Home" workflow turns this on so
                                    # tracker.py only handles servos and lets
                                    # the workflow own faces / speech bubble)
    last_frame_ts: float = 0.0
    last_face_xy: Optional[tuple[int, int]] = None
    last_error_xy: Optional[tuple[int, int]] = None
    last_command: Optional[dict]   = None
    fps: float       = 0.0
    frame_count: int = 0
    detect_count: int = 0           # cumulative face detections since start
    error_count: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)


STATE = State()


# ---------------------------------------------------------------------------
# Face detection: OpenCV YuNet (ONNX). Way more robust than Haar — far fewer
# false positives on textured backgrounds (carpet, doors, wood grain), and
# returns confidence so we can threshold. Model is ~230KB.
# ---------------------------------------------------------------------------

if not os.path.isfile(YUNET_MODEL):
    raise RuntimeError(f"YuNet model not found at {YUNET_MODEL}")

# Input size set per-frame in detect_largest_face since QVGA is fixed.
_FACE = cv2.FaceDetectorYN.create(YUNET_MODEL, "", (320, 240),
                                  score_threshold=MIN_FACE_CONF,
                                  nms_threshold=0.3, top_k=5)


def detect_largest_face(bgr: np.ndarray) -> Optional[tuple[int, int, int, int, float]]:
    """Return (x, y, w, h, score) of the largest face, or None."""
    h, w = bgr.shape[:2]
    _FACE.setInputSize((w, h))
    n, faces = _FACE.detect(bgr)
    if faces is None or len(faces) == 0:
        return None
    # YuNet returns rows of [x, y, w, h, lm_x*5, lm_y*5, score]
    best = max(faces, key=lambda f: f[2] * f[3])
    x, y, fw, fh = (int(best[0]), int(best[1]), int(best[2]), int(best[3]))
    score = float(best[-1])
    if min(fw, fh) < MIN_FACE_PIX:
        return None
    return (x, y, fw, fh, score)


# ---------------------------------------------------------------------------
# MJPEG stream parser. The Arduino-style multipart format is consistent
# with what our firmware emits: \r\n--<boundary>\r\nContent-Type: image/jpeg
# \r\nContent-Length: N\r\n\r\n<bytes>\r\n
# ---------------------------------------------------------------------------

def mjpeg_frames(url: str, timeout: float = 5.0):
    """Yield JPEG byte strings from an MJPEG endpoint. Reconnects on error."""
    while True:
        try:
            log.info("connecting to stream %s", url)
            with requests.get(url, stream=True, timeout=timeout) as r:
                r.raise_for_status()
                buf = b""
                for chunk in r.iter_content(chunk_size=4096):
                    if not chunk:
                        continue
                    buf += chunk
                    while True:
                        # locate JPEG SOI/EOI markers; simplest robust parser
                        soi = buf.find(b"\xff\xd8")
                        if soi < 0:
                            break
                        eoi = buf.find(b"\xff\xd9", soi + 2)
                        if eoi < 0:
                            break
                        jpeg = buf[soi:eoi + 2]
                        buf = buf[eoi + 2:]
                        yield jpeg
        except Exception as e:
            STATE.error_count += 1
            log.warning("stream error: %s; retrying in 2s", e)
            time.sleep(2)


# ---------------------------------------------------------------------------
# Servo command. The firmware accepts {"yaw": int, "pitch": int, "speed": int}
# in absolute degrees. We get current angles from /status and add deltas.
# ---------------------------------------------------------------------------

def get_current_yaw_pitch() -> Optional[tuple[float, float]]:
    try:
        r = requests.get(f"{STACKCHAN_BASE}/status", timeout=2)
        r.raise_for_status()
        d = r.json()
        return float(d.get("yaw", 0)), float(d.get("pitch", 0))
    except Exception as e:
        log.warning("status fetch failed: %s", e)
        return None


def post_look(look_x: float, look_y: float) -> bool:
    """Drive the head with M5StackChan.Motion.lookAtNormalized via the firmware's
    {look_x, look_y, speed} servo body. Inputs are clamped to -1..+1."""
    try:
        body = {
            "look_x": float(clamp(look_x, -1.0, 1.0)),
            "look_y": float(clamp(look_y, -1.0, 1.0)),
            "speed":  SERVO_SPEED,
        }
        r = requests.post(SERVO_URL, json=body, timeout=2)
        r.raise_for_status()
        with STATE.lock:
            STATE.last_command = body
        return True
    except Exception as e:
        log.warning("servo POST failed: %s", e)
        return False


# ---------------------------------------------------------------------------
# Speech-bubble narration. The firmware /speak endpoint shows text in the
# avatar's bubble for hold_ms. We don't want to spam — only re-post when the
# narration actually changes, and add a tiny refresh so the bubble doesn't
# expire while state is unchanged.
# ---------------------------------------------------------------------------

_NARRATION = {"text": None, "ts": 0.0}

def narrate(text: str, hold_ms: int = 4000):
    # In quiet mode the n8n workflow owns the screen — chunked speech bubbles
    # of the actual announcement text — and our chatty "looking for a face..."
    # narrations would fight it. Bail out early.
    if STATE.quiet:
        return
    now = time.time()
    if text == _NARRATION["text"] and now - _NARRATION["ts"] < (hold_ms / 1000.0) * 0.6:
        return  # still showing same message; let it ride
    try:
        requests.post(f"{STACKCHAN_BASE}/speak",
                      json={"text": text, "hold_ms": hold_ms}, timeout=1.5)
        _NARRATION["text"] = text
        _NARRATION["ts"] = now
    except Exception as e:
        log.debug("speak POST failed: %s", e)


def post_face(expression: str):
    """Set avatar expression. Cheap; safe to call repeatedly with the same value."""
    if STATE.quiet:
        # Workflow drives the face emote sequence; don't override it.
        return
    try:
        requests.post(f"{STACKCHAN_BASE}/face",
                      json={"expression": expression}, timeout=1.5)
    except Exception:
        pass


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def tracking_loop():
    last_command_ts = 0.0
    last_face_ts = 0.0      # timestamp of last successful detection (for scan trigger)
    last_scan_cmd_ts = 0.0
    fps_window = []
    # Persistent target — we accumulate face-position evidence into a remembered
    # look point so successive small errors compound smoothly toward the face,
    # rather than each frame fighting from "look forward" baseline.
    target_x = 0.0
    target_y = 0.0
    last_sent_x = 0.0
    last_sent_y = 0.0
    smooth_face_xy: Optional[tuple[float, float]] = None
    detect_streak = 0
    scan_t0 = 0.0           # set when we first enter scan mode (resets cosine phase)
    was_tracking = False    # for detecting False->True engage edge

    for jpeg in mjpeg_frames(STREAM_URL):
        now = time.time()
        STATE.frame_count += 1
        STATE.last_frame_ts = now

        # rolling FPS over last 1s
        fps_window = [t for t in fps_window if now - t < 1.0] + [now]
        STATE.fps = float(len(fps_window))

        # Engage edge: drive head to HOME pose so internal target matches
        # physical reality. See HOME_LOOK_X/Y comment block above.
        if STATE.tracking and not was_tracking:
            log.info("tracker engaging — homing head to (%.2f, %.2f)",
                     HOME_LOOK_X, HOME_LOOK_Y)
            post_look(HOME_LOOK_X, HOME_LOOK_Y)
            target_x = HOME_LOOK_X
            target_y = HOME_LOOK_Y
            last_sent_x = HOME_LOOK_X
            last_sent_y = HOME_LOOK_Y
            last_command_ts = now
            smooth_face_xy = None
            detect_streak = 0
            scan_t0 = 0.0
            last_face_ts = 0.0
        was_tracking = STATE.tracking

        if not STATE.tracking:
            # Tracker idle. We used to re-narrate "(tracking off)" every ~5s
            # so the speech bubble had a known resting state, but it just
            # flashed on screen forever and obscured whatever the n8n
            # workflow / user had put up there last. Silence is better.
            continue

        # Skip frames while the head is mid-move — motion blur trashes detection.
        if last_command_ts > 0 and (now - last_command_ts) * 1000 < SETTLE_AFTER_CMD_MS:
            continue

        # decode + detect
        try:
            bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
        except Exception as e:
            log.warning("decode failed: %s", e)
            continue

        face = detect_largest_face(bgr)
        if face is None:
            if detect_streak > 0:
                # transition: we just lost a face. Speak about it.
                narrate("hmm... where'd you go?", hold_ms=2500)
                post_face("doubt")
            elif STATE.frame_count % 30 == 0:
                narrate("looking for a face...", hold_ms=2500)
            detect_streak = 0
            with STATE.lock:
                STATE.last_face_xy = None
                STATE.last_error_xy = None

            # Scan mode: if we haven't seen a face for SCAN_AFTER_S, slowly
            # sweep the head looking for one. Cosine for natural motion.
            if SCAN_ENABLED and last_face_ts > 0 and now - last_face_ts > SCAN_AFTER_S:
                if scan_t0 == 0.0:
                    scan_t0 = now
                    target_x = 0.0
                if now - last_scan_cmd_ts >= 1.0 / COMMAND_HZ:
                    import math
                    phase = ((now - scan_t0) / SCAN_PERIOD_S) * 2 * math.pi
                    target_x = SCAN_AMPL * math.sin(phase)
                    # Preserve the last tracked pitch — if the face was found at a
                    # particular height, keep looking there during scan. This avoids
                    # an oscillation where scan resets pitch to 0 and tracking has to
                    # re-tilt every time the face is briefly lost.
                    if post_look(target_x, target_y):
                        last_scan_cmd_ts = now
            continue

        was_lost = (detect_streak == 0)
        detect_streak += 1
        STATE.detect_count += 1
        last_face_ts = now
        scan_t0 = 0.0          # exit scan mode
        x, y, w, h, score = face
        cx_raw = x + w // 2
        cy_raw = y + h // 2
        # EMA-smooth the face position so single-frame bbox jitter doesn't move servos
        if smooth_face_xy is None or was_lost:
            smooth_face_xy = (float(cx_raw), float(cy_raw))
        else:
            smooth_face_xy = (
                FACE_EMA * smooth_face_xy[0] + (1 - FACE_EMA) * cx_raw,
                FACE_EMA * smooth_face_xy[1] + (1 - FACE_EMA) * cy_raw,
            )
        if was_lost:
            narrate(f"hi! ({int(score*100)}%)", hold_ms=3000)
            post_face("happy")
            log.info("face acquired at (%d,%d) score=%.2f", cx_raw, cy_raw, score)
        cx = int(smooth_face_xy[0])
        cy = int(smooth_face_xy[1])
        H, W = bgr.shape[:2]
        err_x = cx - W // 2
        err_y = cy - H // 2

        with STATE.lock:
            STATE.last_face_xy = (cx, cy)
            STATE.last_error_xy = (err_x, err_y)

        # gate movement on consecutive detections — single-frame ghosts don't move the head
        if detect_streak < MIN_DETECT_STREAK:
            continue

        # rate-limit commands: control loop runs at ~COMMAND_HZ
        if now - last_command_ts < 1.0 / COMMAND_HZ:
            continue

        # deadzone
        if abs(err_x) < DEADZONE_PIX and abs(err_y) < DEADZONE_PIX:
            continue

        # P-controller in normalized look-space. err is in pixels; the half-frame
        # is 160 (h) / 120 (v). Scale to a normalized step and accumulate into
        # the persistent look target. Library maps -1..+1 to full physical range,
        # so for a face mid-frame edge (err_x=160) the step contribution at gain
        # 0.7 is ~0.7 — a strong corrective nudge.
        step_x = (err_x / 160.0) * GAIN_X
        step_y = (err_y / 120.0) * GAIN_Y
        if INVERT_X: step_x = -step_x
        if INVERT_Y: step_y = -step_y

        target_x = clamp(target_x + step_x, -1.0, 1.0)
        target_y = clamp(target_y + step_y, -1.0, 1.0)

        # Hysteresis: only send a command if the target has moved meaningfully.
        # Without this, jittery face detection keeps sending nearly-identical
        # commands that the servo translates into visible micro-twitches.
        if (abs(target_x - last_sent_x) < TARGET_HYSTERESIS and
            abs(target_y - last_sent_y) < TARGET_HYSTERESIS):
            continue

        if post_look(target_x, target_y):
            last_sent_x, last_sent_y = target_x, target_y
            last_command_ts = now


# ---------------------------------------------------------------------------
# Local control HTTP (bound to 0.0.0.0; used by MCP / curl / n8n container).
# n8n runs in Docker so it can't reach 127.0.0.1 on the host — it dials the
# host's LAN IP (10.0.0.72:5051) instead. LAN-only; no port-forward exists.
# ---------------------------------------------------------------------------

class Control(BaseHTTPRequestHandler):
    def _json(self, code: int, body: dict):
        payload = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt, *args):
        pass  # silence default access log

    def do_GET(self):
        if self.path == "/health":
            self._json(200, {"ok": True, "tracking": STATE.tracking,
                             "quiet": STATE.quiet,
                             "fps": STATE.fps, "frames": STATE.frame_count})
            return
        if self.path == "/metrics":
            with STATE.lock:
                self._json(200, {
                    "ok": True,
                    "tracking": STATE.tracking,
                    "quiet": STATE.quiet,
                    "fps": STATE.fps,
                    "frame_count": STATE.frame_count,
                    "detect_count": STATE.detect_count,
                    "error_count": STATE.error_count,
                    "last_frame_age_s": time.time() - STATE.last_frame_ts if STATE.last_frame_ts else None,
                    "last_face_xy": STATE.last_face_xy,
                    "last_error_xy": STATE.last_error_xy,
                    "last_command": STATE.last_command,
                })
            return
        self._json(404, {"ok": False, "error": "no such route"})

    def do_POST(self):
        if self.path == "/track":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
            except Exception:
                self._json(400, {"ok": False, "error": "bad json"})
                return
            STATE.tracking = bool(body.get("on", False))
            # quiet is optional. If absent, leave the prior value alone so that
            # callers who only want to toggle on/off don't have to re-state it.
            if "quiet" in body:
                STATE.quiet = bool(body["quiet"])
            log.info("tracking=%s quiet=%s", STATE.tracking, STATE.quiet)

            # On /track off (regardless of prior state): park the head and
            # reset visible state. LEDs off, face neutral, servos home at
            # the workflow's slow speed=80. Best-effort — any failed call
            # just logs and moves on. This finishes BEFORE we send the
            # response, and certainly before the os._exit timer fires.
            if not STATE.tracking:
                _shutdown_cleanup()

            self._json(200, {"ok": True,
                             "tracking": STATE.tracking,
                             "quiet": STATE.quiet})
            # Under socket activation we want to hibernate when nothing is
            # tracking — exit cleanly so systemd reclaims the port and the
            # heavy camera-capture loop stops. The 0.5s delay lets this
            # response flush before we tear down.
            if not STATE.tracking and SOCKET_ACTIVATED:
                log.info("tracking off + socket-activated — exiting so systemd hibernates the unit")
                threading.Timer(0.5, lambda: os._exit(0)).start()
            return
        self._json(404, {"ok": False, "error": "no such route"})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-tracking", action="store_true",
                        help="Begin in tracking-on state")
    args = parser.parse_args()
    STATE.tracking = args.start_tracking

    t = threading.Thread(target=tracking_loop, daemon=True)
    t.start()

    # systemd socket activation: grab the inherited fd 3 instead of binding
    # our own. bind_and_activate=False on HTTPServer skips the bind+listen
    # since systemd already did both.
    if SOCKET_ACTIVATED:
        inherited = socket.socket(fileno=3)
        srv = HTTPServer(("0.0.0.0", CONTROL_PORT), Control, bind_and_activate=False)
        srv.socket = inherited
        srv.server_address = inherited.getsockname()
        log.info("tracker control on systemd-passed fd 3 (%s)", srv.server_address)
    else:
        srv = HTTPServer(("0.0.0.0", CONTROL_PORT), Control)
        log.info("tracker control on http://0.0.0.0:%d  (POST /track {\"on\":true})", CONTROL_PORT)
    log.info("stream=%s  servo=%s", STREAM_URL, SERVO_URL)
    srv.serve_forever()


if __name__ == "__main__":
    main()
