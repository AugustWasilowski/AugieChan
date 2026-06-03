#!/usr/bin/env python3
"""StackChan MCP server.

Wraps the StackChan device's unauthenticated HTTP API as MCP tools so a
Claude Code session can drive the robot directly.

Endpoints (per device firmware):
  GET  /status
  POST /face          {"expression": str}
  POST /speak         {"text": str}
  POST /servo         {"yaw": int?, "pitch": int?}
  POST /servo/home
  POST /servo/stop
  POST /led           {"r": int, "g": int, "b": int}

The base URL is read from STACKCHAN_BASE_URL (default http://stackchan.local).
"""
from __future__ import annotations

import os
from typing import Any

import httpx
from fastmcp import FastMCP

BASE_URL = os.environ.get("STACKCHAN_BASE_URL", "http://stackchan.local").rstrip("/")
TIMEOUT = float(os.environ.get("STACKCHAN_TIMEOUT", "10"))

mcp = FastMCP("stackchan")


def _request(method: str, path: str, json: dict[str, Any] | None = None) -> dict[str, Any]:
    url = f"{BASE_URL}{path}"
    with httpx.Client(timeout=TIMEOUT) as client:
        resp = client.request(method, url, json=json)
        resp.raise_for_status()
        if not resp.content:
            return {"ok": True}
        ctype = resp.headers.get("content-type", "")
        if "application/json" in ctype:
            return resp.json()
        return {"ok": True, "body": resp.text}


@mcp.tool()
def status() -> dict[str, Any]:
    """Get StackChan status: ip, mac, rssi, battery, current expression, servo angles."""
    return _request("GET", "/status")


@mcp.tool()
def set_face(expression: str) -> dict[str, Any]:
    """Set the face expression. Common values: neutral, happy, sad, angry, sleepy, doubt."""
    return _request("POST", "/face", json={"expression": expression})


@mcp.tool()
def speak(text: str) -> dict[str, Any]:
    """Speak text via the StackChan's onboard TTS."""
    return _request("POST", "/speak", json={"text": text})


@mcp.tool()
def move_servo(yaw: int | None = None, pitch: int | None = None) -> dict[str, Any]:
    """Move pan (yaw) and/or tilt (pitch) servos to absolute angles in degrees."""
    payload: dict[str, Any] = {}
    if yaw is not None:
        payload["yaw"] = yaw
    if pitch is not None:
        payload["pitch"] = pitch
    return _request("POST", "/servo", json=payload)


@mcp.tool()
def home() -> dict[str, Any]:
    """Return the head to its home/center position."""
    return _request("POST", "/servo/home")


@mcp.tool()
def stop_servo() -> dict[str, Any]:
    """Immediately stop any in-flight servo motion."""
    return _request("POST", "/servo/stop")


@mcp.tool()
def set_led(r: int = 0, g: int = 0, b: int = 0) -> dict[str, Any]:
    """Set the onboard RGB LED. Each channel 0-255."""
    return _request("POST", "/led", json={"r": r, "g": g, "b": b})


@mcp.tool()
def set_buffer(pixels: list[list[int]], brightness: int = 64) -> dict[str, Any]:
    """Paint the 30-LED Port C strip in one shot.

    pixels is a list of [r, g, b] triplets; first N (max 30) LEDs are set, the rest cleared.
    brightness is 0..64 (hard-capped in firmware because Port C 5V can't deliver full-current).
    Empty pixels=[] clears the strip. Used as a progress bar by the macu-render pipeline.
    """
    return _request("POST", "/leds/buffer", json={"pixels": pixels, "brightness": brightness})


@mcp.tool()
def heartbeat(
    total: int = 0,
    running: int = 0,
    waiting: int = 0,
    tokens: int = 0,
    tokens_today: int = 0,
    prompt_id: str | None = None,
    prompt_tool: str | None = None,
    prompt_hint: str | None = None,
) -> dict[str, Any]:
    """Push a buddy heartbeat — the device picks the state from the payload.

    Mirrors the wire shape used by anthropics/claude-desktop-buddy. If prompt_id
    is set the device enters `attention` with a pending prompt. Otherwise the
    state is derived from running/waiting/tokens — and crossing each 50K-token
    boundary triggers a one-shot `celebrate`.
    """
    body: dict[str, Any] = {
        "total": total,
        "running": running,
        "waiting": waiting,
        "tokens": tokens,
        "tokens_today": tokens_today,
    }
    if prompt_id is not None:
        body["prompt"] = {"id": prompt_id}
        if prompt_tool is not None:
            body["prompt"]["tool"] = prompt_tool
        if prompt_hint is not None:
            body["prompt"]["hint"] = prompt_hint
    return _request("POST", "/heartbeat", json=body)


@mcp.tool()
def get_pending() -> dict[str, Any]:
    """Short-poll the device for a pending permission decision.

    Response: {pending: bool, prompt_id: str, decision: "once"|"deny"|""}.
    Reading a populated decision clears it on the device side; the next poll
    will return pending=false.
    """
    return _request("GET", "/pending")


@mcp.tool()
def set_state(state: str, prompt_id: str | None = None) -> dict[str, Any]:
    """Set the buddy state — drives face + LED ring + Port C strip atomically.

    state is one of: idle, busy, attention, celebrate, heart, nap.
    Maps loosely onto the claude-desktop-buddy behavior model:
      idle       no work pending          neutral face, dark
      busy       tools running            neutral face, dim blue ring
      attention  permission prompt open   doubt face, yellow ring + chase strip
      celebrate  token milestone hit      happy face, rainbow strip
      heart      quick approval (<5s)     happy face, dim red ring
      nap        idle > 30s               sleepy face, dark
    prompt_id is echoed back; future revs will use it for approve/deny round-trip.
    """
    payload: dict[str, Any] = {"state": state}
    if prompt_id is not None:
        payload["prompt_id"] = prompt_id
    return _request("POST", "/state", json=payload)


if __name__ == "__main__":
    mcp.run()
