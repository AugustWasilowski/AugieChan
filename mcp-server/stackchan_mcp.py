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


if __name__ == "__main__":
    mcp.run()
