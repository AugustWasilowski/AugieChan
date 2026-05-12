"""Phase-3 patch: tools to add to ~/stackchan-mcp/stackchan_mcp.py.

We bolt onto the existing MCP rather than spinning up a new server, since
the user already has stackchan-mcp running in their Claude config.

Apply by appending these tool definitions and one constant to the existing
file. The TRACKER_URL constant goes near the top alongside BASE_URL.
"""
# --- to add near top, alongside BASE_URL ---
TRACKER_URL = os.environ.get(
    "STACKCHAN_TRACKER_URL", "http://127.0.0.1:5051"
).rstrip("/")


# --- to add at the bottom, before `if __name__ == "__main__"` ---

@mcp.tool()
def track_face_start() -> dict[str, Any]:
    """Start the face-following loop: StackChan will turn its head to keep
    the largest detected face centered in its camera frame. Requires the
    stackchan-tracker service to be running on this host."""
    with httpx.Client(timeout=TIMEOUT) as client:
        r = client.post(f"{TRACKER_URL}/track", json={"on": True})
        r.raise_for_status()
        return r.json()


@mcp.tool()
def track_face_stop() -> dict[str, Any]:
    """Stop the face-following loop. The head stays at its last position;
    you can /servo or /servo/home to reposition."""
    with httpx.Client(timeout=TIMEOUT) as client:
        r = client.post(f"{TRACKER_URL}/track", json={"on": False})
        r.raise_for_status()
        return r.json()


@mcp.tool()
def track_face_status() -> dict[str, Any]:
    """Get tracker status: tracking on/off, FPS, last face position, last
    servo command. Useful for debugging when the head isn't following."""
    with httpx.Client(timeout=TIMEOUT) as client:
        r = client.get(f"{TRACKER_URL}/metrics")
        r.raise_for_status()
        return r.json()
