# AugieChan

Custom firmware and host-side tooling for an [M5Stack StackChan](https://github.com/stack-chan/stack-chan) robot — an ESP32-CoreS3-based desktop companion with two servos, a camera, an RGB LED ring, and a touch sensor on a face plate.

This repo holds three pieces that work together:

| Piece | Lives on | What it does |
| --- | --- | --- |
| **[`firmware/`](firmware/)** | The StackChan (ESP32-CoreS3) | HTTP API around the M5StackChan SDK: servo, face, speech bubble, LED, MJPEG camera stream on `:81`, OTA flashing, and touch-sensor events POSTed to an n8n webhook. |
| **[`tracker/`](tracker/)** | A Linux host on the same LAN | Socket-activated systemd service that pulls the StackChan's MJPEG stream, runs OpenCV YuNet face detection, and POSTs servo commands so the head follows the largest face. |
| **[`mcp-server/`](mcp-server/)** | Anywhere Claude Code runs | An [MCP](https://modelcontextprotocol.io) server that wraps the firmware's HTTP API as tools so an LLM session can drive the robot directly. Optional `mcp_patch.py` adds face-tracker controls. |

## Wiring it all together

```
                              ┌──────────────────────────┐
                              │   StackChan (ESP32-S3)   │
                              │   firmware/              │
                              │                          │
                       ┌────► │ :80  control HTTP        │
                       │      │ :81  MJPEG /stream       │
                       │      └─────────┬────────────────┘
                       │                │ MJPEG
                       │                ▼
   Claude Code ──MCP──►│      ┌──────────────────────────┐
                       │      │  Linux host              │
                       │      │  tracker/  (socket-     │
                       └──────┤  activated systemd)      │
                              │  YuNet → /servo deltas   │
                              └──────────────────────────┘
```

See each subdirectory's README for setup specifics. Start with [`firmware/`](firmware/).

## License

MIT — see [LICENSE](LICENSE).

The bundled `tracker/face_detection_yunet_2023mar.onnx` is the YuNet model from the [OpenCV Zoo](https://github.com/opencv/opencv_zoo) and is redistributed under its own MIT license.
