#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include <M5StackChan.h>
#include <Avatar.h>
#include <Adafruit_NeoPixel.h>

// Camera support: native ESP32 camera driver + ESP-IDF HTTP server.
// We run a SECOND httpd on port 81 dedicated to camera streaming, so MJPEG
// streams don't block the control endpoints (servo/face/speak) on port 80.
#include "esp_camera.h"
extern "C" {
  #include "esp_http_server.h"
}

#include "secrets.h"

using namespace m5avatar;

static Avatar avatar;
static WebServer server(HTTP_PORT);
static String localIP;

// Camera state -------------------------------------------------------------
static volatile bool s_camera_ready = false;
static httpd_handle_t s_camera_httpd = nullptr;

// Soft-reset coordinator -- we ack /reset before actually rebooting so the
// caller's HTTP exchange completes cleanly.
static volatile uint32_t s_reset_at_ms = 0;

// Pin mapping cribbed verbatim from M5CoreS3/utility/GC0308.cpp (M5Stack's
// official driver). The GC0308 does NOT support hardware JPEG output (returns
// ESP_ERR_NOT_SUPPORTED at init), so we capture RGB565 and software-encode
// each frame in the capture/stream handlers via frame2jpg().
static const int JPEG_QUALITY = 60;   // 0–100, higher = better; ~30ms encode
static camera_config_t s_camera_config = {
    .pin_pwdn     = -1,
    .pin_reset    = -1,
    .pin_xclk     = -1,
    .pin_sscb_sda = 12,
    .pin_sscb_scl = 11,
    .pin_d7 = 47, .pin_d6 = 48, .pin_d5 = 16, .pin_d4 = 15,
    .pin_d3 = 42, .pin_d2 = 41, .pin_d1 = 40, .pin_d0 = 39,
    .pin_vsync = 46, .pin_href = 38, .pin_pclk = 45,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size   = FRAMESIZE_QVGA,   // 320x240 — plenty for face tracking
    .jpeg_quality = 0,                 // unused for RGB565
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1,
};

// speech-bubble auto-clear: tracked in loop(), no FreeRTOS task
static volatile uint32_t s_speech_clear_at_ms = 0;
static volatile bool     s_speaking = false;

// /play state. M5.Speaker.playWav holds a pointer (not a copy) into the WAV
// buffer until playback ends, so we own the buffer for the lifetime of the
// playback. We stop() and free() on the next /play call.
static uint8_t* s_play_buf = nullptr;
static size_t   s_play_buf_len = 0;
static const size_t PLAY_BUF_MAX = 4 * 1024 * 1024;   // 4 MB hard cap

// WS2812 strip on Port C (blue, top). Pin 1 of Port C on CoreS3 = GPIO 18.
// Port A is unusable here because it shares I2C with the camera SCCB bus.
// 30 LEDs at brightness 255 white draws ~1.8A; Port C's 5V can deliver maybe
// 500mA before the CoreS3 browns out, so we hard-cap brightness at 64 (~25%).
static const uint8_t  STRIP_PIN = 18;
static const uint16_t STRIP_COUNT = 30;
static const uint8_t  STRIP_MAX_BRIGHTNESS = 64;
static Adafruit_NeoPixel strip(STRIP_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);

enum StripMode : uint8_t { STRIP_MODE_STATIC, STRIP_MODE_RAINBOW, STRIP_MODE_BREATHE, STRIP_MODE_CHASE };
static StripMode s_strip_mode = STRIP_MODE_STATIC;
static uint32_t  s_strip_base_color = 0;                // packed RGB for breathe/chase
static uint16_t  s_strip_phase = 0;                     // animation phase counter
static uint32_t  s_strip_last_tick_ms = 0;
static const uint16_t STRIP_TICK_INTERVAL_MS = 30;      // ~33 FPS

static const char* expressionName(Expression e) {
  switch (e) {
    case Expression::Happy:   return "happy";
    case Expression::Angry:   return "angry";
    case Expression::Sad:     return "sad";
    case Expression::Doubt:   return "doubt";
    case Expression::Sleepy:  return "sleepy";
    case Expression::Neutral: return "neutral";
  }
  return "neutral";
}

static bool parseExpression(const String& name, Expression& out) {
  if (name == "happy")   { out = Expression::Happy;   return true; }
  if (name == "angry")   { out = Expression::Angry;   return true; }
  if (name == "sad")     { out = Expression::Sad;     return true; }
  if (name == "doubt")   { out = Expression::Doubt;   return true; }
  if (name == "sleepy")  { out = Expression::Sleepy;  return true; }
  if (name == "neutral") { out = Expression::Neutral; return true; }
  return false;
}

// ---- helpers ---------------------------------------------------------------

static void sendJson(int code, const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  server.send(code, "application/json", body);
}

static void sendErr(int code, const char* msg) {
  JsonDocument d;
  d["ok"] = false;
  d["error"] = msg;
  sendJson(code, d);
}

static bool readJsonBody(JsonDocument& out) {
  if (!server.hasArg("plain")) return false;
  return deserializeJson(out, server.arg("plain")) == DeserializationError::Ok;
}

// ---- HTTP routes -----------------------------------------------------------

static void handleStatus() {
  JsonDocument d;
  d["ok"] = true;
  d["device"] = "stackchan";
  d["uptime_ms"] = (uint32_t)millis();
  d["ip"] = localIP;
  d["mac"] = WiFi.macAddress();
  d["rssi"] = WiFi.RSSI();
  d["battery_v"] = M5StackChan.getBatteryVoltage();
  d["battery_a"] = M5StackChan.getBatteryCurrent();
  d["expression"] = expressionName(avatar.getExpression());
  d["yaw"] = M5StackChan.Motion.getCurrentYawAngle();
  d["pitch"] = M5StackChan.Motion.getCurrentPitchAngle();
  d["camera"] = s_camera_ready;
  sendJson(200, d);
}

static void handleFace() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }

  if (body["expression"].is<const char*>()) {
    Expression e;
    if (parseExpression(body["expression"].as<String>(), e)) {
      avatar.setExpression(e);
    } else {
      sendErr(400, "unknown expression");
      return;
    }
  }
  if (body["mouth"].is<float>())   avatar.setMouthOpenRatio(body["mouth"].as<float>());
  if (body["eyes"].is<float>())    avatar.setEyeOpenRatio(body["eyes"].as<float>());
  if (body["breath"].is<float>())  avatar.setBreath(body["breath"].as<float>());
  if (body["gaze_v"].is<float>() || body["gaze_h"].is<float>()) {
    float v = body["gaze_v"] | 0.0f;
    float h = body["gaze_h"] | 0.0f;
    avatar.setLeftGaze(v, h);
    avatar.setRightGaze(v, h);
  }

  JsonDocument res;
  res["ok"] = true;
  res["expression"] = expressionName(avatar.getExpression());
  sendJson(200, res);
}

static void handleSpeak() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  String text = body["text"] | "";
  uint32_t hold_ms = body["hold_ms"] | 2500;

  avatar.setSpeechText(text.c_str());
  // crude lip-sync: open mouth, then clear from loop() once hold_ms has elapsed
  avatar.setMouthOpenRatio(0.6f);
  s_speech_clear_at_ms = millis() + hold_ms;
  s_speaking = true;

  JsonDocument res;
  res["ok"] = true;
  res["text"] = text;
  res["hold_ms"] = hold_ms;
  sendJson(200, res);
}

static void handleServo() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  int speed = body["speed"] | 500;
  bool moved = false;

  if (body["yaw"].is<int>() && body["pitch"].is<int>()) {
    M5StackChan.Motion.move(body["yaw"].as<int>(), body["pitch"].as<int>(), speed);
    moved = true;
  } else if (body["yaw"].is<int>()) {
    M5StackChan.Motion.moveYaw(body["yaw"].as<int>(), speed);
    moved = true;
  } else if (body["pitch"].is<int>()) {
    M5StackChan.Motion.movePitch(body["pitch"].as<int>(), speed);
    moved = true;
  }
  if (body["look_x"].is<float>() || body["look_y"].is<float>()) {
    float x = body["look_x"] | 0.0f;
    float y = body["look_y"] | 0.0f;
    M5StackChan.Motion.lookAtNormalized(x, y, speed);
    moved = true;
  }
  if (body["spin"].is<int>()) {
    M5StackChan.Motion.rotateYaw(body["spin"].as<int>());
    moved = true;
  }

  if (!moved) { sendErr(400, "need yaw/pitch/look_x/look_y/spin"); return; }
  JsonDocument res; res["ok"] = true; sendJson(200, res);
}

static void handleHome() {
  int speed = 500;
  if (server.hasArg("plain")) {
    JsonDocument body;
    if (deserializeJson(body, server.arg("plain")) == DeserializationError::Ok) {
      speed = body["speed"] | 500;
    }
  }
  M5StackChan.Motion.goHome(speed);
  JsonDocument res; res["ok"] = true; sendJson(200, res);
}

static void handleStopServo() {
  M5StackChan.Motion.stop();
  JsonDocument res; res["ok"] = true; sendJson(200, res);
}

static void handleLed() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  uint8_t r = body["r"] | 0;
  uint8_t g = body["g"] | 0;
  uint8_t b = body["b"] | 0;

  if (body["index"].is<int>()) {
    int idx = body["index"].as<int>();
    if (idx < 0 || idx > 11) { sendErr(400, "index 0..11"); return; }
    M5StackChan.setRgbColor(idx, r, g, b);
    M5StackChan.refreshRgb();
  } else {
    M5StackChan.showRgbColor(r, g, b);
  }
  JsonDocument res; res["ok"] = true; sendJson(200, res);
}

static void handleReset() {
  // Reply first so the caller doesn't see a TCP RST. Schedule the actual
  // restart from loop() ~250ms after sending. Keeps it simple and safe.
  JsonDocument res; res["ok"] = true; res["restarting_in_ms"] = 250;
  sendJson(200, res);
  s_reset_at_ms = millis() + 250;
}

// POST /play  body: {"url":"http://...","volume":128,"stop_current":true}
// StackChan fetches the WAV via HTTPClient and plays it through the M5Speaker.
// Returns 200 with {ok:true, bytes:<n>} once the audio is queued — does NOT
// block on playback (so /servo, /face stay responsive).
static void handlePlay() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  String url = body["url"] | "";
  int volume = body["volume"] | 128;          // 0..255
  bool stop_current = body["stop_current"] | true;

  if (url.isEmpty()) { sendErr(400, "need url"); return; }
  if (volume < 0) volume = 0;
  if (volume > 255) volume = 255;

  WiFiClientSecure secure;
  secure.setInsecure();
  HTTPClient http;
  bool is_https = url.startsWith("https://");
  bool begin_ok = is_https ? http.begin(secure, url) : http.begin(url);
  if (!begin_ok) { sendErr(502, "http begin failed"); return; }
  http.setTimeout(15000);
  http.setConnectTimeout(5000);

  int code = http.GET();
  if (code != 200) {
    http.end();
    JsonDocument d; d["ok"] = false; d["error"] = "fetch failed"; d["http_code"] = code;
    sendJson(502, d);
    return;
  }

  int total = http.getSize();
  if (total <= 0 || (size_t)total > PLAY_BUF_MAX) {
    http.end();
    JsonDocument d; d["ok"] = false; d["error"] = "bad content-length"; d["size"] = total;
    sendJson(400, d);
    return;
  }

  uint8_t* buf = (uint8_t*)ps_malloc((size_t)total);
  if (!buf) { http.end(); sendErr(507, "ps_malloc failed"); return; }

  WiFiClient* stream = http.getStreamPtr();
  size_t got = 0;
  uint32_t deadline = millis() + 20000;
  while (got < (size_t)total && millis() < deadline) {
    int avail = stream->available();
    if (avail > 0) {
      int r = stream->readBytes(buf + got, avail);
      if (r > 0) got += r;
    } else if (!http.connected()) {
      break;
    } else {
      delay(2);
    }
  }
  http.end();

  if (got != (size_t)total) {
    free(buf);
    JsonDocument d; d["ok"] = false; d["error"] = "short read"; d["got"] = got; d["want"] = total;
    sendJson(502, d);
    return;
  }

  // Stop any in-flight playback (drains DMA) before freeing its buffer.
  if (s_play_buf != nullptr) {
    M5.Speaker.stop();
    free(s_play_buf);
    s_play_buf = nullptr;
    s_play_buf_len = 0;
  }
  s_play_buf = buf;
  s_play_buf_len = (size_t)total;

  M5.Speaker.setVolume((uint8_t)volume);
  bool ok = M5.Speaker.playWav(s_play_buf, s_play_buf_len, 1, -1, stop_current);

  JsonDocument res;
  res["ok"] = ok;
  res["bytes"] = (uint32_t)s_play_buf_len;
  res["volume"] = volume;
  sendJson(ok ? 200 : 500, res);
}

// ---- WS2812 strip helpers ------------------------------------------------

static uint8_t clampBrightness(int v) {
  if (v < 0) return 0;
  if (v > STRIP_MAX_BRIGHTNESS) return STRIP_MAX_BRIGHTNESS;
  return (uint8_t)v;
}

static bool parseRgb(JsonDocument& body, uint8_t* r, uint8_t* g, uint8_t* b) {
  if (!body["r"].is<int>() || !body["g"].is<int>() || !body["b"].is<int>()) return false;
  int rr = body["r"]; int gg = body["g"]; int bb = body["b"];
  if (rr < 0 || rr > 255 || gg < 0 || gg > 255 || bb < 0 || bb > 255) return false;
  *r = (uint8_t)rr; *g = (uint8_t)gg; *b = (uint8_t)bb;
  return true;
}

// Adafruit_NeoPixel ColorHSV returns gamma-uncorrected; ColorHSV + gamma32
// gives perceptually-even rainbows. Hue is 0..65535.
static uint32_t hsv(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {
  return Adafruit_NeoPixel::gamma32(Adafruit_NeoPixel::ColorHSV(hue, sat, val));
}

// Step the active effect. Called from loop(); cheap when nothing to do.
static void stripTick() {
  if (s_strip_mode == STRIP_MODE_STATIC) return;
  uint32_t now = millis();
  if (now - s_strip_last_tick_ms < STRIP_TICK_INTERVAL_MS) return;
  s_strip_last_tick_ms = now;
  s_strip_phase++;

  switch (s_strip_mode) {
    case STRIP_MODE_RAINBOW: {
      // Full hue rotation across the strip, slowly drifting.
      uint16_t base = s_strip_phase * 256;   // ~7.6s per full rotation
      for (uint16_t i = 0; i < STRIP_COUNT; ++i) {
        uint16_t hue = base + (uint16_t)((uint32_t)i * 65536 / STRIP_COUNT);
        strip.setPixelColor(i, hsv(hue));
      }
      strip.show();
      break;
    }
    case STRIP_MODE_BREATHE: {
      // Triangle wave 0..255..0 over ~3s, scaled to brightness cap.
      uint16_t p = s_strip_phase % 200;
      uint8_t lvl = (p < 100) ? (p * 255 / 100) : ((200 - p) * 255 / 100);
      uint8_t r = ((s_strip_base_color >> 16) & 0xFF) * lvl / 255;
      uint8_t g = ((s_strip_base_color >>  8) & 0xFF) * lvl / 255;
      uint8_t b = ((s_strip_base_color      ) & 0xFF) * lvl / 255;
      strip.fill(strip.Color(r, g, b));
      strip.show();
      break;
    }
    case STRIP_MODE_CHASE: {
      // One pixel of base_color travels around; everything else dark.
      uint16_t head = s_strip_phase % STRIP_COUNT;
      strip.clear();
      strip.setPixelColor(head, s_strip_base_color);
      strip.show();
      break;
    }
    default: break;
  }
}

// POST /leds  body: {r,g,b,brightness?}    -> solid color on all pixels
static void handleLeds() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  uint8_t r, g, b;
  if (!parseRgb(body, &r, &g, &b)) { sendErr(400, "need r,g,b (0..255)"); return; }
  uint8_t bright = clampBrightness(body["brightness"] | (int)STRIP_MAX_BRIGHTNESS);

  s_strip_mode = STRIP_MODE_STATIC;
  strip.setBrightness(bright);
  strip.fill(strip.Color(r, g, b));
  strip.show();

  JsonDocument res;
  res["ok"] = true;
  res["mode"] = "solid";
  res["brightness"] = bright;
  sendJson(200, res);
}

// POST /leds/pixel  body: {index,r,g,b}    -> set one pixel (STATIC mode)
static void handleLedsPixel() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  if (!body["index"].is<int>()) { sendErr(400, "need index"); return; }
  int idx = body["index"];
  if (idx < 0 || idx >= STRIP_COUNT) { sendErr(400, "index 0..29"); return; }
  uint8_t r, g, b;
  if (!parseRgb(body, &r, &g, &b)) { sendErr(400, "need r,g,b (0..255)"); return; }

  s_strip_mode = STRIP_MODE_STATIC;
  strip.setPixelColor((uint16_t)idx, strip.Color(r, g, b));
  strip.show();

  JsonDocument res; res["ok"] = true; res["index"] = idx; sendJson(200, res);
}

// POST /leds/effect  body: {name:"rainbow|breathe|chase|off", r?,g?,b?, brightness?}
// rainbow ignores r,g,b. breathe/chase use them as the base color (default red).
static void handleLedsEffect() {
  JsonDocument body;
  if (!readJsonBody(body)) { sendErr(400, "invalid json body"); return; }
  String name = body["name"] | "";
  if (name.isEmpty()) { sendErr(400, "need name"); return; }

  uint8_t bright = clampBrightness(body["brightness"] | (int)STRIP_MAX_BRIGHTNESS);
  strip.setBrightness(bright);

  // Optional base color for breathe/chase. Defaults to red so it's never invisible.
  uint8_t r = body["r"] | 255;
  uint8_t g = body["g"] | 0;
  uint8_t b = body["b"] | 0;
  s_strip_base_color = strip.Color(r, g, b);
  s_strip_phase = 0;
  s_strip_last_tick_ms = 0;

  if (name == "rainbow")       s_strip_mode = STRIP_MODE_RAINBOW;
  else if (name == "breathe")  s_strip_mode = STRIP_MODE_BREATHE;
  else if (name == "chase")    s_strip_mode = STRIP_MODE_CHASE;
  else if (name == "off") {
    s_strip_mode = STRIP_MODE_STATIC;
    strip.clear();
    strip.show();
  } else {
    sendErr(400, "name must be rainbow|breathe|chase|off");
    return;
  }

  JsonDocument res;
  res["ok"] = true;
  res["mode"] = name;
  res["brightness"] = bright;
  sendJson(200, res);
}

static void handleNotFound() {
  sendErr(404, "no such route");
}

// ---- outbound event poster ------------------------------------------------

static void postEvent(const char* event, const JsonDocument& extra) {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure secure;
  secure.setInsecure();   // n8n cert is fine; skip verification for now
  HTTPClient http;
  if (!http.begin(secure, N8N_EVENT_URL)) return;
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);

  JsonDocument payload;
  payload["device"] = "stackchan";
  payload["mac"] = WiFi.macAddress();
  payload["uptime_ms"] = (uint32_t)millis();
  payload["event"] = event;
  for (JsonPairConst kv : extra.as<JsonObjectConst>()) {
    payload[kv.key()] = kv.value();
  }
  String body;
  serializeJson(payload, body);
  http.POST(body);
  http.end();
}

// ---- camera streaming (port 81 via ESP-IDF httpd) -------------------------
//
// Why a separate httpd on port 81?
//   The Arduino WebServer is single-threaded — handleClient() processes one
//   request at a time. An MJPEG stream is an open-ended response, so it would
//   block /servo, /face, etc. for the entire stream. esp_http_server (ESP-IDF)
//   runs in its own task, so streaming and control coexist cleanly.

// Encode an RGB565 framebuffer to JPEG. Caller frees out_buf with free().
static bool encode_fb_jpeg(camera_fb_t* fb, uint8_t** out_buf, size_t* out_len) {
  return frame2jpg(fb, JPEG_QUALITY, out_buf, out_len);
}

static esp_err_t cam_capture_handler(httpd_req_t* req) {
  if (!s_camera_ready) { httpd_resp_send_500(req); return ESP_FAIL; }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

  uint8_t* jpg = nullptr; size_t jpg_len = 0;
  bool ok = encode_fb_jpeg(fb, &jpg, &jpg_len);
  esp_camera_fb_return(fb);
  if (!ok || !jpg) { httpd_resp_send_500(req); return ESP_FAIL; }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)jpg, jpg_len);
  free(jpg);
  return res;
}

static esp_err_t cam_stream_handler(httpd_req_t* req) {
  static const char* CT = "multipart/x-mixed-replace;boundary=stackchanframe";
  static const char* PARTHDR = "\r\n--stackchanframe\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  char part_buf[80];

  if (!s_camera_ready) { httpd_resp_send_500(req); return ESP_FAIL; }

  esp_err_t res = httpd_resp_set_type(req, CT);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(20 / portTICK_PERIOD_MS); continue; }

    uint8_t* jpg = nullptr; size_t jpg_len = 0;
    bool ok = encode_fb_jpeg(fb, &jpg, &jpg_len);
    esp_camera_fb_return(fb);
    if (!ok || !jpg) { vTaskDelay(20 / portTICK_PERIOD_MS); continue; }

    int hlen = snprintf(part_buf, sizeof(part_buf), PARTHDR, (unsigned)jpg_len);
    bool send_ok = httpd_resp_send_chunk(req, part_buf, hlen) == ESP_OK
                && httpd_resp_send_chunk(req, (const char*)jpg, jpg_len) == ESP_OK;
    free(jpg);
    if (!send_ok) break;
    // Software JPEG encoding adds ~30ms; loop already runs ~10–15 FPS.
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  return ESP_OK;
}

static bool startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port   = 32769;     // must differ from the default 32768
  config.lru_purge_enable = true;
  config.max_uri_handlers = 4;
  config.max_open_sockets = 4;
  // Stream + JPEG encode runs in this httpd's task; bump stack so an unlucky
  // path through frame2jpg doesn't blow the default 4KB.
  config.stack_size = 8192;
  config.recv_wait_timeout = 5;
  config.send_wait_timeout = 5;

  if (httpd_start(&s_camera_httpd, &config) != ESP_OK) return false;

  httpd_uri_t capture_uri = {
    .uri = "/capture", .method = HTTP_GET,
    .handler = cam_capture_handler, .user_ctx = nullptr
  };
  httpd_uri_t stream_uri = {
    .uri = "/stream", .method = HTTP_GET,
    .handler = cam_stream_handler, .user_ctx = nullptr
  };
  httpd_register_uri_handler(s_camera_httpd, &capture_uri);
  httpd_register_uri_handler(s_camera_httpd, &stream_uri);
  return true;
}

static bool initCamera() {
  // The CoreS3's GC0308 sits on the same I2C bus M5Unified claims at boot.
  // Releasing here lets the camera driver own SCCB during init.
  M5.In_I2C.release();
  esp_err_t err = esp_camera_init(&s_camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  // Default sensor orientation has the StackChan looking at a mirrored world;
  // the host-side tracker assumes a non-mirrored frame, so flip horizontally.
  if (sensor_t* s = esp_camera_sensor_get()) {
    s->set_hmirror(s, 1);
    s->set_vflip(s, 0);
  }
  s_camera_ready = true;
  return true;
}

// ---- setup / loop ---------------------------------------------------------

static void connectWifi() {
  M5StackChan.Display().setTextSize(2);
  M5StackChan.Display().setTextColor(TFT_CYAN);
  M5StackChan.Display().setCursor(0, 0);
  M5StackChan.Display().printf("WiFi: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
    delay(250);
    M5StackChan.Display().print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    M5StackChan.Display().setTextColor(TFT_RED);
    M5StackChan.Display().printf("\nWiFi FAIL\n");
    return;
  }
  localIP = WiFi.localIP().toString();
  M5StackChan.Display().setTextColor(TFT_GREEN);
  M5StackChan.Display().printf("\nIP: %s\n", localIP.c_str());

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", HTTP_PORT);
  }
}

// Wireless flashing. Once running, `arduino-cli upload --port stackchan.local`
// or any espota client (with OTA_PASSWORD) can re-flash without USB.
static void setupOTA() {
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    avatar.setSpeechText("flashing...");
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() {
    avatar.setSpeechText("flashed!");
    Serial.println("OTA: end");
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("OTA error %u\n", err);
    avatar.setSpeechText("OTA fail");
  });
  ArduinoOTA.begin();
}

void setup() {
  Serial.begin(115200);
  M5StackChan.begin();

  // WS2812 strip on Port C — start dark at the safe brightness cap.
  strip.begin();
  strip.setBrightness(STRIP_MAX_BRIGHTNESS);
  strip.clear();
  strip.show();

  connectWifi();
  delay(1500);

  // hand the screen over to Avatar
  avatar.init();
  avatar.setSpeechFont(&fonts::Font2);
  if (!localIP.isEmpty()) {
    String hello = String("hi! ") + localIP;
    avatar.setSpeechText(hello.c_str());
  }

  server.on("/",            HTTP_GET,  handleStatus);
  server.on("/status",      HTTP_GET,  handleStatus);
  server.on("/face",        HTTP_POST, handleFace);
  server.on("/speak",       HTTP_POST, handleSpeak);
  server.on("/servo",       HTTP_POST, handleServo);
  server.on("/servo/home",  HTTP_POST, handleHome);
  server.on("/servo/stop",  HTTP_POST, handleStopServo);
  server.on("/led",         HTTP_POST, handleLed);
  server.on("/leds",        HTTP_POST, handleLeds);
  server.on("/leds/pixel",  HTTP_POST, handleLedsPixel);
  server.on("/leds/effect", HTTP_POST, handleLedsEffect);
  server.on("/play",        HTTP_POST, handlePlay);
  server.on("/reset",       HTTP_POST, handleReset);
  server.onNotFound(handleNotFound);
  server.begin();

  setupOTA();

  Serial.printf("HTTP listening on http://%s:%d\n", localIP.c_str(), HTTP_PORT);

  // Camera comes up last so any failure here doesn't keep control HTTP from
  // serving. /servo, /face, /speak still work even if the camera bricks.
  if (initCamera()) {
    if (startCameraServer()) {
      Serial.printf("Camera HTTP listening on http://%s:81/{stream,capture}\n",
                    localIP.c_str());
    } else {
      Serial.println("Camera HTTPD failed to start");
    }
  } else {
    Serial.println("Camera disabled (init failed)");
  }
}

void loop() {
  M5StackChan.update();
  server.handleClient();
  ArduinoOTA.handle();
  stripTick();

  // Deferred soft-reset (set by /reset handler).
  if (s_reset_at_ms != 0 && (int32_t)(millis() - s_reset_at_ms) >= 0) {
    Serial.println("rebooting via /reset");
    delay(50);
    ESP.restart();
  }

  // expire pending speech bubble (overflow-safe via signed compare)
  if (s_speaking && (int32_t)(millis() - s_speech_clear_at_ms) >= 0) {
    avatar.setSpeechText("");
    avatar.setMouthOpenRatio(0.0f);
    s_speaking = false;
  }

  // touch sensor → n8n
  auto& ts = M5StackChan.TouchSensor;
  if (ts.wasClicked()) {
    JsonDocument extra;
    auto& intens = ts.getIntensities();
    JsonArray arr = extra["intensities"].to<JsonArray>();
    arr.add(intens[0]); arr.add(intens[1]); arr.add(intens[2]);
    postEvent("touch_click", extra);
  }
  if (ts.wasSwipedForward()) {
    JsonDocument extra;
    postEvent("touch_swipe_forward", extra);
  }
  if (ts.wasSwipedBackward()) {
    JsonDocument extra;
    postEvent("touch_swipe_backward", extra);
  }

  delay(10);
}
