#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// Select eye texture set by numeric id (Animated_Eyes style):
// 1=default, 2=dragon, 3=cat, 4=doe, 5=goat
#ifndef EYE_STYLE_ID
#define EYE_STYLE_ID 1
#endif

#if EYE_STYLE_ID == 1
#include "defaultEye.h"
#define EYE_STYLE_NAME "default"
#elif EYE_STYLE_ID == 2
#include "dragonEye.h"
#define EYE_STYLE_NAME "dragon"
#elif EYE_STYLE_ID == 3
#include "catEye.h"
#define EYE_STYLE_NAME "cat"
#elif EYE_STYLE_ID == 4
#include "doeEye.h"
#define EYE_STYLE_NAME "doe"
#elif EYE_STYLE_ID == 5
#include "goatEye.h"
#define EYE_STYLE_NAME "goat"
#else
#error Unsupported EYE_STYLE_ID (use 1..5)
#endif

// Confirmed wiring:
// TFT_MOSI=7, TFT_SCLK=6, TFT_DC=5, TFT_RST=4, TFT_CS=-1 (manual CS)
#ifndef CS_LEFT_PIN
#define CS_LEFT_PIN 16
#endif
#ifndef CS_RIGHT_PIN
#define CS_RIGHT_PIN 8
#endif

static constexpr int8_t CS_LEFT  = CS_LEFT_PIN;
static constexpr int8_t CS_RIGHT = CS_RIGHT_PIN;

static constexpr uint8_t LEFT_ROTATION  = 3;
static constexpr uint8_t RIGHT_ROTATION = 1;
static constexpr bool LEFT_INVERT_COLORS  = false;
static constexpr bool RIGHT_INVERT_COLORS = false;

// Mirror toggles. Left X mirror is enabled so left eye mirrors right eye motion.
static constexpr bool LEFT_MIRROR_X = false;
static constexpr bool LEFT_MIRROR_Y = false;
static constexpr bool RIGHT_MIRROR_X = false;
static constexpr bool RIGHT_MIRROR_Y = false;

static constexpr int DISPLAY_WIDTH  = 128;
static constexpr int DISPLAY_HEIGHT = 160;
static constexpr int EYE_Y_OFFSET   = (DISPLAY_HEIGHT - SCREEN_HEIGHT) / 2;

// Eye behavior tuning (matched close to original animated-eyes feel).
// Some eye headers define IRIS_MIN/IRIS_MAX; use those when present.
#ifdef IRIS_MIN
static constexpr int IRIS_MIN_VAL = IRIS_MIN;
#else
static constexpr int IRIS_MIN_VAL = 90;
#endif
#ifdef IRIS_MAX
static constexpr int IRIS_MAX_VAL = IRIS_MAX;
#else
static constexpr int IRIS_MAX_VAL = 130;
#endif
static constexpr int CONVERGENCE_PX = 0;
static constexpr float EYE_POS_SMOOTH = 0.48f;
static constexpr int LEFT_X_OFFSET = 0;
static constexpr int RIGHT_X_OFFSET = 0;
static constexpr int LEFT_Y_OFFSET = 0;
static constexpr int RIGHT_Y_OFFSET = 0;
static constexpr bool LEFT_LID_MIRROR = false;
static constexpr bool RIGHT_LID_MIRROR = true;

static constexpr bool EYELID_TRACKING = true;
// defaultEye/dragonEye table colors are stored in opposite byte order.
static constexpr bool SWAP_PIXEL_BYTES = true;

static constexpr uint16_t PIXEL_BUFFER_SIZE = 1024;
static constexpr bool TRI_STATE_INACTIVE_CS = false;
static constexpr uint16_t CS_SETTLE_US = 12;
static constexpr bool RIGHT_EYE_DEBUG_MARK = true;
static constexpr uint16_t RIGHT_EYE_DEBUG_MARK_COLOR = 0xF800; // Red

static inline bool isRightEyeDebugMarkPixel(uint8_t eyeIndex, int16_t x, int16_t y) {
  if (!RIGHT_EYE_DEBUG_MARK || eyeIndex != 0) return false;

  // Small "popped blood vessel" style mark near the right eye pupil.
  const int16_t cx = SCREEN_WIDTH / 2 + 8;
  const int16_t cy = SCREEN_HEIGHT / 2 - 4;
  const int16_t dx = x - cx;
  const int16_t dy = y - cy;

  const bool coreDot = (dx * dx + dy * dy) <= 2;
  const bool horizontal = (dy == 0 && dx >= -6 && dx <= 3);
  const bool diagUp = (dy == dx && dx >= -2 && dx <= 5);
  const bool diagDown = (dy == -dx && dx >= -1 && dx <= 4);

  return coreDot || horizontal || diagUp || diagDown;
}

TFT_eSPI tft;
static uint16_t pixelBuffer[PIXEL_BUFFER_SIZE];
static WebServer otaServer(80);
static bool otaUploadInProgress = false;
static bool otaEnabled = false;

enum RenderMode : uint8_t {
  RENDER_BOTH = 0,
  RENDER_LEFT_ONLY = 1,
  RENDER_RIGHT_ONLY = 2,
};
static RenderMode renderMode = RENDER_BOTH;

#ifndef CS_DIAG_LOG
#define CS_DIAG_LOG 1
#endif

struct CsDiagState {
  uint32_t selectCount[2] = {0, 0};
  uint32_t drawCount[2] = {0, 0};
  uint8_t lastSelectedEye = 255;
  uint8_t lastCsLeftLevel = 1;
  uint8_t lastCsRightLevel = 1;
  uint32_t lastSelectUs = 0;
  uint32_t lastReportMs = 0;
};

static CsDiagState csDiag;

#ifndef OTA_WIFI_SSID
#define OTA_WIFI_SSID ""
#endif
#ifndef OTA_WIFI_PASS
#define OTA_WIFI_PASS ""
#endif
#ifndef OTA_WIFI_STATIC_IP
#define OTA_WIFI_STATIC_IP ""
#endif
#ifndef OTA_WIFI_STATIC_GW
#define OTA_WIFI_STATIC_GW ""
#endif
#ifndef OTA_WIFI_STATIC_MASK
#define OTA_WIFI_STATIC_MASK "255.255.255.0"
#endif
#ifndef OTA_WIFI_STATIC_DNS1
#define OTA_WIFI_STATIC_DNS1 ""
#endif
#ifndef OTA_WIFI_STATIC_DNS2
#define OTA_WIFI_STATIC_DNS2 ""
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "ESP32-EYES-OTA"
#endif
#ifndef OTA_AP_PASS
#define OTA_AP_PASS "eyesupdate"
#endif

enum BlinkState : uint8_t {
  NOBLINK = 0,
  ENBLINK = 1,
  DEBLINK = 2,
};

struct GazeState {
  int16_t x = 512;
  int16_t y = 512;
  int16_t oldX = 512;
  int16_t oldY = 512;
  int16_t newX = 512;
  int16_t newY = 512;
  bool inMotion = false;
  uint32_t eventStartUs = 0;
  uint32_t eventDurationUs = 250000;
};

struct BlinkTracker {
  BlinkState state = NOBLINK;
  uint32_t startUs = 0;
  uint32_t durationUs = 0;
  uint32_t nextBlinkUs = 0;
};

static GazeState gaze;
static BlinkTracker blink;

static float irisScale = (IRIS_MIN_VAL + IRIS_MAX_VAL) * 0.5f;
static float irisTarget = (IRIS_MIN_VAL + IRIS_MAX_VAL) * 0.5f;
static uint32_t nextIrisMs = 0;

static uint8_t trackedUpper[2] = {128, 128};
static float smoothBaseSx = -1.0f;
static float smoothBaseSy = -1.0f;

static String buildOtaFormHtml() {
  const char *renderModeName =
    (renderMode == RENDER_LEFT_ONLY) ? "left only" :
    (renderMode == RENDER_RIGHT_ONLY) ? "right only" : "both";
  String html;
  html.reserve(2800);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 Eyes OTA</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:620px;margin:20px auto;padding:0 12px;}";
  html += "code{background:#f2f2f2;padding:2px 5px;border-radius:4px;}li{margin:6px 0;}</style>";
  html += "</head><body>";
  html += "<h2>ESP32 Eyes OTA</h2>";
  html += "<p><b>Current style:</b> ";
  html += EYE_STYLE_NAME;
  html += " (";
  html += String((int)EYE_STYLE_ID);
  html += ")</p>";
  html += "<p><b>Render mode:</b> ";
  html += renderModeName;
  html += "</p>";
  html += "<form method='POST' action='/mode'>";
  html += "<label>Draw target:</label> ";
  html += "<select name='m'>";
  html += "<option value='0'";
  if (renderMode == RENDER_BOTH) html += " selected";
  html += ">both</option>";
  html += "<option value='1'";
  if (renderMode == RENDER_LEFT_ONLY) html += " selected";
  html += ">left only</option>";
  html += "<option value='2'";
  if (renderMode == RENDER_RIGHT_ONLY) html += " selected";
  html += ">right only</option>";
  html += "</select> ";
  html += "<input type='submit' value='Apply'>";
  html += "</form>";
  html += "<form method='POST' action='/recover' style='margin-top:8px'>";
  html += "<input type='submit' value='Recover Displays'>";
  html += "</form>";
  html += "<p style='font-size:12px'>Use left/right only to detect CS cross-talk ghosting.</p>";
  html += "<p>Upload a prebuilt firmware <code>.bin</code>:</p>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware' accept='.bin' required>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<hr><p><b>Style mapping (build-time):</b></p><ul>";
  html += "<li>1 = default (<code>esp32s3-eyes</code>)</li>";
  html += "<li>2 = dragon (<code>esp32s3-eyes-2</code>)</li>";
  html += "<li>3 = cat (<code>esp32s3-eyes-3</code>)</li>";
  html += "<li>4 = doe (<code>esp32s3-eyes-4</code>)</li>";
  html += "<li>5 = goat (<code>esp32s3-eyes-5</code>)</li>";
  html += "</ul>";
  html += "<p>Tip: from repo root run ";
  html += "<code>tools\\ota_style_upload.ps1 -Style 3 -Device ";
  html += WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  html += "</code></p>";
  html += "</body></html>";
  return html;
}

static inline int16_t clampi16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static inline uint16_t maybeSwap565(uint16_t c) {
  if (!SWAP_PIXEL_BYTES) return c;
  return (uint16_t)((c >> 8) | (c << 8));
}

static inline void deselectBoth() {
  pinMode(CS_LEFT, OUTPUT);
  pinMode(CS_RIGHT, OUTPUT);
  digitalWrite(CS_LEFT, HIGH);
  digitalWrite(CS_RIGHT, HIGH);
  if (CS_SETTLE_US) delayMicroseconds(CS_SETTLE_US);
}

static inline int8_t eyeCsPin(uint8_t eyeIndex) {
  return (eyeIndex == 0) ? CS_LEFT : CS_RIGHT;
}

static inline void selectEye(uint8_t eyeIndex) {
  // Force a known-good state on both CS lines before selecting one panel.
  pinMode(CS_LEFT, OUTPUT);
  pinMode(CS_RIGHT, OUTPUT);
  digitalWrite(CS_LEFT, HIGH);
  digitalWrite(CS_RIGHT, HIGH);
  if (CS_SETTLE_US) delayMicroseconds(CS_SETTLE_US);

  const int8_t active = eyeCsPin(eyeIndex);
  const int8_t inactive = (eyeIndex == 0) ? CS_RIGHT : CS_LEFT;

  // Keep inactive panel unselected before changing active CS.
  if (TRI_STATE_INACTIVE_CS) {
    pinMode(inactive, INPUT_PULLUP);
  } else {
    pinMode(inactive, OUTPUT);
    digitalWrite(inactive, HIGH);
  }

  pinMode(active, OUTPUT);
  digitalWrite(active, HIGH);
  if (CS_SETTLE_US) delayMicroseconds(CS_SETTLE_US);
  digitalWrite(active, LOW);
  if (CS_SETTLE_US) delayMicroseconds(CS_SETTLE_US);

#if CS_DIAG_LOG
  csDiag.selectCount[eyeIndex]++;
  csDiag.lastSelectedEye = eyeIndex;
  csDiag.lastSelectUs = micros();
  csDiag.lastCsLeftLevel = (uint8_t)digitalRead(CS_LEFT);
  csDiag.lastCsRightLevel = (uint8_t)digitalRead(CS_RIGHT);
#endif
}

static void reportCsDiagIfDue() {
#if CS_DIAG_LOG
  const uint32_t nowMs = millis();
  if ((nowMs - csDiag.lastReportMs) < 1000U) return;
  csDiag.lastReportMs = nowMs;

  static uint32_t prevSelectL = 0;
  static uint32_t prevSelectR = 0;
  static uint32_t prevDrawL = 0;
  static uint32_t prevDrawR = 0;

  const uint32_t dSelectL = csDiag.selectCount[0] - prevSelectL;
  const uint32_t dSelectR = csDiag.selectCount[1] - prevSelectR;
  const uint32_t dDrawL = csDiag.drawCount[0] - prevDrawL;
  const uint32_t dDrawR = csDiag.drawCount[1] - prevDrawR;

  prevSelectL = csDiag.selectCount[0];
  prevSelectR = csDiag.selectCount[1];
  prevDrawL = csDiag.drawCount[0];
  prevDrawR = csDiag.drawCount[1];

  Serial.printf(
    "CS-DIAG mode=%u select/s(L,R)=%lu,%lu draw/s(L,R)=%lu,%lu levels(L,R)=%u,%u lastEye=%u lastUs=%lu\n",
    (unsigned)renderMode,
    (unsigned long)dSelectL, (unsigned long)dSelectR,
    (unsigned long)dDrawL, (unsigned long)dDrawR,
    (unsigned)csDiag.lastCsLeftLevel, (unsigned)csDiag.lastCsRightLevel,
    (unsigned)csDiag.lastSelectedEye, (unsigned long)csDiag.lastSelectUs
  );
#endif
}

static void configureDisplay(uint8_t eyeIndex, uint8_t rotation, bool invertColors) {
  selectEye(eyeIndex);
  tft.setRotation(rotation & 3U);
  tft.invertDisplay(invertColors);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(eyeCsPin(eyeIndex), HIGH);
}

static void reinitDisplays(const char *reason) {
  Serial.printf("Reinitializing displays (%s)\n", reason ? reason : "unknown");

  // TFT_CS is disabled in TFT_eSPI, so both displays are selected for init.
  digitalWrite(CS_LEFT, LOW);
  digitalWrite(CS_RIGHT, LOW);
  tft.init();
  tft.fillScreen(TFT_BLACK);
  deselectBoth();

  configureDisplay(0, LEFT_ROTATION, LEFT_INVERT_COLORS);
  configureDisplay(1, RIGHT_ROTATION, RIGHT_INVERT_COLORS);
}

static void setupOtaRoutes() {
  otaServer.on("/", HTTP_GET, []() {
    otaServer.send(200, "text/html", buildOtaFormHtml());
  });

  otaServer.on("/status", HTTP_GET, []() {
    String ip = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    String json = "{\"style_id\":";
    json += String((int)EYE_STYLE_ID);
    json += ",\"style_name\":\"";
    json += EYE_STYLE_NAME;
    json += "\",\"render_mode\":";
    json += String((int)renderMode);
    json += ",\"diag\":";
    json += String((int)CS_DIAG_LOG);
    json += ",\"sel_l\":";
    json += String((unsigned long)csDiag.selectCount[0]);
    json += ",\"sel_r\":";
    json += String((unsigned long)csDiag.selectCount[1]);
    json += ",\"draw_l\":";
    json += String((unsigned long)csDiag.drawCount[0]);
    json += ",\"draw_r\":";
    json += String((unsigned long)csDiag.drawCount[1]);
    json += ",\"ip\":\"";
    json += ip;
    json += "\"}";
    otaServer.send(200, "application/json", json);
  });

  otaServer.on("/mode", HTTP_POST, []() {
    if (!otaServer.hasArg("m")) {
      otaServer.send(400, "text/plain", "Missing mode");
      return;
    }
    int m = otaServer.arg("m").toInt();
    if (m < 0 || m > 2) {
      otaServer.send(400, "text/plain", "Mode must be 0,1,2");
      return;
    }
    renderMode = (RenderMode)m;
    Serial.printf("Render mode changed to %d\n", m);
    reinitDisplays("mode-change");
    trackedUpper[0] = 128;
    trackedUpper[1] = 128;
    otaServer.sendHeader("Location", "/");
    otaServer.send(303, "text/plain", "OK");
  });

  otaServer.on("/recover", HTTP_POST, []() {
    reinitDisplays("manual-recover");
    trackedUpper[0] = 128;
    trackedUpper[1] = 128;
    otaServer.sendHeader("Location", "/");
    otaServer.send(303, "text/plain", "Recovered");
  });

  otaServer.on("/update", HTTP_POST,
    []() {
      bool ok = !Update.hasError();
      otaServer.sendHeader("Connection", "close");
      otaServer.send(200, "text/plain", ok ? "OK, rebooting..." : "Update failed");
      delay(120);
      if (ok) ESP.restart();
    },
    []() {
      HTTPUpload &upload = otaServer.upload();
      if (upload.status == UPLOAD_FILE_START) {
        otaUploadInProgress = true;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
          Update.printError(Serial);
        }
        otaUploadInProgress = false;
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        otaUploadInProgress = false;
      }
      yield();
    }
  );

  otaServer.onNotFound([]() {
    otaServer.send(404, "text/plain", "Not found");
  });

  otaServer.begin();
  otaEnabled = true;
}

static void initOtaNetworking() {
  const char *staSsid = OTA_WIFI_SSID;
  const char *staPass = OTA_WIFI_PASS;
  const char *staIpStr = OTA_WIFI_STATIC_IP;
  const char *staGwStr = OTA_WIFI_STATIC_GW;
  const char *staMaskStr = OTA_WIFI_STATIC_MASK;
  const char *staDns1Str = OTA_WIFI_STATIC_DNS1;
  const char *staDns2Str = OTA_WIFI_STATIC_DNS2;

  if (staSsid && strlen(staSsid) > 0) {
    WiFi.mode(WIFI_STA);

    if (staIpStr && strlen(staIpStr) > 0) {
      IPAddress ip, gw, mask, dns1, dns2;
      bool okIp = ip.fromString(staIpStr);
      bool okGw = gw.fromString((staGwStr && strlen(staGwStr) > 0) ? staGwStr : "0.0.0.0");
      bool okMask = mask.fromString((staMaskStr && strlen(staMaskStr) > 0) ? staMaskStr : "255.255.255.0");
      bool okDns1 = dns1.fromString((staDns1Str && strlen(staDns1Str) > 0) ? staDns1Str : "0.0.0.0");
      bool okDns2 = dns2.fromString((staDns2Str && strlen(staDns2Str) > 0) ? staDns2Str : "0.0.0.0");

      if (okIp && okGw && okMask && okDns1 && okDns2) {
        if (!WiFi.config(ip, gw, mask, dns1, dns2)) {
          Serial.println("OTA STA static IP config failed, using DHCP");
        } else {
          Serial.printf("OTA STA static IP request: %s\n", ip.toString().c_str());
        }
      } else {
        Serial.println("OTA STA static IP parse failed, using DHCP");
      }
    }

    WiFi.begin(staSsid, staPass);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
      delay(50);
      yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("OTA STA connected: %s\n", WiFi.localIP().toString().c_str());
      setupOtaRoutes();
      return;
    }
  }

  // AP fallback so OTA still works without config or router.
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS);
  if (apOk) {
    Serial.printf("OTA AP started: SSID=%s PASS=%s IP=%s\n",
                  OTA_AP_SSID, OTA_AP_PASS, WiFi.softAPIP().toString().c_str());
    setupOtaRoutes();
  } else {
    Serial.println("OTA AP failed");
  }
}

static void fillDisplayColor(uint8_t eyeIndex, uint16_t color) {
  selectEye(eyeIndex);
  tft.fillScreen(color);
  digitalWrite(eyeCsPin(eyeIndex), HIGH);
}

static void updateGazeMotion() {
  const uint32_t t = micros();
  if (gaze.eventStartUs == 0) {
    gaze.eventStartUs = t;
    gaze.eventDurationUs = 200000;
  }

  const uint32_t dt = t - gaze.eventStartUs;

  if (gaze.inMotion) {
    if (dt >= gaze.eventDurationUs) {
      gaze.inMotion = false;
      gaze.eventStartUs = t;
      // Hold durations: short enough to keep eyes lively.
      gaze.eventDurationUs = (uint32_t)random(120000, 950000);
      gaze.x = gaze.oldX = gaze.newX;
      gaze.y = gaze.oldY = gaze.newY;
    } else {
      float u = (float)dt / (float)gaze.eventDurationUs;
      if (u < 0.0f) u = 0.0f;
      if (u > 1.0f) u = 1.0f;
      float e = u * u * (3.0f - 2.0f * u); // Smoothstep ease curve.
      gaze.x = (int16_t)(gaze.oldX + (int32_t)((gaze.newX - gaze.oldX) * e));
      gaze.y = (int16_t)(gaze.oldY + (int32_t)((gaze.newY - gaze.oldY) * e));
    }
  } else {
    gaze.x = gaze.oldX;
    gaze.y = gaze.oldY;

    if (dt >= gaze.eventDurationUs) {
      int32_t dx;
      int32_t dy;
      do {
        gaze.newX = (int16_t)random(1024);
        gaze.newY = (int16_t)random(1024);
        dx = (gaze.newX * 2) - 1023;
        dy = (gaze.newY * 2) - 1023;
      } while ((dx * dx + dy * dy) > (1023L * 1023L));

      gaze.eventDurationUs = (uint32_t)random(34000, 90000); // Fast saccades.
      gaze.eventStartUs = t;
      gaze.inMotion = true;
    }
  }

  // Tiny micro-motion to avoid robotic stillness.
  const uint32_t ms = millis();
  gaze.x = clampi16(gaze.x + (int16_t)(3.0f * sinf((float)ms * 0.0063f)), 0, 1023);
  gaze.y = clampi16(gaze.y + (int16_t)(2.0f * sinf((float)ms * 0.0048f + 1.2f)), 0, 1023);
}

static void updateBlink() {
  const uint32_t t = micros();

  if (blink.state == NOBLINK) {
    if ((int32_t)(t - blink.nextBlinkUs) >= 0) {
      blink.state = ENBLINK;
      blink.startUs = t;
      blink.durationUs = (uint32_t)random(28000, 52000);
    }
    return;
  }

  if ((t - blink.startUs) >= blink.durationUs) {
    if (blink.state == ENBLINK) {
      blink.state = DEBLINK;
      blink.startUs = t;
      blink.durationUs *= 2U;
    } else {
      blink.state = NOBLINK;
      blink.nextBlinkUs = t + (uint32_t)random(700000, 4200000);
    }
  }
}

static void updateIris() {
  const uint32_t nowMs = millis();

  if (nowMs >= nextIrisMs) {
    irisTarget = (float)random(IRIS_MIN_VAL, IRIS_MAX_VAL + 1);
    nextIrisMs = nowMs + (uint32_t)random(450, 1800);
  }

  irisScale += (irisTarget - irisScale) * 0.16f;
  if (irisScale < IRIS_MIN_VAL) irisScale = IRIS_MIN_VAL;
  if (irisScale > IRIS_MAX_VAL) irisScale = IRIS_MAX_VAL;
}

static void computeEyelidThresholds(
  uint8_t eyeIndex,
  int16_t scleraX,
  int16_t scleraY,
  uint8_t &upperThreshold,
  uint8_t &lowerThreshold) {

  if (EYELID_TRACKING) {
    int16_t sampleX = SCLERA_WIDTH / 2 - (scleraX / 2);
    int16_t sampleY = SCLERA_HEIGHT / 2 - (scleraY + IRIS_HEIGHT / 4);

    uint8_t n = 0;
    if (sampleY >= 0 && sampleY < SCREEN_HEIGHT) {
      sampleX = clampi16(sampleX, 0, SCREEN_WIDTH - 1);
      int16_t sampleX2 = SCREEN_WIDTH - 1 - sampleX;
      uint8_t a = pgm_read_byte(upper + sampleY * SCREEN_WIDTH + sampleX);
      uint8_t b = pgm_read_byte(upper + sampleY * SCREEN_WIDTH + sampleX2);
      n = (uint8_t)((a + b) / 2);
    }

    trackedUpper[eyeIndex] = (uint8_t)((trackedUpper[eyeIndex] * 3 + n) / 4);
    upperThreshold = trackedUpper[eyeIndex];
    lowerThreshold = (uint8_t)(254 - upperThreshold);
  } else {
    upperThreshold = 0;
    lowerThreshold = 0;
  }

  if (blink.state != NOBLINK) {
    uint32_t elapsed = micros() - blink.startUs;
    uint32_t s = (elapsed >= blink.durationUs) ? 255U : (255U * elapsed) / blink.durationUs;
    s = (blink.state == DEBLINK) ? (1U + s) : (256U - s);

    upperThreshold = (uint8_t)((upperThreshold * s + 254U * (257U - s)) / 256U);
    lowerThreshold = (uint8_t)((lowerThreshold * s + 254U * (257U - s)) / 256U);
  }
}

static void drawOneEye(
  uint8_t eyeIndex,
  uint16_t irisScaleNow,
  int16_t scleraX,
  int16_t scleraY,
  uint8_t upperThreshold,
  uint8_t lowerThreshold) {

  const int8_t cs = eyeCsPin(eyeIndex);
  const int16_t irisYOffset = (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;
  const int16_t irisXOffset = (SCLERA_WIDTH - IRIS_WIDTH) / 2;

  selectEye(eyeIndex);
  tft.startWrite();
  tft.setAddrWindow(0, EYE_Y_OFFSET, SCREEN_WIDTH, SCREEN_HEIGHT);

  uint32_t pixels = 0;
  int16_t scleraYRow = scleraY;
  int16_t irisYRow = scleraY - irisYOffset;
  bool lidMirror = (eyeIndex == 0) ? LEFT_LID_MIRROR : RIGHT_LID_MIRROR;
  int16_t lidXStart = lidMirror ? 0 : (SCREEN_WIDTH - 1);
  int8_t dlid = lidMirror ? 1 : -1;

  for (int16_t y = 0; y < SCREEN_HEIGHT; ++y, ++scleraYRow, ++irisYRow) {
    int16_t scleraXCol = scleraX;
    int16_t irisXCol = scleraX - irisXOffset;
    int16_t lidX = lidXStart;

    for (int16_t x = 0; x < SCREEN_WIDTH; ++x, ++scleraXCol, ++irisXCol, lidX += dlid) {
      uint16_t p;

      if ((pgm_read_byte(lower + y * SCREEN_WIDTH + lidX) <= lowerThreshold) ||
          (pgm_read_byte(upper + y * SCREEN_WIDTH + lidX) <= upperThreshold)) {
        p = 0x0000;
      } else if ((irisYRow < 0) || (irisYRow >= IRIS_HEIGHT) ||
                 (irisXCol < 0) || (irisXCol >= IRIS_WIDTH)) {
        p = pgm_read_word(sclera + scleraYRow * SCLERA_WIDTH + scleraXCol);
      } else {
        uint16_t pv = pgm_read_word(polar + irisYRow * IRIS_WIDTH + irisXCol);
        uint32_t d = (irisScaleNow * (pv & 0x7F)) / 128;
        if (d < IRIS_MAP_HEIGHT) {
          uint32_t a = (IRIS_MAP_WIDTH * (pv >> 7)) / 512;
          p = pgm_read_word(iris + d * IRIS_MAP_WIDTH + a);
        } else {
          p = pgm_read_word(sclera + scleraYRow * SCLERA_WIDTH + scleraXCol);
        }
      }

      if (isRightEyeDebugMarkPixel(eyeIndex, x, y)) {
        p = RIGHT_EYE_DEBUG_MARK_COLOR;
      }

      pixelBuffer[pixels++] = maybeSwap565(p);

      if (pixels >= PIXEL_BUFFER_SIZE) {
        tft.pushColors(pixelBuffer, pixels, false);
        pixels = 0;
        yield();
      }
    }
  }

  if (pixels > 0) {
    tft.pushColors(pixelBuffer, pixels, false);
  }

  tft.endWrite();
  digitalWrite(cs, HIGH);
#if CS_DIAG_LOG
  csDiag.drawCount[eyeIndex]++;
#endif
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("Dual-eye texture renderer startup");
  Serial.printf("Pins: MOSI=%d SCLK=%d DC=%d RST=%d CS_L=%d CS_R=%d\n",
                TFT_MOSI, TFT_SCLK, TFT_DC, TFT_RST, CS_LEFT, CS_RIGHT);
  Serial.printf("Rot: L=%u R=%u mirror(Lx,Ly,Rx,Ry)=(%d,%d,%d,%d) eyeStyle=%s\n",
                LEFT_ROTATION, RIGHT_ROTATION,
                LEFT_MIRROR_X, LEFT_MIRROR_Y, RIGHT_MIRROR_X, RIGHT_MIRROR_Y,
                EYE_STYLE_NAME);
#if CS_DIAG_LOG
  Serial.println("CS diagnostics: ENABLED (1 line/sec)");
#else
  Serial.println("CS diagnostics: DISABLED");
#endif

  randomSeed((uint32_t)esp_random());

  pinMode(CS_LEFT, OUTPUT);
  pinMode(CS_RIGHT, OUTPUT);
  deselectBoth();

  reinitDisplays("boot");

  // Quick sanity color test so channel issues show immediately.
  fillDisplayColor(0, TFT_RED);
  fillDisplayColor(1, TFT_RED);
  delay(120);
  fillDisplayColor(0, TFT_GREEN);
  fillDisplayColor(1, TFT_GREEN);
  delay(120);
  fillDisplayColor(0, TFT_BLUE);
  fillDisplayColor(1, TFT_BLUE);
  delay(120);
  fillDisplayColor(0, TFT_BLACK);
  fillDisplayColor(1, TFT_BLACK);

  gaze.eventStartUs = micros();
  gaze.eventDurationUs = 180000;
  blink.nextBlinkUs = micros() + 900000;
  nextIrisMs = millis() + 500;

  initOtaNetworking();
  if (otaEnabled) {
    Serial.println("OTA web updater ready at http://<device-ip>/");
  }
}

void loop() {
  if (otaEnabled) otaServer.handleClient();
  if (otaUploadInProgress) {
    delay(1);
    return;
  }

  updateGazeMotion();
  updateBlink();
  updateIris();

  // Smooth one shared eye position first, then derive mirrored/calibrated panel positions.
  int16_t baseSxRaw = map(gaze.x, 0, 1023, 0, SCLERA_WIDTH - SCREEN_WIDTH);
  int16_t baseSyRaw = map(gaze.y, 0, 1023, 0, SCLERA_HEIGHT - SCREEN_HEIGHT);
  if (smoothBaseSx < 0.0f) {
    smoothBaseSx = (float)baseSxRaw;
    smoothBaseSy = (float)baseSyRaw;
  } else {
    smoothBaseSx += ((float)baseSxRaw - smoothBaseSx) * EYE_POS_SMOOTH;
    smoothBaseSy += ((float)baseSyRaw - smoothBaseSy) * EYE_POS_SMOOTH;
  }
  int16_t baseSx = clampi16((int16_t)(smoothBaseSx + 0.5f), 0, SCLERA_WIDTH - SCREEN_WIDTH);
  int16_t baseSy = clampi16((int16_t)(smoothBaseSy + 0.5f), 0, SCLERA_HEIGHT - SCREEN_HEIGHT);

  // Alternate draw order each frame so one panel is not always second.
  static bool drawRightFirst = false;
  uint8_t firstEye = drawRightFirst ? 1 : 0;
  uint8_t secondEye = firstEye ^ 1;
  drawRightFirst = !drawRightFirst;

  for (uint8_t pass = 0; pass < 2; ++pass) {
    uint8_t eyeIndex = (pass == 0) ? firstEye : secondEye;
    if (renderMode == RENDER_LEFT_ONLY && eyeIndex != 0) continue;
    if (renderMode == RENDER_RIGHT_ONLY && eyeIndex != 1) continue;

    int16_t sx = baseSx;
    int16_t sy = baseSy;

    if (eyeIndex == 0 && LEFT_MIRROR_X) sx = (SCLERA_WIDTH - SCREEN_WIDTH) - sx;
    if (eyeIndex == 0 && LEFT_MIRROR_Y) sy = (SCLERA_HEIGHT - SCREEN_HEIGHT) - sy;
    if (eyeIndex == 1 && RIGHT_MIRROR_X) sx = (SCLERA_WIDTH - SCREEN_WIDTH) - sx;
    if (eyeIndex == 1 && RIGHT_MIRROR_Y) sy = (SCLERA_HEIGHT - SCREEN_HEIGHT) - sy;

    // Eye pair convergence and final calibration offsets.
    sx += (eyeIndex == 0) ? -CONVERGENCE_PX : CONVERGENCE_PX;
    sx += (eyeIndex == 0) ? LEFT_X_OFFSET : RIGHT_X_OFFSET;
    sy += (eyeIndex == 0) ? LEFT_Y_OFFSET : RIGHT_Y_OFFSET;
    sx = clampi16(sx, 0, SCLERA_WIDTH - SCREEN_WIDTH);
    sy = clampi16(sy, 0, SCLERA_HEIGHT - SCREEN_HEIGHT);

    uint8_t uT = 0;
    uint8_t lT = 0;
    computeEyelidThresholds(eyeIndex, sx, sy, uT, lT);

    drawOneEye(eyeIndex, (uint16_t)irisScale, sx, sy, uT, lT);
  }

  reportCsDiagIfDue();
}
