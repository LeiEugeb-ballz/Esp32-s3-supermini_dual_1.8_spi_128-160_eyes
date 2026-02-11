#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

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

static WebServer server(80);

static const char *kHtml =
  "<!doctype html><html><body style='font-family:Arial'>"
  "<h2>ESP32 Recovery OTA</h2>"
  "<p>Upload firmware .bin</p>"
  "<form method='POST' action='/update' enctype='multipart/form-data'>"
  "<input type='file' name='firmware' accept='.bin' required>"
  "<input type='submit' value='Update'>"
  "</form></body></html>";

static void setupRoutes() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", kHtml);
  });

  server.on("/status", HTTP_GET, []() {
    String ip = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    String json = "{\"mode\":\"recovery\",\"ip\":\"";
    json += ip;
    json += "\"}";
    server.send(200, "application/json", json);
  });

  server.on("/update", HTTP_POST,
    []() {
      bool ok = !Update.hasError();
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", ok ? "OK, rebooting..." : "Update failed");
      delay(150);
      if (ok) ESP.restart();
    },
    []() {
      HTTPUpload &upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) Update.printError(Serial);
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
      }
      yield();
    }
  );

  server.begin();
}

static void startNetworking() {
  const char *ssid = OTA_WIFI_SSID;
  const char *pass = OTA_WIFI_PASS;

  if (ssid && strlen(ssid) > 0) {
    WiFi.mode(WIFI_STA);

    const char *ipStr = OTA_WIFI_STATIC_IP;
    if (ipStr && strlen(ipStr) > 0) {
      IPAddress ip, gw, mask, dns1, dns2;
      bool okIp = ip.fromString(ipStr);
      bool okGw = gw.fromString((OTA_WIFI_STATIC_GW && strlen(OTA_WIFI_STATIC_GW) > 0) ? OTA_WIFI_STATIC_GW : "0.0.0.0");
      bool okMask = mask.fromString((OTA_WIFI_STATIC_MASK && strlen(OTA_WIFI_STATIC_MASK) > 0) ? OTA_WIFI_STATIC_MASK : "255.255.255.0");
      bool okDns1 = dns1.fromString((OTA_WIFI_STATIC_DNS1 && strlen(OTA_WIFI_STATIC_DNS1) > 0) ? OTA_WIFI_STATIC_DNS1 : "0.0.0.0");
      bool okDns2 = dns2.fromString((OTA_WIFI_STATIC_DNS2 && strlen(OTA_WIFI_STATIC_DNS2) > 0) ? OTA_WIFI_STATIC_DNS2 : "0.0.0.0");
      if (okIp && okGw && okMask && okDns1 && okDns2) {
        WiFi.config(ip, gw, mask, dns1, dns2);
      }
    }

    WiFi.begin(ssid, pass);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
      delay(50);
      yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("RECOVERY OTA STA: %s\n", WiFi.localIP().toString().c_str());
      return;
    }
  }

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(OTA_AP_SSID, OTA_AP_PASS);
  if (ok) {
    Serial.printf("RECOVERY OTA AP: SSID=%s PASS=%s IP=%s\n",
                  OTA_AP_SSID, OTA_AP_PASS, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("RECOVERY OTA AP failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println("Recovery OTA firmware");
  startNetworking();
  setupRoutes();
}

void loop() {
  server.handleClient();
  delay(1);
}
