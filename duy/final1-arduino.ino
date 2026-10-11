#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "esp_camera.h"

// =====================================================
// UART to STM32
// STM32 main.c receives ONE BYTE on USART2:
// '1' forward, '2'-'4' right, '5'-'7' left,
// '8' backward, '0' stop. Do NOT send newline to STM32.
// =====================================================
HardwareSerial STM32Serial(1);
#define STM32_RX 2
#define STM32_TX 1
#define STM32_BAUD 115200

// =====================================================
// Wi-Fi settings (preserved from your current sketch)
// =====================================================
const char* ssid = "JettRevivemeee";
const char* password = "0987654321@";

// =====================================================
// Network ports
// =====================================================
const int CAMERA_PORT = 3333;
const int COMMAND_PORT = 80;
const int DISCOVERY_PORT = 4210;
const char* DISCOVERY_MESSAGE = "DISCOVER_ESP32_CAM";

WiFiServer cameraServer(CAMERA_PORT);
WiFiUDP udp;
WebServer commandServer(COMMAND_PORT);

// =====================================================
// ESP32-S3 WROOM CAM + OV5640 pin map
// =====================================================
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5
#define CAM_PIN_D0     11
#define CAM_PIN_D1     9
#define CAM_PIN_D2     8
#define CAM_PIN_D3     10
#define CAM_PIN_D4     12
#define CAM_PIN_D5     18
#define CAM_PIN_D6     17
#define CAM_PIN_D7     16
#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_PCLK   13

// =====================================================
// Camera initialization
// =====================================================
void initCamera() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;

  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;

  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_pclk     = CAM_PIN_PCLK;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM found.");
    config.frame_size   = FRAMESIZE_QVGA;   // 320x240; test higher only when stable
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM not found.");
    config.frame_size   = FRAMESIZE_QVGA;   // conservative fallback
    config.jpeg_quality = 16;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.print("Camera init failed: 0x");
    Serial.println(err, HEX);
    while (true) {
      delay(100);
    }
  }

  Serial.println("Camera init OK.");
  sensor_t *s = esp_camera_sensor_get();

  if (s) {
    Serial.print("Detected camera PID: 0x");
    Serial.println(s->id.PID, HEX);

    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_lenc(s, 1);
  }
}

// =====================================================
// Camera TCP helpers
// Frame format: "FRAM" + 4-byte little-endian JPEG size + JPEG
// =====================================================
bool writeAll(WiFiClient &client, const uint8_t *data, size_t len) {
  size_t sent = 0;
  while (sent < len) {
    if (!client.connected()) return false;

    size_t n = client.write(data + sent, len - sent);
    if (n == 0) {
      delay(1);
      continue;
    }
    sent += n;
  }
  return true;
}

bool sendFrame(WiFiClient &client) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed.");
    return false;
  }

  if (fb->format != PIXFORMAT_JPEG) {
    Serial.println("Frame is not JPEG.");
    esp_camera_fb_return(fb);
    return false;
  }

  uint32_t len = fb->len;
  bool ok = true;
  ok &= writeAll(client, (const uint8_t*)"FRAM", 4);
  ok &= writeAll(client, (const uint8_t*)&len, 4);
  ok &= writeAll(client, fb->buf, fb->len);

  esp_camera_fb_return(fb);
  return ok;
}

// =====================================================
// UDP discovery
// =====================================================
void handleDiscovery() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  char packet[128];
  int len = udp.read(packet, sizeof(packet) - 1);
  if (len <= 0) return;

  packet[len] = '\0';
  String message(packet);
  message.trim();

  if (message == DISCOVERY_MESSAGE) {
    String response = "ESP32CAM,";
    response += WiFi.localIP().toString();
    response += ",";
    response += String(CAMERA_PORT);

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(response);
    udp.endPacket();
  }
}

// =====================================================
// Wi-Fi connection
// =====================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// =====================================================
// Command mapping: JSON command -> STM32 one-byte command
// This mapping matches main.c exactly.
// =====================================================
String normalizeCommand(String command) {
  command.trim();
  command.toUpperCase();
  command.replace("-", "_");
  command.replace(" ", "_");

  // Common aliases
  if (command == "STRAIGHT" || command == "GO_FORWARD") return "FORWARD";
  if (command == "REVERSE" || command == "BACK") return "BACKWARD";
  if (command == "RIGHT" || command == "TURN_RIGHT") return "RIGHT_MEDIUM";
  if (command == "LEFT" || command == "TURN_LEFT") return "LEFT_MEDIUM";
  if (command == "RIGHT_LIGHT" || command == "RIGHT_SLIGHT") return "RIGHT_SMALL";
  if (command == "RIGHT_MID" || command == "RIGHT_MODERATE") return "RIGHT_MEDIUM";
  if (command == "RIGHT_STRONG" || command == "RIGHT_HARD") return "RIGHT_LARGE";
  if (command == "LEFT_LIGHT" || command == "LEFT_SLIGHT") return "LEFT_SMALL";
  if (command == "LEFT_MID" || command == "LEFT_MODERATE") return "LEFT_MEDIUM";
  if (command == "LEFT_STRONG" || command == "LEFT_HARD") return "LEFT_LARGE";

  return command;
}

char commandToSTM32Code(const String &command) {
  if (command == "FORWARD")       return '1';
  if (command == "RIGHT_SMALL")   return '2';
  if (command == "RIGHT_MEDIUM")  return '3';
  if (command == "RIGHT_LARGE")   return '4';
  if (command == "LEFT_SMALL")    return '5';
  if (command == "LEFT_MEDIUM")   return '6';
  if (command == "LEFT_LARGE")    return '7';
  if (command == "BACKWARD")      return '8';
  if (command == "STOP")          return '0';

  // Allow a raw code for bench testing only.
  if (command.length() == 1 && String("012345678").indexOf(command[0]) >= 0) {
    return command[0];
  }

  return '\0';
}

bool sendSTM32Command(const String &inputCommand) {
  String command = normalizeCommand(inputCommand);
  char code = commandToSTM32Code(command);

  if (code == '\0') {
    Serial.print("[STM32 TX] Rejected invalid command: ");
    Serial.println(inputCommand);
    return false;
  }

  // main.c reads exactly one byte; do NOT use println().
  size_t written = STM32Serial.write((uint8_t)code);

  Serial.print("[STM32 TX] ");
  Serial.print(command);
  Serial.print(" -> '");
  Serial.print(code);
  Serial.println("'");

  return written == 1;
}

// =====================================================
// HTTP POST /command: JSON command from Laptop
// Example {"type":"command","command":"RIGHT_MEDIUM","source":"YOLO"}
// =====================================================
void sendJsonError(int statusCode, const char* message) {
  StaticJsonDocument<160> response;
  response["status"] = "ERROR";
  response["message"] = message;

  String responseText;
  serializeJson(response, responseText);
  commandServer.send(statusCode, "application/json", responseText);
}

void handleCommandPost() {
  String body = commandServer.arg("plain");

  Serial.println();
  Serial.println("[HTTP] POST /command");

  StaticJsonDocument<384> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    sendJsonError(400, "Invalid JSON");
    return;
  }

  const char* commandValue = doc["command"];
  if (commandValue == nullptr) {
    sendJsonError(400, "Missing command");
    return;
  }

  String command = normalizeCommand(String(commandValue));
  char code = commandToSTM32Code(command);

  if (code == '\0') {
    sendJsonError(400, "Invalid command");
    Serial.print("[HTTP] Invalid command: ");
    Serial.println(command);
    return;
  }

  if (!sendSTM32Command(command)) {
    sendJsonError(500, "UART write failed");
    return;
  }

  StaticJsonDocument<192> response;
  response["status"] = "OK";
  response["command"] = command;
  response["stm32_byte"] = String(code);
  response["note"] = "Accepted by ESP32 and written to UART; STM32 execution is not acknowledged.";

  String responseText;
  serializeJson(response, responseText);
  commandServer.send(200, "application/json", responseText);
}

void handleStatusGet() {
  StaticJsonDocument<192> response;
  response["status"] = "OK";
  response["device"] = "ESP32-S3-CAM";
  response["camera_port"] = CAMERA_PORT;
  response["command_endpoint"] = "/command";
  response["stm32_baud"] = STM32_BAUD;

  String responseText;
  serializeJson(response, responseText);
  commandServer.send(200, "application/json", responseText);
}

// Arduino Serial Monitor can be used to test a named command.
void handleSerialCommand() {
  if (!Serial.available()) return;

  String command = normalizeCommand(Serial.readStringUntil('\n'));
  if (command.length() == 0) return;

  if (commandToSTM32Code(command) == '\0') {
    Serial.print("[SERIAL] Invalid command: ");
    Serial.println(command);
    return;
  }

  sendSTM32Command(command);
}

// =====================================================
// setup
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(1000);

  Serial.println();
  Serial.println("===== ESP32-S3 OV5640 CAMERA + STM32 =====");

  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
  Serial.println("UART to STM32 started (115200, 8N1).");

  initCamera();
  connectWiFi();

  cameraServer.begin();
  udp.begin(DISCOVERY_PORT);

  commandServer.on("/command", HTTP_POST, handleCommandPost);
  commandServer.on("/status", HTTP_GET, handleStatusGet);
  commandServer.onNotFound([]() {
    sendJsonError(404, "Endpoint not found");
  });
  commandServer.begin();

  Serial.println("Camera TCP server started on port 3333.");
  Serial.println("HTTP JSON command server started on port 80.");
  Serial.println("POST /command ; GET /status");
  Serial.println("UDP discovery started on port 4210.");
  Serial.println("Ready.");
}

// =====================================================
// loop
// =====================================================
void loop() {
  commandServer.handleClient();
  handleSerialCommand();
  handleDiscovery();

  WiFiClient client = cameraServer.available();

  if (client) {
    Serial.println("Python camera client connected.");

    while (client.connected()) {
      // Service command HTTP requests even while streaming.
      commandServer.handleClient();
      handleSerialCommand();
      handleDiscovery();

      if (!sendFrame(client)) {
        Serial.println("Frame send failed.");
        break;
      }

      delay(100);
    }

    client.stop();
    Serial.println("Python camera client disconnected.");
  }
}
