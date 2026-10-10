/*
  Wayfinder rover: Wi-Fi hotspot seeker with a fixed forward HC-SR04.

  Board: Arduino UNO R4 WiFi
  Motor driver: L298N
    ENA D8, IN1 D9, IN2 D10  (left motor)
    ENB D13, IN3 D11, IN4 D12 (right motor)
  Fixed HC-SR04: TRIG D6, ECHO D7

  Copy secrets.h.example to secrets.h and set the hotspot credentials and the
  laptop's IP address. The laptop and rover must join the same hotspot.

  Safety: a missing or invalid sonar reading is UNKNOWN, never clear space.
  The rover will not drive forward without a valid front reading above SAFE_CM.
  There are no wheel encoders or IMU, so this firmware does not claim to know
  its absolute position or build a physical map.
*/
#include <Arduino.h>
#include <Arduino_LED_Matrix.h>
#include <WiFiS3.h>
#include <math.h>

#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #define WAYFINDER_WIFI_SSID "CHANGE_ME"
  #define WAYFINDER_WIFI_PASSWORD "CHANGE_ME"
  #define WAYFINDER_BACKEND_HOST "192.168.1.100"
  #define WAYFINDER_BACKEND_PORT 8000
#endif

// These values are deliberately separate from the hotspot credentials. They
// can be tuned after an on-site one-metre calibration (Serial command: c).
#ifndef WAYFINDER_RSSI_AT_ONE_METRE
  #define WAYFINDER_RSSI_AT_ONE_METRE -45.0f
#endif
#ifndef WAYFINDER_PATH_LOSS_N
  #define WAYFINDER_PATH_LOSS_N 2.5f
#endif
#ifndef WAYFINDER_RSSI_CALIBRATED
  #define WAYFINDER_RSSI_CALIBRATED 0
#endif

constexpr uint8_t ENA = 8, IN1 = 9, IN2 = 10;
constexpr uint8_t ENB = 13, IN3 = 11, IN4 = 12;
constexpr uint8_t TRIG_PIN = 6, ECHO_PIN = 7;

constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;
constexpr bool SWAP_SIDES = false;

constexpr float SAFE_CM = 22.0f;
constexpr float TARGET_RSSI_DBM = -50.0f;
constexpr float RSSI_IMPROVEMENT_DB = 1.5f;
constexpr uint32_t STEP_MS = 340;
constexpr uint32_t BACK_MS = 360;
constexpr uint32_t TURN_MS = 260;  // Tune on the floor: intended to be about 30 degrees.
constexpr uint32_t AVOID_TURN_MS = 520; // Tune on the floor: intended to be about 70-90 degrees.
constexpr uint32_t TELEMETRY_MS = 700;
constexpr uint32_t COMMAND_MS = 850;
constexpr uint32_t WIFI_RETRY_MS = 8000;
constexpr uint32_t RSSI_SAMPLE_MS = 350;
constexpr uint32_t RSSI_CALIBRATION_MS = 10000;
constexpr float RSSI_FILTER_ALPHA = 0.20f;

ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {};

enum RoverMode : uint8_t { IDLE, MANUAL, WIFI_HUNT, AVOIDING, TARGET_NEARBY, WAITING_WIFI, BLOCKED };
enum DriveKind : uint8_t { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
enum HuntStage : uint8_t { HUNT_BASELINE, HUNT_AFTER_STEP, HUNT_AFTER_BACK, HUNT_AFTER_TURN };

RoverMode mode = IDLE;
DriveKind activeDrive = DRIVE_STOP;
HuntStage huntStage = HUNT_BASELINE;
uint32_t driveEndsAt = 0;
uint32_t lastTelemetryAt = 0;
uint32_t lastCommandAt = 0;
uint32_t lastWiFiAttemptAt = 0;
uint32_t lastRssiSampleAt = 0;
float baselineRssi = -127.0f;
float lastRssi = -127.0f;
float filteredRssi = NAN;
float rssiAtOneMetre = WAYFINDER_RSSI_AT_ONE_METRE;
float calibrationSum = 0.0f;
uint16_t calibrationSamples = 0;
uint32_t calibrationStartedAt = 0;
bool rssiAvailable = false;
bool calibrationActive = false;
bool rssiCalibrated = WAYFINDER_RSSI_CALIBRATED;
bool turnRightNext = true;
String lastPlanAction = "";

const char* modeName() {
  switch (mode) {
    case IDLE: return "IDLE";
    case MANUAL: return "MANUAL";
    case WIFI_HUNT: return "WIFI_HUNT";
    case AVOIDING: return "AVOIDING";
    case TARGET_NEARBY: return "TARGET_NEARBY";
    case WAITING_WIFI: return "WAITING_WIFI";
    case BLOCKED: return "BLOCKED";
  }
  return "IDLE";
}

void showSymbol(DriveKind kind) {
  memset(frame, 0, sizeof(frame));
  if (kind == DRIVE_STOP) {
    for (int i = 2; i < 10; ++i) frame[2][i] = frame[5][i] = 1;
    for (int i = 2; i < 6; ++i) frame[i][2] = frame[i][9] = 1;
  } else if (kind == DRIVE_FORWARD) {
    for (int y = 1; y < 7; ++y) frame[y][5] = frame[y][6] = 1;
    frame[0][5] = frame[0][6] = 1; frame[1][4] = frame[1][7] = 1;
  } else if (kind == DRIVE_BACKWARD) {
    for (int y = 1; y < 7; ++y) frame[y][5] = frame[y][6] = 1;
    frame[7][5] = frame[7][6] = 1; frame[6][4] = frame[6][7] = 1;
  } else {
    const bool right = kind == DRIVE_RIGHT;
    for (int x = 2; x < 10; ++x) frame[3][x] = frame[4][x] = 1;
    const int tip = right ? 10 : 1;
    const int base = right ? 8 : 3;
    frame[2][base] = frame[5][base] = 1;
    frame[3][tip] = frame[4][tip] = 1;
  }
  matrix.renderBitmap(frame, 8, 12);
}

void setMotor(uint8_t enable, uint8_t forwardPin, uint8_t reversePin, int direction) {
  if (direction > 0) {
    digitalWrite(forwardPin, HIGH); digitalWrite(reversePin, LOW); digitalWrite(enable, HIGH);
  } else if (direction < 0) {
    digitalWrite(forwardPin, LOW); digitalWrite(reversePin, HIGH); digitalWrite(enable, HIGH);
  } else {
    digitalWrite(forwardPin, LOW); digitalWrite(reversePin, LOW); digitalWrite(enable, LOW);
  }
}

void driveRaw(int left, int right) {
  if (SWAP_SIDES) { int swap = left; left = right; right = swap; }
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
  setMotor(ENA, IN1, IN2, left);
  setMotor(ENB, IN3, IN4, right);
}

void stopDrive() {
  driveRaw(0, 0);
  activeDrive = DRIVE_STOP;
  driveEndsAt = 0;
  showSymbol(DRIVE_STOP);
}

void startDrive(DriveKind kind, uint32_t durationMs) {
  int left = 0, right = 0;
  if (kind == DRIVE_FORWARD) { left = 1; right = 1; }
  if (kind == DRIVE_BACKWARD) { left = -1; right = -1; }
  if (kind == DRIVE_LEFT) { left = -1; right = 1; }
  if (kind == DRIVE_RIGHT) { left = 1; right = -1; }
  driveRaw(left, right);
  activeDrive = kind;
  driveEndsAt = millis() + durationMs;
  showSymbol(kind);
}

bool driveFinished() {
  if (activeDrive == DRIVE_STOP) return true;
  if (static_cast<int32_t>(millis() - driveEndsAt) >= 0) {
    stopDrive();
    return true;
  }
  return false;
}

float readDistanceCm() {
  float readings[3];
  uint8_t count = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    const uint32_t width = pulseIn(ECHO_PIN, HIGH, 25000);
    const float value = width ? width * 0.0343f / 2.0f : -1.0f;
    if (value >= 3.0f && value <= 350.0f) readings[count++] = value;
    delay(8);
  }
  if (!count) return -1.0f;
  for (uint8_t i = 0; i + 1 < count; ++i)
    for (uint8_t j = i + 1; j < count; ++j)
      if (readings[j] < readings[i]) { const float tmp = readings[i]; readings[i] = readings[j]; readings[j] = tmp; }
  return readings[count / 2];
}

bool forwardClear(float frontCm) { return frontCm >= SAFE_CM; }

bool validRssi(int value) {
  return value >= -127 && value <= -1;
}

void finishCalibration() {
  calibrationActive = false;
  if (calibrationSamples < 5) {
    Serial.println("RSSI calibration failed: not enough valid samples.");
    return;
  }
  rssiAtOneMetre = calibrationSum / calibrationSamples;
  rssiCalibrated = true;
  Serial.print("RSSI calibration complete. RSSI at 1 m: ");
  Serial.println(rssiAtOneMetre, 1);
  Serial.println("Copy that number into WAYFINDER_RSSI_AT_ONE_METRE in secrets.h if you want it after reboot.");
}

void updateRssi(bool force = false) {
  if (WiFi.status() != WL_CONNECTED) {
    rssiAvailable = false;
    filteredRssi = NAN;
    calibrationActive = false;
    return;
  }
  if (!force && millis() - lastRssiSampleAt < RSSI_SAMPLE_MS) return;
  lastRssiSampleAt = millis();
  const int raw = WiFi.RSSI();
  if (!validRssi(raw)) return;

  if (!rssiAvailable) filteredRssi = static_cast<float>(raw);
  else filteredRssi = RSSI_FILTER_ALPHA * raw + (1.0f - RSSI_FILTER_ALPHA) * filteredRssi;
  rssiAvailable = true;

  if (calibrationActive) {
    calibrationSum += raw;
    ++calibrationSamples;
    if (millis() - calibrationStartedAt >= RSSI_CALIBRATION_MS) finishCalibration();
  }
}

float currentRssi() {
  return rssiAvailable ? filteredRssi : -127.0f;
}

float roughWifiDistanceMetres() {
  if (!rssiAvailable || !rssiCalibrated) return -1.0f;
  return powf(10.0f, (rssiAtOneMetre - filteredRssi) / (10.0f * WAYFINDER_PATH_LOSS_N));
}

float targetRssiThreshold() {
  // After calibration, stop about 1.5-2 m from the phone rather than relying
  // on an arbitrary indoor RSSI value. Without calibration, use a conservative
  // default that must be tuned on site.
  return rssiCalibrated ? rssiAtOneMetre - 6.0f : TARGET_RSSI_DBM;
}

void startCalibration() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("RSSI calibration needs a Wi-Fi connection.");
    return;
  }
  calibrationSum = 0.0f;
  calibrationSamples = 0;
  calibrationStartedAt = millis();
  calibrationActive = true;
  Serial.println("RSSI calibration started. Keep rover exactly 1 m from the hotspot for 10 seconds.");
}

void printStatus() {
  Serial.print("Wi-Fi: ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "connected" : "waiting");
  Serial.print(" | RSSI: ");
  if (rssiAvailable) Serial.print(filteredRssi, 1); else Serial.print("n/a");
  Serial.print(" dBm | mode: ");
  Serial.println(modeName());
}

void handleSerial() {
  while (Serial.available()) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C') startCalibration();
    if (command == 's' || command == 'S') printStatus();
  }
}

const char* signalTrend(float rssi) {
  if (rssi <= -126 || lastRssi <= -126) return "COLLECTING";
  if (rssi > lastRssi + RSSI_IMPROVEMENT_DB) return "STRONGER";
  if (rssi < lastRssi - RSSI_IMPROVEMENT_DB) return "WEAKER";
  return "STABLE";
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWiFiAttemptAt < WIFI_RETRY_MS) return;
  lastWiFiAttemptAt = millis();
  mode = WAITING_WIFI;
  stopDrive();
  WiFi.begin(WAYFINDER_WIFI_SSID, WAYFINDER_WIFI_PASSWORD);
}

bool httpGetCommand(String& response) {
  WiFiClient client;
  if (!client.connect(WAYFINDER_BACKEND_HOST, WAYFINDER_BACKEND_PORT)) return false;
  client.print("GET /api/device/command HTTP/1.1\r\nHost: ");
  client.print(WAYFINDER_BACKEND_HOST);
  client.print("\r\nConnection: close\r\n\r\n");
  const uint32_t deadline = millis() + 900;
  while (client.connected() && millis() < deadline) {
    while (client.available()) response += static_cast<char>(client.read());
  }
  client.stop();
  const int body = response.indexOf("\r\n\r\n");
  if (body < 0) return false;
  response.remove(0, body + 4);
  return response.indexOf("\"action\"") >= 0;
}

String jsonAction(const String& body) {
  const String key = "\"action\":\"";
  const int start = body.indexOf(key);
  if (start < 0) return "";
  const int valueStart = start + key.length();
  const int end = body.indexOf('"', valueStart);
  return end < 0 ? "" : body.substring(valueStart, end);
}

void startHunt() {
  if (WiFi.status() != WL_CONNECTED) { mode = WAITING_WIFI; return; }
  stopDrive();
  mode = WIFI_HUNT;
  huntStage = HUNT_BASELINE;
  updateRssi(true);
  baselineRssi = currentRssi();
}

void manualMove(DriveKind kind, uint32_t durationMs, float frontCm) {
  if (kind == DRIVE_FORWARD && !forwardClear(frontCm)) {
    mode = BLOCKED;
    stopDrive();
    return;
  }
  mode = MANUAL;
  startDrive(kind, durationMs);
}

void applyPlan(const String& action, float frontCm) {
  if (action == lastPlanAction) return;
  lastPlanAction = action;
  if (action == "stop") { mode = IDLE; stopDrive(); return; }
  if (action == "explore" || action == "wifi_hunt") { startHunt(); return; }
  if (action == "scan") { mode = IDLE; stopDrive(); return; }
  if (action == "forward") { manualMove(DRIVE_FORWARD, STEP_MS, frontCm); return; }
  if (action == "backward") { manualMove(DRIVE_BACKWARD, STEP_MS, frontCm); return; }
  if (action == "left") { manualMove(DRIVE_LEFT, TURN_MS, frontCm); return; }
  if (action == "right") { manualMove(DRIVE_RIGHT, TURN_MS, frontCm); return; }
}

void pollCommand(float frontCm) {
  if (millis() - lastCommandAt < COMMAND_MS || WiFi.status() != WL_CONNECTED) return;
  lastCommandAt = millis();
  String response;
  if (httpGetCommand(response)) applyPlan(jsonAction(response), frontCm);
}

void avoid(float frontCm) {
  mode = AVOIDING;
  if (huntStage != HUNT_AFTER_BACK && huntStage != HUNT_AFTER_TURN) {
    stopDrive();
    startDrive(DRIVE_BACKWARD, BACK_MS);
    huntStage = HUNT_AFTER_BACK;
    return;
  }
  if (!driveFinished()) return;
  if (huntStage == HUNT_AFTER_BACK) {
    startDrive(turnRightNext ? DRIVE_RIGHT : DRIVE_LEFT, AVOID_TURN_MS);
    turnRightNext = !turnRightNext;
    huntStage = HUNT_AFTER_TURN;
    return;
  }
  if (!driveFinished()) return;
  updateRssi(true);
  baselineRssi = currentRssi();
  mode = WIFI_HUNT;
  huntStage = HUNT_BASELINE;
}

void runHunt(float frontCm) {
  if (WiFi.status() != WL_CONNECTED) { mode = WAITING_WIFI; stopDrive(); return; }
  if (currentRssi() >= targetRssiThreshold()) { mode = TARGET_NEARBY; stopDrive(); return; }
  if (frontCm < 0 || !forwardClear(frontCm)) { avoid(frontCm); return; }
  if (!driveFinished()) return;
  updateRssi(huntStage == HUNT_BASELINE || huntStage == HUNT_AFTER_STEP || huntStage == HUNT_AFTER_TURN);
  const float rssi = currentRssi();
  if (huntStage == HUNT_BASELINE) {
    baselineRssi = rssi;
    startDrive(DRIVE_FORWARD, STEP_MS);
    huntStage = HUNT_AFTER_STEP;
  } else if (huntStage == HUNT_AFTER_STEP) {
    if (rssi >= baselineRssi + RSSI_IMPROVEMENT_DB) {
      baselineRssi = rssi;
      startDrive(DRIVE_FORWARD, STEP_MS);
    } else {
      startDrive(DRIVE_BACKWARD, BACK_MS);
      huntStage = HUNT_AFTER_BACK;
    }
  } else if (huntStage == HUNT_AFTER_BACK) {
    startDrive(turnRightNext ? DRIVE_RIGHT : DRIVE_LEFT, TURN_MS);
    turnRightNext = !turnRightNext;
    huntStage = HUNT_AFTER_TURN;
  } else {
    baselineRssi = rssi;
    huntStage = HUNT_BASELINE;
  }
}

void sendTelemetry(float frontCm) {
  if (WiFi.status() != WL_CONNECTED || millis() - lastTelemetryAt < TELEMETRY_MS) return;
  lastTelemetryAt = millis();
  const float rssi = currentRssi();
  const char* trend = signalTrend(rssi);
  WiFiClient client;
  if (!client.connect(WAYFINDER_BACKEND_HOST, WAYFINDER_BACKEND_PORT)) return;
  char body[500];
  const bool valid = frontCm >= 0;
  const float roughDistance = roughWifiDistanceMetres();
  snprintf(body, sizeof(body),
    "{\"source\":\"uno-r4-wifi\",\"mode\":\"%s\",\"rssi_dbm\":%.1f,\"signal_trend\":\"%s\",\"target_ssid\":\"%s\",\"network\":\"%s\",\"target_nearby\":%s,\"rssi_calibrated\":%s,\"rough_wifi_distance_m\":%s,\"sonar\":{\"front\":%s},\"scan\":{\"relative_angle\":0,\"distance_cm\":%s,\"valid_return\":%s}}",
    modeName(), rssi, trend, WAYFINDER_WIFI_SSID, WAYFINDER_WIFI_SSID,
    mode == TARGET_NEARBY ? "true" : "false",
    rssiCalibrated ? "true" : "false",
    roughDistance >= 0 ? String(roughDistance, 1).c_str() : "null",
    valid ? String(frontCm, 1).c_str() : "null",
    valid ? String(frontCm, 1).c_str() : "null",
    valid ? "true" : "false");
  client.print("POST /api/telemetry HTTP/1.1\r\nHost: ");
  client.print(WAYFINDER_BACKEND_HOST);
  client.print("\r\nContent-Type: application/json\r\nContent-Length: ");
  client.print(strlen(body));
  client.print("\r\nConnection: close\r\n\r\n");
  client.print(body);
  lastRssi = rssi;
  client.stop();
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  driveRaw(0, 0);
  activeDrive = DRIVE_STOP;
  matrix.begin();
  showSymbol(DRIVE_STOP);
  Serial.begin(115200);
  delay(1500);
  Serial.println("Wayfinder rover ready. Wi-Fi + fixed forward sonar. Serial: c=calibrate at 1 m, s=status.");
  ensureWiFi();
}

void loop() {
  ensureWiFi();
  updateRssi();
  handleSerial();
  const float frontCm = readDistanceCm();
  pollCommand(frontCm);
  if (mode == WIFI_HUNT || mode == AVOIDING) runHunt(frontCm);
  else if (mode == MANUAL) {
    if (activeDrive == DRIVE_FORWARD && !forwardClear(frontCm)) {
      mode = BLOCKED;
      stopDrive();
    } else driveFinished();
  }
  sendTelemetry(frontCm);
}
