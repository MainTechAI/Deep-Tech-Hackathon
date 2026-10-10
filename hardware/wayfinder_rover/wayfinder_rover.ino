/*
  Wayfinder rover: Wi-Fi hotspot seeker with a fixed forward HC-SR04.

  Board: Arduino UNO R4 WiFi
  Motor driver: L298N
    ENA D8, IN1 D9, IN2 D10  (left motor)
    ENB D13, IN3 D11, IN4 D12 (right motor)
  Fixed HC-SR04: TRIG D3, ECHO D2

  Copy secrets.h.example to secrets.h and set the hotspot credentials and the
  laptop's IP address. The laptop and rover must join the same hotspot.

  Safety: a missing or invalid sonar reading is UNKNOWN, never clear space.
  The rover will not drive forward without a valid front reading above safeCm.
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

// ENA/ENB must be on PWM pins (UNO R4: D3, D5, D6, D9, D10, D11) for speed control.
constexpr uint8_t ENA = 5, IN1 = 9, IN2 = 10;
constexpr uint8_t ENB = 6, IN3 = 11, IN4 = 12;
constexpr uint8_t TRIG_PIN = 3, ECHO_PIN = 2;

constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;
constexpr bool SWAP_SIDES = false;

// Starting values, not measured. Raise a duty if the wheels hum without turning.
constexpr uint8_t KICK_DUTY = 200;   // Brief push to get the wheels turning at low duty.
constexpr uint32_t KICK_MS = 40;
constexpr uint8_t TURN_DUTY = 170;

constexpr float OPEN_CM = 350.0f;  // Reported when nothing is in sonar range.
constexpr float RSSI_IMPROVEMENT_DB = 1.5f;
constexpr float RSSI_WORSE_DB = 4.0f;  // Turn when this far below the best reading on a heading.
constexpr uint32_t BACK_MS = 360;
constexpr uint32_t AVOID_TURN_MS = 300; // Tune on the floor: intended to be about 70 degrees.

// Dead reckoning for the map. There are no encoders, so the pose is integrated
// from how long each move ran, and it drifts. The turn rate and start-up lag
// are fitted to two floor observations (130 ms -> ~20 degrees, 520 ms -> ~120
// degrees). The forward speed is from a measured 1 s run.
constexpr float FORWARD_CM_PER_S = 30.0f;  // Measured: 30 cm in a 1 s run at duty 110.
constexpr float TURN_DEG_PER_S = 256.0f;
constexpr float MOVE_LAG_S = 0.052f;  // Time before the wheels are up to speed.
constexpr uint32_t TELEMETRY_MS = 700;
constexpr uint32_t COMMAND_MS = 850;
constexpr uint32_t WIFI_RETRY_MS = 8000;
constexpr uint32_t RSSI_SAMPLE_MS = 150;
constexpr uint32_t RSSI_CALIBRATION_MS = 10000;
constexpr float RSSI_FILTER_ALPHA = 0.30f;

// Live settings. These are the defaults; the console sliders override them
// through the "tuning" object in the gateway's command response.
constexpr uint8_t DEFAULT_DRIVE_DUTY = 110;
uint8_t driveDuty = DEFAULT_DRIVE_DUTY;  // Slow cruise so RSSI can keep up with the motion.
uint32_t stepMs = 340;                   // One forward step.
uint32_t turnMs = 130;                   // Observed on the floor: about 20 degrees.
uint32_t settleMs = 700;                 // Stand still after each move so RSSI can settle.
float safeCm = 25.0f;                    // Avoid obstacles closer than this.
float leftTrim = 0.98f, rightTrim = 1.0f;  // Per-wheel balance so the rover drives straight.
float targetRssi = -50.0f;               // Found at this signal or stronger: about 50 cm in the test room.

ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {};

enum RoverMode : uint8_t { IDLE, MANUAL, WIFI_HUNT, AVOIDING, TARGET_NEARBY, WAITING_WIFI, BLOCKED };
enum DriveKind : uint8_t { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
enum HuntStage : uint8_t { HUNT_BASELINE, HUNT_AFTER_STEP, HUNT_AFTER_BACK, HUNT_AFTER_TURN };

RoverMode mode = IDLE;
DriveKind activeDrive = DRIVE_STOP;
HuntStage huntStage = HUNT_BASELINE;
uint32_t driveEndsAt = 0;
uint32_t driveStartedAt = 0;
uint32_t driveStoppedAt = 0;
float poseX = 0.0f, poseY = 0.0f;  // Metres from where the rover started.
float poseHeading = 90.0f;         // Degrees, counter-clockwise; 90 points up the map.
String lastResetToken = "";
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
long lastCommandId = -1;

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

void setMotor(uint8_t enable, uint8_t forwardPin, uint8_t reversePin, int direction, uint8_t duty) {
  if (direction > 0) {
    digitalWrite(forwardPin, HIGH); digitalWrite(reversePin, LOW); analogWrite(enable, duty);
  } else if (direction < 0) {
    digitalWrite(forwardPin, LOW); digitalWrite(reversePin, HIGH); analogWrite(enable, duty);
  } else {
    digitalWrite(forwardPin, LOW); digitalWrite(reversePin, LOW); analogWrite(enable, 0);
  }
}

void driveRaw(int left, int right, uint8_t duty = 0) {
  if (SWAP_SIDES) { int swap = left; left = right; right = swap; }
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
  setMotor(ENA, IN1, IN2, left, static_cast<uint8_t>(duty * leftTrim));
  setMotor(ENB, IN3, IN4, right, static_cast<uint8_t>(duty * rightTrim));
}

// Adds the move that just ended to the estimated pose.
void integratePose() {
  if (activeDrive == DRIVE_STOP) return;
  const float seconds = max(0.0f, (millis() - driveStartedAt) / 1000.0f - MOVE_LAG_S);
  if (activeDrive == DRIVE_LEFT) poseHeading += TURN_DEG_PER_S * seconds;
  else if (activeDrive == DRIVE_RIGHT) poseHeading -= TURN_DEG_PER_S * seconds;
  else {
    // Speed is assumed proportional to duty, which is only roughly true.
    const float metres = FORWARD_CM_PER_S * driveDuty / DEFAULT_DRIVE_DUTY * seconds / 100.0f * (activeDrive == DRIVE_FORWARD ? 1.0f : -1.0f);
    poseX += metres * cosf(poseHeading * DEG_TO_RAD);
    poseY += metres * sinf(poseHeading * DEG_TO_RAD);
  }
  poseHeading = fmodf(poseHeading + 360.0f, 360.0f);
}

void stopDrive() {
  driveRaw(0, 0);
  if (activeDrive != DRIVE_STOP) driveStoppedAt = millis();
  integratePose();
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
  const bool turning = kind == DRIVE_LEFT || kind == DRIVE_RIGHT;
  integratePose();
  if (!turning && KICK_DUTY > driveDuty) {
    driveRaw(left, right, KICK_DUTY);
    delay(KICK_MS);
  }
  driveRaw(left, right, turning ? TURN_DUTY : driveDuty);
  activeDrive = kind;
  driveStartedAt = millis();
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

// One ping. A sensor that answers but hears no echo means open space, so it
// returns OPEN_CM. Only a sensor that never raises ECHO (unplugged, unpowered)
// returns -1.
float pingCm() {
  uint32_t startedAt = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {  // Previous ping still waiting for its echo.
    if (micros() - startedAt > 60000) return OPEN_CM;
  }
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  startedAt = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - startedAt > 5000) return -1.0f;
  }
  const uint32_t roseAt = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - roseAt > 25000) return OPEN_CM;
  }
  return min((micros() - roseAt) * 0.0343f / 2.0f, OPEN_CM);
}

// quick: return the first valid ping instead of a median of three, so a timed
// move is not held up while the motors are running.
float readDistanceCm(bool quick = false) {
  float readings[3];
  uint8_t count = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    const float value = pingCm();
    if (value >= 3.0f) readings[count++] = value;
    if (quick && count) return readings[0];
    delay(8);
  }
  if (!count) return -1.0f;
  for (uint8_t i = 0; i + 1 < count; ++i)
    for (uint8_t j = i + 1; j < count; ++j)
      if (readings[j] < readings[i]) { const float tmp = readings[i]; readings[i] = readings[j]; readings[j] = tmp; }
  return readings[count / 2];
}

bool forwardClear(float frontCm) { return frontCm >= safeCm; }

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
  // Team decision: the target counts as found at -50 dBm or stronger, which
  // was about 50 cm from the phone in the test room.
  return targetRssi;
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
  if (WiFi.status() == WL_CONNECTED) {
    if (mode == WAITING_WIFI) mode = IDLE;
    return;
  }
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
  return response.indexOf("\"command_id\"") >= 0;
}

String jsonAction(const String& body) {
  // The gateway's JSON has a space after the colon, so do not match a fixed string.
  const String key = "\"action\"";
  const int start = body.indexOf(key);
  if (start < 0) return "";
  const int valueStart = body.indexOf('"', body.indexOf(':', start + key.length())) + 1;
  if (valueStart <= 0) return "";
  const int end = body.indexOf('"', valueStart);
  return end < 0 ? "" : body.substring(valueStart, end);
}

// The gateway increments command_id on every operator command, so the same
// action can be sent twice in a row.
long jsonCommandId(const String& body) {
  const String key = "\"command_id\"";
  const int start = body.indexOf(key);
  if (start < 0) return 0;
  return body.substring(body.indexOf(':', start + key.length()) + 1).toInt();
}

bool jsonNumber(const String& body, const char* name, float& out) {
  const String key = String("\"") + name + "\"";
  const int start = body.indexOf(key);
  if (start < 0) return false;
  out = body.substring(body.indexOf(':', start + key.length()) + 1).toFloat();
  return true;
}

void applyTuning(const String& body) {
  float v;
  if (jsonNumber(body, "drive_duty", v)) driveDuty = constrain(static_cast<int>(v), 60, 255);
  if (jsonNumber(body, "step_ms", v)) stepMs = constrain(static_cast<int>(v), 50, 2000);
  if (jsonNumber(body, "settle_ms", v)) settleMs = constrain(static_cast<int>(v), 0, 5000);
  if (jsonNumber(body, "turn_ms", v)) turnMs = constrain(static_cast<int>(v), 30, 1000);
  if (jsonNumber(body, "safe_cm", v)) safeCm = constrain(v, 5.0f, 100.0f);
  if (jsonNumber(body, "target_rssi", v)) targetRssi = constrain(v, -90.0f, -20.0f);
  if (jsonNumber(body, "left_trim_pct", v)) {
    // Above 100 the right wheel is slowed instead, since duty cannot exceed full power.
    v = constrain(v, 70.0f, 130.0f);
    leftTrim = min(1.0f, v / 100.0f);
    rightTrim = min(1.0f, 100.0f / v);
  }
}

// The console's reset clears the map, so the pose estimate restarts with it.
void applyResetToken(const String& body) {
  const String key = "\"reset_token\"";
  const int start = body.indexOf(key);
  if (start < 0) return;
  const int valueStart = body.indexOf(':', start + key.length()) + 1;
  int end = body.indexOf(',', valueStart);
  if (end < 0) end = body.indexOf('}', valueStart);
  const String token = body.substring(valueStart, end);
  if (lastResetToken.length() && token != lastResetToken) { poseX = 0.0f; poseY = 0.0f; poseHeading = 90.0f; }
  lastResetToken = token;
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

void applyPlan(const String& action, long commandId, float frontCm) {
  // The first poll after a reboot only records the id, so a command left over
  // in the gateway is not replayed.
  const bool firstPoll = lastCommandId < 0;
  if (commandId == lastCommandId) return;
  lastCommandId = commandId;
  if (firstPoll) return;
  if (action == "stop") { mode = IDLE; stopDrive(); return; }
  if (action == "explore" || action == "wifi_hunt") { startHunt(); return; }
  if (action == "scan") { mode = IDLE; stopDrive(); return; }
  if (action == "forward") { manualMove(DRIVE_FORWARD, stepMs, frontCm); return; }
  // Calibration run: measure the distance covered to set FORWARD_CM_PER_S.
  if (action == "forward_1s") { manualMove(DRIVE_FORWARD, 1000, frontCm); return; }
  if (action == "backward") { manualMove(DRIVE_BACKWARD, stepMs, frontCm); return; }
  if (action == "left") { manualMove(DRIVE_LEFT, turnMs, frontCm); return; }
  if (action == "right") { manualMove(DRIVE_RIGHT, turnMs, frontCm); return; }
}

void pollCommand(float frontCm) {
  if (millis() - lastCommandAt < COMMAND_MS || WiFi.status() != WL_CONNECTED) return;
  lastCommandAt = millis();
  String response;
  if (!httpGetCommand(response)) return;
  applyResetToken(response);
  applyTuning(response);
  applyPlan(jsonAction(response), jsonCommandId(response), frontCm);
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
    // Always the same way: alternating left and right undid the previous turn.
    startDrive(DRIVE_RIGHT, AVOID_TURN_MS);
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
  if (millis() - driveStoppedAt < settleMs) return;
  updateRssi(huntStage == HUNT_BASELINE || huntStage == HUNT_AFTER_STEP || huntStage == HUNT_AFTER_TURN);
  const float rssi = currentRssi();
  if (huntStage == HUNT_BASELINE) {
    baselineRssi = rssi;
    startDrive(DRIVE_FORWARD, stepMs);
    huntStage = HUNT_AFTER_STEP;
  } else if (huntStage == HUNT_AFTER_STEP) {
    // Keep the heading unless the signal is clearly weaker than the best seen
    // on it. RSSI jitters by a few dB, so demanding an improvement after every
    // step made the rover reverse almost every time.
    if (rssi > baselineRssi) baselineRssi = rssi;
    if (rssi >= baselineRssi - RSSI_WORSE_DB) {
      startDrive(DRIVE_FORWARD, stepMs);
    } else {
      startDrive(DRIVE_RIGHT, turnMs);
      huntStage = HUNT_AFTER_TURN;
    }
  } else if (huntStage == HUNT_AFTER_BACK) {
    startDrive(DRIVE_RIGHT, AVOID_TURN_MS);
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
  char body[640];
  const bool valid = frontCm >= 0;
  const bool echo = valid && frontCm < OPEN_CM;  // Open space is not a map point.
  const float roughDistance = roughWifiDistanceMetres();
  snprintf(body, sizeof(body),
    "{\"source\":\"uno-r4-wifi\",\"mode\":\"%s\",\"rssi_dbm\":%.1f,\"signal_trend\":\"%s\",\"target_ssid\":\"%s\",\"network\":\"%s\",\"target_nearby\":%s,\"rssi_calibrated\":%s,\"rough_wifi_distance_m\":%s,\"position\":{\"x\":%.2f,\"y\":%.2f},\"heading\":%.0f,\"sonar\":{\"front\":%s},\"scan\":{\"relative_angle\":0,\"distance_cm\":%s,\"valid_return\":%s}}",
    modeName(), rssi, trend, WAYFINDER_WIFI_SSID, WAYFINDER_WIFI_SSID,
    mode == TARGET_NEARBY ? "true" : "false",
    rssiCalibrated ? "true" : "false",
    roughDistance >= 0 ? String(roughDistance, 1).c_str() : "null",
    poseX, poseY, poseHeading,
    valid ? String(frontCm, 1).c_str() : "null",
    echo ? String(frontCm, 1).c_str() : "null",
    echo ? "true" : "false");
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
  // HTTP calls block for up to a second and the motors keep running through
  // them, so the backend is only contacted while the rover is stopped.
  const bool moving = !driveFinished();
  const float frontCm = readDistanceCm(moving);
  if (!moving) {
    pollCommand(frontCm);
    if (activeDrive == DRIVE_STOP) sendTelemetry(frontCm);
  }
  if (mode == WIFI_HUNT || mode == AVOIDING) runHunt(frontCm);
  else if (mode == MANUAL) {
    if (activeDrive == DRIVE_FORWARD && !forwardClear(frontCm)) {
      mode = BLOCKED;
      stopDrive();
    } else driveFinished();
  }
}
