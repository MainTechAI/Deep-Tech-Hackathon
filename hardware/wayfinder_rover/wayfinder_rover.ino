/*
  Wayfinder rover: Wi-Fi hotspot seeker with a fixed forward HC-SR04.

  Board: Arduino UNO R4 WiFi
  Motor driver: L298N
    ENA D5 (PWM), IN1 D9, IN2 D10  (left motor)
    ENB D6 (PWM), IN3 D11, IN4 D12 (right motor)
  Fixed HC-SR04: TRIG D3, ECHO D2

  Copy secrets.h.example to secrets.h and set the hotspot credentials and the
  laptop's IP address. The laptop and rover must join the same hotspot.

  Sonar: no echo means open space. Only a sonar that does not answer at all
  (unplugged, unpowered) blocks forward motion.

  Position: there are no wheel encoders or IMU. The pose is dead-reckoned from
  how long each move ran, and forward travel is corrected from the change in
  the sonar reading whenever there is a steady echo ahead. It still drifts,
  mostly in heading.

  Timing: Wi-Fi calls (status, RSSI, HTTP) block for tens of milliseconds to a
  second, and the motors keep running through them. So while a move is running
  the loop does nothing but watch the clock and, when driving forward, ping the
  sonar. Everything else happens while the rover is stopped.
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

// Motors
constexpr uint8_t KICK_DUTY = 200;   // Brief push to get the wheels turning at low duty.
constexpr uint32_t KICK_MS = 40;
constexpr uint8_t TURN_DUTY = 170;
constexpr uint32_t BRAKE_MS = 120;   // Short the motors after a move so it ends in the same place every time.
constexpr uint32_t BACK_MS = 360;    // Back-up before turning away from an obstacle.
constexpr float AVOID_TURN_DEG = 70.0f;
constexpr float TURN_LAG_S = 0.052f; // Time before the wheels are up to speed in a turn.
constexpr uint32_t MAX_MOVE_MS = 1500; // No single move runs longer: Stop is only heard between moves.

// Sonar
constexpr float OPEN_CM = 350.0f;        // Reported when nothing is in sonar range.
constexpr uint32_t PING_GAP_MS = 50;     // Echoes of the previous ping must die out, or they read as false short distances.
constexpr uint32_t PING_BUDGET_MS = 35;  // Do not start a ping this close to the end of a move.
constexpr float SONAR_OFFSET_CM = 9.0f;  // Sonar face ahead of the turning centre (between the wheels). Estimate.
constexpr float MAP_MAX_CM = 200.0f;     // Beyond this the HC-SR04 is too vague to map or to measure travel with.
constexpr float CLEAR_OPEN_CM = 50.0f;   // How far ahead an open reading is trusted to be empty. Short, because a wall hit at an angle also reads as open.
constexpr uint8_t STUCK_HIGH_LIMIT = 4;  // Consecutive pings with ECHO held high before the sonar counts as dead.

// Odometry
constexpr uint8_t DEFAULT_DRIVE_DUTY = 110;
constexpr float FORWARD_CM_PER_S = 30.0f;  // Measured: 30 cm in a 1 s run at duty 110, start-up kick and coast included.
constexpr uint32_t ODO_SETTLE_MS = 250;    // Stand still this long before trusting the "after" sonar reading.

// Wi-Fi and backend
constexpr uint32_t EXCHANGE_MS = 400;
constexpr uint32_t WIFI_CHECK_MS = 500;
constexpr uint32_t WIFI_RETRY_MS = 8000;
constexpr uint32_t BACKEND_LOST_MS = 6000;  // Stop moving if the console cannot reach the rover to say Stop.
constexpr uint32_t RSSI_SAMPLE_MS = 150;
constexpr uint32_t RSSI_CALIBRATION_MS = 10000;
constexpr float RSSI_FILTER_ALPHA = 0.30f;
constexpr float RSSI_IMPROVEMENT_DB = 1.5f;
constexpr float RSSI_WORSE_DB = 4.0f;       // Turn when this far below the best reading on a heading.
constexpr uint8_t RSSI_MIN_SAMPLES = 3;     // Samples at a standstill before the hunt decides.
constexpr float RSSI_SURE_DB = 4.0f;        // This far above the target, one stop is enough to call it found.

// Live settings. These are the defaults; the console sliders override them
// through the "tuning" object in the gateway's reply.
uint8_t driveDuty = DEFAULT_DRIVE_DUTY;  // Slow cruise so RSSI can keep up with the motion.
uint32_t stepMs = 340;                   // One forward step.
uint32_t settleMs = 700;                 // Stand still after each move so RSSI can settle.
float turnDeg = 20.0f;                   // One manual turn.
float huntTurnDeg = 45.0f;               // Turn made by the hunt when the signal drops.
float turnRate = 256.0f;                 // Degrees per second at TURN_DUTY. Set from the 1 s spin test.
float safeCm = 25.0f;                    // Avoid obstacles closer than this.
float leftTrim = 0.98f, rightTrim = 1.0f;  // Per-wheel balance so the rover drives straight.
float targetRssi = -50.0f;               // Found at this signal or stronger: about 50 cm in the test room.

ArduinoLEDMatrix matrix;
uint8_t frame[8][12] = {};

enum RoverMode : uint8_t { IDLE, MANUAL, WIFI_HUNT, AVOIDING, TARGET_NEARBY, WAITING_WIFI, BLOCKED };
enum DriveKind : uint8_t { DRIVE_STOP, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
// What the hunt did last, which decides what it does next.
enum HuntStage : uint8_t { HUNT_START, HUNT_STEPPED, HUNT_BACKED, HUNT_TURNED };

RoverMode mode = IDLE;
DriveKind activeDrive = DRIVE_STOP;
HuntStage huntStage = HUNT_START;
uint32_t driveStartedAt = 0;
uint32_t driveEndsAt = 0;
uint32_t driveStoppedAt = 0;
bool braking = false;

float poseX = 0.0f, poseY = 0.0f;  // Metres from where the rover started.
float poseHeading = 90.0f;         // Degrees, counter-clockwise; 90 points up the map.
float speedScale = 1.0f;           // Learned correction to FORWARD_CM_PER_S from sonar-measured steps.
float odoBeforeCm = -1.0f;         // Steady echo ahead before the current straight move, or -1.
float odoExpectedCm = 0.0f;        // Signed travel the clock predicted for the move that just ended.
bool odoPending = false;
bool odoLearn = false;              // The move ran its full length forward, so it may tune speedScale.
uint16_t odoFixes = 0;

float frontCm = -1.0f;             // Latest sonar reading.
bool frontSteady = false;          // True when frontCm is a median taken at a standstill.
uint32_t lastPingAt = 0;
uint8_t stuckHighCount = 0;
uint8_t strongStops = 0;           // Consecutive stops with the signal at or above the target.

bool wifiUp = false;
uint32_t lastWiFiCheckAt = 0;
uint32_t lastWiFiAttemptAt = 0;
uint32_t lastExchangeAt = 0;
uint32_t lastBackendOkAt = 0;
uint32_t lastRssiSampleAt = 0;
float filteredRssi = NAN;          // Smoothed across time, for display.
float stopRssiSum = 0.0f;          // Samples since the rover last stopped, for hunt decisions.
uint8_t stopRssiCount = 0;
float bestRssi = -127.0f;          // Best reading on the current heading.
float lastSentRssi = -127.0f;
float rssiAtOneMetre = WAYFINDER_RSSI_AT_ONE_METRE;
float calibrationSum = 0.0f;
uint16_t calibrationSamples = 0;
uint32_t calibrationStartedAt = 0;
bool rssiAvailable = false;
bool calibrationActive = false;
bool rssiCalibrated = WAYFINDER_RSSI_CALIBRATED;
long lastCommandId = -1;
String lastResetToken = "";

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

// ---------------------------------------------------------------- motors

void setMotor(uint8_t enable, uint8_t forwardPin, uint8_t reversePin, int direction, uint8_t duty) {
  digitalWrite(forwardPin, direction > 0 ? HIGH : LOW);
  digitalWrite(reversePin, direction < 0 ? HIGH : LOW);
  analogWrite(enable, duty);
}

void driveRaw(int left, int right, uint8_t duty) {
  if (SWAP_SIDES) { int swap = left; left = right; right = swap; }
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
  setMotor(ENA, IN1, IN2, left, static_cast<uint8_t>(duty * leftTrim));
  setMotor(ENB, IN3, IN4, right, static_cast<uint8_t>(duty * rightTrim));
}

// Both inputs low with the bridge enabled shorts the motor, which stops it
// quickly. With the bridge disabled it coasts.
void brakeMotors() { setMotor(ENA, IN1, IN2, 0, 255); setMotor(ENB, IN3, IN4, 0, 255); braking = true; }
void releaseMotors() { setMotor(ENA, IN1, IN2, 0, 0); setMotor(ENB, IN3, IN4, 0, 0); braking = false; }

uint32_t turnMsFor(float degrees) {
  return min(static_cast<uint32_t>((TURN_LAG_S + degrees / turnRate) * 1000.0f), MAX_MOVE_MS);
}

void movePose(float cm) {
  poseX += cm / 100.0f * cosf(poseHeading * DEG_TO_RAD);
  poseY += cm / 100.0f * sinf(poseHeading * DEG_TO_RAD);
}

// Adds the move that just ended to the estimated pose, from the clock alone.
void integratePose() {
  if (activeDrive == DRIVE_STOP) return;
  const float seconds = (millis() - driveStartedAt) / 1000.0f;
  if (activeDrive == DRIVE_LEFT || activeDrive == DRIVE_RIGHT) {
    poseHeading += turnRate * max(0.0f, seconds - TURN_LAG_S) * (activeDrive == DRIVE_LEFT ? 1.0f : -1.0f);
    poseHeading = fmodf(poseHeading + 360.0f, 360.0f);
    return;
  }
  // Speed is assumed proportional to duty, which is only roughly true;
  // speedScale absorbs the difference once sonar-measured steps come in.
  const float cm = FORWARD_CM_PER_S * speedScale * driveDuty / DEFAULT_DRIVE_DUTY * seconds;
  odoExpectedCm = activeDrive == DRIVE_FORWARD ? cm : -cm;
  odoPending = odoBeforeCm >= 0.0f;
  odoLearn = activeDrive == DRIVE_FORWARD && static_cast<int32_t>(millis() - driveEndsAt) >= 0;
  movePose(odoExpectedCm);
}

void stopDrive() {
  if (activeDrive == DRIVE_STOP) return;
  brakeMotors();
  driveStoppedAt = millis();
  integratePose();
  activeDrive = DRIVE_STOP;
  stopRssiSum = 0.0f;
  stopRssiCount = 0;
  showSymbol(DRIVE_STOP);
}

void startDrive(DriveKind kind, uint32_t durationMs) {
  stopDrive();
  int left = 0, right = 0;
  if (kind == DRIVE_FORWARD) { left = 1; right = 1; }
  if (kind == DRIVE_BACKWARD) { left = -1; right = -1; }
  if (kind == DRIVE_LEFT) { left = -1; right = 1; }
  if (kind == DRIVE_RIGHT) { left = 1; right = -1; }
  const bool turning = kind == DRIVE_LEFT || kind == DRIVE_RIGHT;
  // A straight move can be measured if there is a steady echo ahead to compare with afterwards.
  odoPending = false;
  odoBeforeCm = (!turning && frontSteady && frontCm >= 0.0f && frontCm < MAP_MAX_CM) ? frontCm : -1.0f;
  braking = false;
  if (!turning && KICK_DUTY > driveDuty) {
    driveRaw(left, right, KICK_DUTY);
    delay(KICK_MS);
  }
  driveRaw(left, right, turning ? TURN_DUTY : driveDuty);
  activeDrive = kind;
  frontSteady = false;
  driveStartedAt = millis();
  driveEndsAt = driveStartedAt + min(durationMs, MAX_MOVE_MS);
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

// ----------------------------------------------------------------- sonar

// One ping. A sensor that answers but hears no echo means open space, so it
// returns OPEN_CM. Only a sensor that never raises ECHO (unplugged, unpowered)
// returns -1.
float pingCm() {
  const uint32_t sincePing = millis() - lastPingAt;
  if (sincePing < PING_GAP_MS) delay(PING_GAP_MS - sincePing);
  uint32_t startedAt = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {  // Previous ping still waiting for its echo.
    if (micros() - startedAt > 60000) {
      // Some clones hold ECHO high for a while after a ping with no echo, which
      // is open space. Held high ping after ping, it is a loose wire or a dead
      // sensor, and that must not read as clear.
      lastPingAt = millis();
      if (stuckHighCount < 255) ++stuckHighCount;
      return stuckHighCount >= STUCK_HIGH_LIMIT ? -1.0f : OPEN_CM;
    }
  }
  stuckHighCount = 0;
  lastPingAt = millis();
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

// quick: return the first ping that shows clear space instead of a median of
// three, so a move is not held up while the motors are running. A close
// reading is still confirmed with the full median: single short readings can
// be noise, not an obstacle.
float readDistanceCm(bool quick) {
  float readings[3];
  uint8_t count = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    const float value = pingCm();
    if (value >= 3.0f) readings[count++] = value;
    if (quick && count && readings[0] >= safeCm) return readings[0];
  }
  if (!count) return -1.0f;
  for (uint8_t i = 0; i + 1 < count; ++i)
    for (uint8_t j = i + 1; j < count; ++j)
      if (readings[j] < readings[i]) { const float tmp = readings[i]; readings[i] = readings[j]; readings[j] = tmp; }
  return readings[(count - 1) / 2];  // With two readings, the nearer one.
}

bool forwardClear() { return frontCm >= safeCm; }

// Once the rover has stood still after a straight move, the change in the
// sonar reading is the distance really covered. Use it in place of the clock's
// guess when the two roughly agree; a wild difference means the echo came from
// something else (a person walking past, a different surface), so keep the guess.
void resolveOdometry() {
  if (!odoPending || millis() - driveStoppedAt < ODO_SETTLE_MS) return;
  odoPending = false;
  if (frontCm < 0.0f || frontCm >= MAP_MAX_CM || fabsf(odoExpectedCm) < 3.0f) return;
  const float measuredCm = odoBeforeCm - frontCm;
  const float ratio = measuredCm / odoExpectedCm;
  if (ratio < 0.5f || ratio > 2.0f) return;
  movePose(measuredCm - odoExpectedCm);
  ++odoFixes;
  // Learn the speed only from full forward steps that agree fairly well: a
  // wall seen at an angle shortens the reading and would bias it for good.
  if (odoLearn && ratio > 0.7f && ratio < 1.4f)
    speedScale = constrain(speedScale * (1.0f + 0.15f * (ratio - 1.0f)), 0.5f, 2.0f);
}

// ------------------------------------------------------------------ RSSI

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

void updateRssi() {
  if (!wifiUp) {
    rssiAvailable = false;
    filteredRssi = NAN;
    calibrationActive = false;
    return;
  }
  if (millis() - lastRssiSampleAt < RSSI_SAMPLE_MS) return;
  lastRssiSampleAt = millis();
  const int raw = WiFi.RSSI();
  if (!validRssi(raw)) return;

  if (!rssiAvailable) filteredRssi = static_cast<float>(raw);
  else filteredRssi = RSSI_FILTER_ALPHA * raw + (1.0f - RSSI_FILTER_ALPHA) * filteredRssi;
  rssiAvailable = true;
  if (stopRssiCount < 250) { stopRssiSum += raw; ++stopRssiCount; }

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

void startCalibration() {
  if (!wifiUp) {
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
  Serial.print(wifiUp ? "connected" : "waiting");
  Serial.print(" | RSSI: ");
  if (rssiAvailable) Serial.print(filteredRssi, 1); else Serial.print("n/a");
  Serial.print(" dBm | mode: ");
  Serial.print(modeName());
  Serial.print(" | front cm: ");
  Serial.print(frontCm, 1);
  Serial.print(" | pose: ");
  Serial.print(poseX, 2); Serial.print(", "); Serial.print(poseY, 2);
  Serial.print(" @ "); Serial.print(poseHeading, 0);
  Serial.print(" | cm/s: ");
  Serial.print(FORWARD_CM_PER_S * speedScale * driveDuty / DEFAULT_DRIVE_DUTY, 1);
  Serial.print(" | sonar fixes: ");
  Serial.println(odoFixes);
}

void handleSerial() {
  while (Serial.available()) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C') startCalibration();
    if (command == 's' || command == 'S') printStatus();
  }
}

const char* signalTrend(float rssi) {
  if (rssi <= -126 || lastSentRssi <= -126) return "COLLECTING";
  if (rssi > lastSentRssi + RSSI_IMPROVEMENT_DB) return "STRONGER";
  if (rssi < lastSentRssi - RSSI_IMPROVEMENT_DB) return "WEAKER";
  return "STABLE";
}

// --------------------------------------------------------------- backend

// WiFi.status() is a round trip to the radio module, so it is polled at a
// modest rate and only while stopped; everything else reads wifiUp.
void ensureWiFi() {
  if (millis() - lastWiFiCheckAt < WIFI_CHECK_MS) return;
  lastWiFiCheckAt = millis();
  wifiUp = WiFi.status() == WL_CONNECTED;
  if (wifiUp) {
    if (mode == WAITING_WIFI) mode = IDLE;
    return;
  }
  mode = WAITING_WIFI;
  if (lastWiFiAttemptAt && millis() - lastWiFiAttemptAt < WIFI_RETRY_MS) return;
  lastWiFiAttemptAt = millis();
  WiFi.begin(WAYFINDER_WIFI_SSID, WAYFINDER_WIFI_PASSWORD);
}

// The gateway's JSON has a space after the colon, so do not match fixed strings.
int jsonValueStart(const String& body, const char* name) {
  const String key = String("\"") + name + "\"";
  const int start = body.indexOf(key);
  if (start < 0) return -1;
  const int colon = body.indexOf(':', start + key.length());
  return colon < 0 ? -1 : colon + 1;
}

bool jsonNumber(const String& body, const char* name, float& out) {
  const int start = jsonValueStart(body, name);
  if (start < 0) return false;
  out = body.substring(start).toFloat();
  return true;
}

String jsonAction(const String& body) {
  const int start = jsonValueStart(body, "action");
  if (start < 0) return "";
  const int valueStart = body.indexOf('"', start) + 1;
  if (valueStart <= 0) return "";
  const int end = body.indexOf('"', valueStart);
  return end < 0 ? "" : body.substring(valueStart, end);
}

void applyTuning(const String& body) {
  float v;
  if (jsonNumber(body, "drive_duty", v)) {
    const uint8_t duty = constrain(static_cast<int>(v), 60, 255);
    if (duty != driveDuty) speedScale = 1.0f;  // What was learned belongs to the old power.
    driveDuty = duty;
  }
  if (jsonNumber(body, "step_ms", v)) stepMs = constrain(static_cast<int>(v), 50, 1000);
  if (jsonNumber(body, "settle_ms", v)) settleMs = constrain(static_cast<int>(v), 0, 5000);
  if (jsonNumber(body, "turn_deg", v)) turnDeg = constrain(v, 3.0f, 180.0f);
  if (jsonNumber(body, "hunt_turn_deg", v)) huntTurnDeg = constrain(v, 5.0f, 180.0f);
  if (jsonNumber(body, "turn_rate", v)) turnRate = constrain(v, 40.0f, 900.0f);
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
  const int start = jsonValueStart(body, "reset_token");
  if (start < 0) return;
  int end = body.indexOf(',', start);
  if (end < 0) end = body.indexOf('}', start);
  String token = body.substring(start, end);
  token.trim();
  if (lastResetToken.length() && token != lastResetToken) { poseX = 0.0f; poseY = 0.0f; poseHeading = 90.0f; }
  lastResetToken = token;
}

// When an overhead camera is tracking the rover, the gateway sends the pose it
// saw. Adopting it at a standstill keeps the dead-reckoned estimate in the
// camera's frame, so it can carry on if the camera loses sight of the rover.
void applyExternalPose(const String& body) {
  float x, y, heading;
  if (!jsonNumber(body, "ext_x", x) || !jsonNumber(body, "ext_y", y) || !jsonNumber(body, "ext_heading", heading)) return;
  poseX = x;
  poseY = y;
  poseHeading = fmodf(heading + 360.0f, 360.0f);
}

void startHunt() {
  if (!wifiUp) { mode = WAITING_WIFI; return; }
  stopDrive();
  mode = WIFI_HUNT;
  huntStage = HUNT_START;
  // Decide from fresh samples, not ones gathered while the rover sat idle.
  stopRssiSum = 0.0f;
  stopRssiCount = 0;
  strongStops = 0;
}

void manualMove(DriveKind kind, uint32_t durationMs) {
  if (kind == DRIVE_FORWARD && !forwardClear()) {
    mode = BLOCKED;
    stopDrive();
    return;
  }
  mode = MANUAL;
  startDrive(kind, durationMs);
}

void applyPlan(const String& body) {
  // The gateway gives every operator command a new id, so the same action can
  // be sent twice in a row. The first reply after a reboot only records the id,
  // so a command left over in the gateway is not replayed.
  float idValue = 0.0f;
  if (!jsonNumber(body, "command_id", idValue)) return;
  const long commandId = body.substring(jsonValueStart(body, "command_id")).toInt();
  const bool firstReply = lastCommandId < 0;
  if (commandId == lastCommandId) return;
  lastCommandId = commandId;
  if (firstReply) return;
  const String action = jsonAction(body);
  if (action == "stop" || action == "scan") { mode = IDLE; stopDrive(); return; }
  if (action == "explore" || action == "wifi_hunt") { startHunt(); return; }
  if (action == "forward") { manualMove(DRIVE_FORWARD, stepMs); return; }
  if (action == "backward") { manualMove(DRIVE_BACKWARD, stepMs); return; }
  if (action == "left") { manualMove(DRIVE_LEFT, turnMsFor(turnDeg)); return; }
  if (action == "right") { manualMove(DRIVE_RIGHT, turnMsFor(turnDeg)); return; }
  // Calibration runs: measure what one second covers and set the speed or turn rate from it.
  if (action == "forward_1s") { manualMove(DRIVE_FORWARD, 1000); return; }
  if (action == "spin_1s") { manualMove(DRIVE_LEFT, 1000); return; }
}

// One HTTP round trip: post telemetry, and read the pending command, tuning
// and reset token from the reply.
void exchange() {
  if (!wifiUp || millis() - lastExchangeAt < EXCHANGE_MS) return;
  lastExchangeAt = millis();
  const float rssi = currentRssi();
  const bool valid = frontCm >= 0.0f;
  const bool echo = valid && frontCm < MAP_MAX_CM;  // Open space and vague far returns are not map points.
  // Distances the map uses are from the turning centre, not the sonar face.
  const float hitCm = frontCm + SONAR_OFFSET_CM;
  const float clearCm = !valid ? 0.0f : (frontCm >= OPEN_CM ? CLEAR_OPEN_CM : min(hitCm - 12.0f, MAP_MAX_CM));
  const float roughDistance = roughWifiDistanceMetres();
  char body[720];
  snprintf(body, sizeof(body),
    "{\"source\":\"uno-r4-wifi\",\"mode\":\"%s\",\"rssi_dbm\":%.1f,\"signal_trend\":\"%s\",\"target_ssid\":\"%s\",\"network\":\"%s\",\"target_nearby\":%s,\"rssi_calibrated\":%s,\"rough_wifi_distance_m\":%s,\"position\":{\"x\":%.2f,\"y\":%.2f},\"heading\":%.0f,\"speed_cm_s\":%.1f,\"odo_fixes\":%u,\"ack_command_id\":%ld,\"reset_token\":%s,\"sonar\":{\"front\":%s},\"scan\":{\"relative_angle\":0,\"distance_cm\":%s,\"valid_return\":%s,\"clear_cm\":%.0f}}",
    modeName(), rssi, signalTrend(rssi), WAYFINDER_WIFI_SSID, WAYFINDER_WIFI_SSID,
    mode == TARGET_NEARBY ? "true" : "false",
    rssiCalibrated ? "true" : "false",
    roughDistance >= 0 ? String(roughDistance, 1).c_str() : "null",
    poseX, poseY, poseHeading,
    FORWARD_CM_PER_S * speedScale * driveDuty / DEFAULT_DRIVE_DUTY, odoFixes, lastCommandId,
    lastResetToken.length() ? lastResetToken.c_str() : "-1",
    valid ? String(frontCm, 1).c_str() : "null",
    echo ? String(hitCm, 1).c_str() : "null",
    echo ? "true" : "false",
    max(clearCm, 0.0f));

  WiFiClient client;
  client.setConnectionTimeout(1500);
  if (!client.connect(WAYFINDER_BACKEND_HOST, WAYFINDER_BACKEND_PORT)) {
    client.stop();  // connect() takes a socket on the radio module even when it fails; only stop() frees it.
    return;
  }
  client.print("POST /api/telemetry HTTP/1.1\r\nHost: ");
  client.print(WAYFINDER_BACKEND_HOST);
  client.print("\r\nContent-Type: application/json\r\nContent-Length: ");
  client.print(strlen(body));
  client.print("\r\nConnection: close\r\n\r\n");
  client.print(body);
  lastSentRssi = rssi;

  String response;
  response.reserve(700);
  const uint32_t startedAt = millis();
  while (client.connected() && millis() - startedAt < 900) {
    while (client.available()) response += static_cast<char>(client.read());
  }
  while (client.available()) response += static_cast<char>(client.read());
  client.stop();
  const int bodyStart = response.indexOf("\r\n\r\n");
  if (bodyStart < 0) return;
  response.remove(0, bodyStart + 4);
  if (jsonValueStart(response, "command_id") < 0) return;
  lastBackendOkAt = millis();
  applyResetToken(response);
  applyExternalPose(response);
  applyTuning(response);
  applyPlan(response);
}

// ------------------------------------------------------------------ hunt

// Run and tumble: keep stepping along a heading while the signal holds up,
// and turn when it clearly drops. One antenna gives no bearing, so direction
// only shows up as a change in signal after moving. Called at a standstill.
void runHunt() {
  if (!wifiUp) { mode = WAITING_WIFI; return; }
  if (millis() - driveStoppedAt < settleMs || stopRssiCount < RSSI_MIN_SAMPLES) return;
  // The mean of the samples taken at this spot, not a filter that still
  // remembers the previous spot.
  const float rssi = stopRssiSum / stopRssiCount;
  // Samples at one spot share the same reflections, so one strong spot can be
  // a fluke. Found means two stops in a row at the target, or one well above it.
  strongStops = rssi >= targetRssi ? strongStops + 1 : 0;
  if (strongStops >= 2 || rssi >= targetRssi + RSSI_SURE_DB) { mode = TARGET_NEARBY; return; }

  if (huntStage == HUNT_BACKED) {
    // Always the same way: alternating left and right undid the previous turn.
    startDrive(DRIVE_RIGHT, turnMsFor(AVOID_TURN_DEG));
    huntStage = HUNT_TURNED;
    return;
  }
  if (frontCm < 0.0f) return;  // Sonar not answering: stay put instead of driving blind.
  if (!forwardClear()) {
    mode = AVOIDING;
    startDrive(DRIVE_BACKWARD, BACK_MS);
    huntStage = HUNT_BACKED;
    return;
  }
  mode = WIFI_HUNT;
  if (huntStage == HUNT_STEPPED) {
    if (rssi > bestRssi) bestRssi = rssi;
    if (rssi < bestRssi - RSSI_WORSE_DB) {
      startDrive(DRIVE_RIGHT, turnMsFor(huntTurnDeg));
      huntStage = HUNT_TURNED;
      return;
    }
  } else {
    bestRssi = rssi;  // New heading: start comparing from here.
  }
  startDrive(DRIVE_FORWARD, stepMs);
  huntStage = HUNT_STEPPED;
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  // Pull-up so a missing ECHO wire reads steadily high and is caught as a dead sonar.
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT_PULLUP);
  releaseMotors();
  matrix.begin();
  showSymbol(DRIVE_STOP);
  Serial.begin(115200);
  delay(1500);
  Serial.println("Wayfinder rover ready. Wi-Fi + fixed forward sonar. Serial: c=calibrate at 1 m, s=status.");
}

void loop() {
  if (!driveFinished()) {
    // Motors running: keep the loop tight so the move ends on time. Only a
    // forward move needs the sonar, and only when a ping fits before the end.
    const uint32_t now = millis();
    if (activeDrive == DRIVE_FORWARD && now - lastPingAt >= PING_GAP_MS &&
        static_cast<int32_t>(driveEndsAt - now) > static_cast<int32_t>(PING_BUDGET_MS)) {
      frontCm = readDistanceCm(true);
      if (!forwardClear()) {
        stopDrive();
        if (mode == MANUAL) mode = BLOCKED;
      }
    }
    return;
  }

  if (braking && millis() - driveStoppedAt >= BRAKE_MS) releaseMotors();
  handleSerial();
  ensureWiFi();
  updateRssi();
  frontCm = readDistanceCm(false);
  frontSteady = millis() - driveStoppedAt >= ODO_SETTLE_MS;
  resolveOdometry();
  // Report only once the pose and the sonar reading belong to the same standstill.
  if (frontSteady && !odoPending) exchange();

  const bool autonomous = mode == WIFI_HUNT || mode == AVOIDING;
  if (autonomous && millis() - lastBackendOkAt > BACKEND_LOST_MS) {
    mode = IDLE;  // Nobody can press Stop any more.
    return;
  }
  if (activeDrive == DRIVE_STOP && autonomous) runHunt();
}
