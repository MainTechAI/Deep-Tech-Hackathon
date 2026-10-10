/*
  UNO R4 WiFi + HC-SR04 + SG90 local polar scanner.
  Standalone scanner: it DOES NOT control or stop wheel motors.
  Keep the robot stationary, wheels disabled, during acquisition.
  Wiring: servo signal D10; sonar TRIG D11; sonar ECHO D2 (interrupt).
  HC-SR04 VCC -> UNO 5V. Servo -> separate regulated 5V supply.
  Join all grounds. Never connect a 2S battery directly to 5V loads.

  Arduino IDE: board "Arduino UNO R4 WiFi", library "Servo" by Arduino.
  USB serial 115200: p=pause, r=resume, c=clear map. Resume clears map.
  JSON Lines protocol is documented in README.md. Viewer: viewer.py.
  Coordinates are centimetres: x=right, y=forward, sensor at origin.
  Servo angle 90 is forward. Set MIRROR_SCAN if left/right are reversed.
  This is a range/return map, NOT SLAM or a collision-certified free-space map.
*/
#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t SERVO_PIN = 10;
constexpr uint8_t TRIG_PIN = 11;
constexpr uint8_t ECHO_PIN = 2; // Interrupt-capable on UNO R4; NOT the old D12.

// Start away from mechanical end stops. After testing the mounting, set
// SCAN_MIN_DEG=0 and SCAN_MAX_DEG=180 for a nominal full half-circle.
constexpr int SCAN_MIN_DEG = 10;
constexpr int SCAN_MAX_DEG = 170;
constexpr int SCAN_STEP_DEG = 10;
constexpr bool MIRROR_SCAN = false;
// Conservative initial pulse limits. Calibrate physical angles/range on YOUR
// servo. Commanded degrees are not measured shaft angles. Do not force stops.
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_MAX_US = 2000;
constexpr uint32_t SETTLE_MS = 160; // Increase if the loaded bracket still moves.
constexpr uint32_t INITIAL_SETTLE_MS = 800;
constexpr uint32_t PING_INTERVAL_US = 65000; // Datasheet recommends >60ms.
constexpr uint32_t ECHO_TIMEOUT_US = 25000;
constexpr uint16_t MIN_MM = 20;
constexpr uint16_t MAX_MM = 2500; // Deliberately conservative display limit.
constexpr uint8_t SAMPLES_PER_ANGLE = 3;
constexpr uint8_t MIN_VALID_SAMPLES = 2;
constexpr int BIN_COUNT = (SCAN_MAX_DEG - SCAN_MIN_DEG) / SCAN_STEP_DEG + 1;
static_assert(SCAN_STEP_DEG > 0 && SCAN_MIN_DEG >= 0 && SCAN_MAX_DEG <= 180,
              "Invalid sweep configuration");
static_assert(SCAN_MIN_DEG < SCAN_MAX_DEG &&
              (SCAN_MAX_DEG - SCAN_MIN_DEG) % SCAN_STEP_DEG == 0,
              "Sweep must contain whole steps");

struct MapBin {
  uint16_t distanceMm;
  uint32_t measuredAtMs;
  uint8_t validSamples;
  bool measured;
  bool valid;
};
MapBin localMap[BIN_COUNT] = {};
Servo scanner;

enum Phase : uint8_t { SETTLING, WAIT_PING, WAIT_ECHO, PUBLISH, PAUSED };
Phase phase = SETTLING;
int binIndex = 0;
int sweepDirection = 1;
uint32_t sweepNumber = 0;
uint32_t epoch = 0;
uint32_t movedAtMs = 0;
uint32_t settleDurationMs = INITIAL_SETTLE_MS;
uint32_t lastPingUs = 0;
uint32_t pingStartedUs = 0;
uint8_t sampleCount = 0;
uint8_t validCount = 0;
uint16_t samples[SAMPLES_PER_ANGLE] = {};
bool pendingPoint = false;
bool pendingReset = true;
bool pendingStatus = false;

volatile bool echoArmed = false;
volatile bool echoRose = false;
volatile bool echoDone = false;
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoWidthUs = 0;

// ISR records timestamps only. No Serial, Servo commands or allocations here.
void echoChanged() {
  if (!echoArmed) return;
  const uint32_t now = micros();
  if (digitalRead(ECHO_PIN) == HIGH) {
    if (!echoRose) { echoRiseUs = now; echoRose = true; }
  } else if (echoRose) {
    echoWidthUs = now - echoRiseUs;
    echoDone = true;
    echoArmed = false;
  }
}

void cancelEcho() {
  noInterrupts();
  echoArmed = false;
  echoRose = false;
  echoDone = false;
  interrupts();
  digitalWrite(TRIG_PIN, LOW);
}

void clearMap() {
  for (int i = 0; i < BIN_COUNT; ++i) localMap[i] = {};
  ++epoch;
  pendingReset = true;
}

void moveToBin(uint32_t settlingMs) {
  scanner.write(SCAN_MIN_DEG + binIndex * SCAN_STEP_DEG);
  movedAtMs = millis();
  settleDurationMs = settlingMs;
  sampleCount = validCount = 0;
  phase = SETTLING;
}

void advanceScan() {
  if ((sweepDirection > 0 && binIndex == BIN_COUNT - 1) ||
      (sweepDirection < 0 && binIndex == 0)) {
    sweepDirection = -sweepDirection;
    ++sweepNumber;
  }
  binIndex += sweepDirection; // Endpoint is sampled once, then reverse.
  moveToBin(SETTLE_MS);
}

void finishSample(bool received, uint32_t widthUs) {
  // 343m/s at approximately 20C; duration includes outward + return paths.
  const uint32_t mm = (widthUs * 343UL + 1000UL) / 2000UL;
  if (received && widthUs <= ECHO_TIMEOUT_US && mm >= MIN_MM && mm <= MAX_MM)
    samples[validCount++] = static_cast<uint16_t>(mm);
  ++sampleCount;
  if (sampleCount < SAMPLES_PER_ANGLE) { phase = WAIT_PING; return; }

  for (uint8_t i = 1; i < validCount; ++i) {
    uint16_t value = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > value) { samples[j + 1] = samples[j]; --j; }
    samples[j + 1] = value;
  }
  MapBin &b = localMap[binIndex];
  b.measured = true;
  b.valid = validCount >= MIN_VALID_SAMPLES;
  b.validSamples = validCount;
  b.measuredAtMs = millis();
  // Median of 3. With 2 valid returns, keep nearer one (conservative).
  b.distanceMm = b.valid ? samples[(validCount - 1) / 2] : 0;
  pendingPoint = true;
  phase = PUBLISH;
}

// Incremental TX avoids waiting for an entire record to fit in the USB buffer.
// Scanning does not depend on a viewer being connected.
char tx[240];
size_t txLength = 0, txOffset = 0;
void serviceSerial() {
  if (!Serial) {
    txLength = txOffset = 0;
    return;
  }
  if (txOffset == txLength) {
    txLength = txOffset = 0;
    int n = 0;
    if (pendingReset) {
      n = snprintf(tx, sizeof(tx),
        "{\"type\":\"reset\",\"epoch\":%lu,\"min\":%d,\"max\":%d,\"step\":%d,\"max_cm\":%u}\n",
        (unsigned long)epoch, SCAN_MIN_DEG, SCAN_MAX_DEG, SCAN_STEP_DEG, MAX_MM / 10);
      pendingReset = false;
    } else if (pendingStatus) {
      n = snprintf(tx, sizeof(tx), "{\"type\":\"status\",\"paused\":%s}\n",
                   phase == PAUSED ? "true" : "false");
      pendingStatus = false;
    } else if (pendingPoint) {
      const MapBin &b = localMap[binIndex];
      const int angle = SCAN_MIN_DEG + binIndex * SCAN_STEP_DEG;
      const int bearing = (MIRROR_SCAN ? -1 : 1) * (90 - angle);
      n = snprintf(tx, sizeof(tx),
        "{\"type\":\"point\",\"epoch\":%lu,\"sweep\":%lu,\"angle\":%d,\"bearing\":%d,\"mm\":%u,\"valid\":%s,\"samples\":%u,\"ms\":%lu}\n",
        (unsigned long)epoch, (unsigned long)sweepNumber, angle, bearing,
        b.distanceMm, b.valid ? "true" : "false", b.validSamples,
        (unsigned long)b.measuredAtMs);
      pendingPoint = false;
    }
    if (n > 0 && static_cast<size_t>(n) < sizeof(tx)) txLength = n;
  }
  if (txOffset < txLength) {
    int room = Serial.availableForWrite();
    if (room > 0) {
      size_t count = txLength - txOffset;
      if (count > static_cast<size_t>(room)) count = room;
      txOffset += Serial.write(reinterpret_cast<const uint8_t *>(tx) + txOffset, count);
    }
  }
}

void handleCommands() {
  // Bound work per loop, even if a host sends excessive input.
  for (uint8_t i = 0; i < 8 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c != 'p' && c != 'r' && c != 'c') continue;
    const bool wasPaused = phase == PAUSED;
    cancelEcho();
    pendingPoint = false;
    sampleCount = validCount = 0;
    if (c == 'p') {
      phase = PAUSED; // Servo continues holding its current orientation.
    } else {
      clearMap();
      if (c == 'c' && wasPaused) phase = PAUSED;
      else moveToBin(SETTLE_MS);
    }
    pendingStatus = true;
  }
}

void setup() {
  Serial.begin(115200); // No while(!Serial): autonomous startup.
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoChanged, CHANGE);
  scanner.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  lastPingUs = micros() - PING_INTERVAL_US;
  clearMap();
  moveToBin(INITIAL_SETTLE_MS);
}

void loop() {
  handleCommands();
  if (phase == SETTLING && (uint32_t)(millis() - movedAtMs) >= settleDurationMs)
    phase = WAIT_PING;

  if (phase == WAIT_PING && (uint32_t)(micros() - lastPingUs) >= PING_INTERVAL_US) {
    lastPingUs = pingStartedUs = micros();
    if (digitalRead(ECHO_PIN) == HIGH) {
      finishSample(false, 0); // Stuck-high/late echo: unknown, never free space.
    } else {
      noInterrupts();
      echoRose = echoDone = false;
      echoArmed = true;
      interrupts();
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10); // Only intentional wait; not a blocking echo read.
      digitalWrite(TRIG_PIN, LOW);
      phase = WAIT_ECHO;
    }
  }
  if (phase == WAIT_ECHO) {
    noInterrupts();
    bool done = echoDone;
    uint32_t width = echoWidthUs;
    if (done) echoDone = false;
    interrupts();
    if (done) finishSample(true, width);
    else if ((uint32_t)(micros() - pingStartedUs) >= ECHO_TIMEOUT_US) {
      cancelEcho();
      finishSample(false, 0);
    }
  }
  // Do not spend time on USB while timing an echo.
  if (phase != WAIT_ECHO) serviceSerial();
  if (phase == PUBLISH) {
    // USB can lag/disconnect: drop telemetry rather than stop sensing.
    pendingPoint = false;
    advanceScan();
  }
}
