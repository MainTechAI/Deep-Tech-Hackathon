#include <ArduinoBLE.h>
#include "Arduino_LED_Matrix.h"
#include <math.h>
#include <string.h>

const char* PHONE_UUID = "d4937a10-21fc-4f53-8bc7-4105805d99a0";
const unsigned long SIGNAL_TIMEOUT_MS = 4000;
const float PATH_LOSS_N = 2.0f; // Start here; indoors often needs 2 to 4.
const float SMOOTHING_ALPHA = 0.20f;
float rssiAtOneMetre = -59.0f; // Placeholder: calibrate with Serial command c.
float filteredRssi = 0;
float distanceMetres = NAN;
bool phoneVisible = false;
unsigned long lastSeen = 0, lastDisplay = 0;
bool calibrating = false;
unsigned long calibrationStarted = 0;
float calibrationSum = 0;
unsigned int calibrationCount = 0;
ArduinoLEDMatrix matrix;
uint8_t pixels[8][12];
const uint8_t digits[10][5] = {
  {7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7},
  {5,5,7,1,1}, {7,4,7,1,7}, {7,4,7,5,7}, {7,1,1,1,1},
  {7,5,7,5,7}, {7,5,7,1,7}
};
void drawDigit(int number, int x) {
  for (int y=0; y<5; y++)
    for (int col=0; col<3; col++)
      pixels[y+1][x+col] = (digits[number][y] >> (2-col)) & 1;
}
void showMissing() {
  memset(pixels, 0, sizeof(pixels));
  for (int x=1; x<11; x++) pixels[3][x] = (x<4 || x>7);
  matrix.renderBitmap(pixels, 8, 12);
}
void showDistance(float metres) {
  memset(pixels, 0, sizeof(pixels));
  int tenths = (int)roundf(metres * 10);
  if (tenths > 999) {
    // Three horizontal bars indicate above display range, not 99.9.
    for (int x=1; x<11; x++) pixels[1][x]=pixels[3][x]=pixels[5][x]=1;
  } else {
    if (tenths >= 100) drawDigit(tenths/100, 0);
    drawDigit((tenths/10)%10, 4);
    drawDigit(tenths%10, 9);
    pixels[5][8] = 1; // decimal point; read 2.4 as metres
  }
  matrix.renderBitmap(pixels, 8, 12);
}
void onDiscovered(BLEDevice device) {
  bool matches = false;
  for (int i=0; i<device.advertisedServiceUuidCount(); i++)
    if (device.advertisedServiceUuid(i).equalsIgnoreCase(PHONE_UUID)) matches = true;
  if (!matches) return;
  int rssi = device.rssi();
  if (rssi >= 0 || rssi < -127) return;
  unsigned long now = millis();
  bool fresh = !phoneVisible || now-lastSeen > SIGNAL_TIMEOUT_MS;
  filteredRssi = fresh ? (float)rssi :
    SMOOTHING_ALPHA*rssi + (1-SMOOTHING_ALPHA)*filteredRssi;
  lastSeen = now;
  phoneVisible = true;
  if (calibrating) { calibrationSum += rssi; calibrationCount++; }
}

void printStatus() {
  Serial.println("BLE scanner is active.");
  Serial.print("Target service UUID: "); Serial.println(PHONE_UUID);
  Serial.print("Phone visible: "); Serial.println(phoneVisible ? "yes" : "no");
  if (phoneVisible) {
    Serial.print("Filtered RSSI: "); Serial.print(filteredRssi, 1);
    Serial.println(" dBm");
  }
  Serial.println("Use c to calibrate at 1 metre. The phone must advertise this service UUID.");
}

void setup() {
  Serial.begin(115200);

  // Allow time to open Serial Monitor, but still run without USB.
  unsigned long started = millis();
  while (!Serial && millis() - started < 3000) {
    delay(10);
  }

  // The ESP32-S3 connectivity module may still be booting when USB Serial
  // becomes ready. Give it a fixed settling time before the first BLE AT call.
  delay(2000);

  Serial.println("Starting Bluetooth before LED matrix...");

  if (!BLE.begin()) {
    Serial.println("BLE.begin() FAILED with matrix not started.");
    Serial.println("Check board package, connectivity firmware, and BLE library.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Bluetooth initialized successfully.");

  matrix.begin();
  showMissing();
  Serial.println("LED matrix initialized.");

  BLE.setEventHandler(BLEDiscovered, onDiscovered);

  if (!BLE.scanForUuid(PHONE_UUID, true)) {
    Serial.println("Scan start failed.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Scanning. At 1 metre send c for 5-second calibration.");
}

void loop() {
  BLE.poll();
  unsigned long now = millis();
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's' || command == 'S') {
      printStatus();
    } else if (command == 'c' || command == 'C') {
      calibrating = true;
      calibrationStarted = now;
      calibrationSum = 0;
      calibrationCount = 0;
      Serial.println("Keep phone 1 metre away, stationary, for 5 seconds.");
    }
  }
  if (calibrating && now-calibrationStarted >= 5000) {
    calibrating = false;
    if (calibrationCount >= 5) {
      rssiAtOneMetre = calibrationSum/calibrationCount;
      Serial.print("RSSI_AT_1M = "); Serial.println(rssiAtOneMetre, 2);
      Serial.println("To keep calibration after reboot, put this value in rssiAtOneMetre.");
    } else Serial.println("Calibration failed: too few packets. Start the phone beacon.");
  }
  if (phoneVisible && now-lastSeen > SIGNAL_TIMEOUT_MS) phoneVisible = false;
  distanceMetres = phoneVisible ?
    powf(10.0f, (rssiAtOneMetre-filteredRssi)/(10.0f*PATH_LOSS_N)) : NAN;
  if (now-lastDisplay >= 250) {
    lastDisplay = now;
    if (phoneVisible) {
      showDistance(distanceMetres);
      Serial.print("RSSI="); Serial.print(filteredRssi, 1);
      Serial.print(" dBm, distance="); Serial.print(distanceMetres, 2);
      Serial.println(" m");
    } else showMissing();
  }
  // Future motor controller: use phoneVisible and distanceMetres.
  // RSSI has no bearing information. Stop motors when phoneVisible is false.
}
