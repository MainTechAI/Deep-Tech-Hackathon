#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>

// Wokwi Private IoT Gateway resolves this address to the computer running VS Code.
// The same JSON contract is used by the physical Uno R4 WiFi later.
constexpr char WIFI_SSID[] = "Wokwi-GUEST";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TELEMETRY_URL[] = "http://host.wokwi.internal:8000/api/telemetry";

constexpr uint8_t TRIG_PIN = 5;
constexpr uint8_t ECHO_PIN = 17;
constexpr uint8_t SERVO_PIN = 18;

Servo scanner;
int angle = 25;
int step = 5;
float sonarLeft = NAN;
float sonarFront = NAN;
float sonarRight = NAN;

float measureDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 400.0;
  return min(400.0f, duration * 0.0343f / 2.0f);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wokwi WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.println(" connected");
}

String jsonNumberOrNull(float value) {
  return isnan(value) ? "null" : String(value, 1);
}

void sendTelemetry(float distance) {
  if (WiFi.status() != WL_CONNECTED) return;

  // `scan` is one real sonar return: servo angle relative to rover centre + distance.
  // The backend converts it into a point in the global map.
  const int relativeAngle = angle - 90;
  const String payload =
      "{\"source\":\"wokwi-esp32\",\"mode\":\"SCANNING\",\"heading\":0,\"position\":{\"x\":0,\"y\":0},"
      "\"sonar\":{\"left\":" + jsonNumberOrNull(sonarLeft) +
      ",\"front\":" + jsonNumberOrNull(sonarFront) +
      ",\"right\":" + jsonNumberOrNull(sonarRight) + "},"
      "\"scan\":{\"relative_angle\":" + String(relativeAngle) +
      ",\"distance_cm\":" + String(distance, 1) + "}}";

  HTTPClient http;
  http.begin(TELEMETRY_URL);
  http.addHeader("Content-Type", "application/json");
  const int status = http.POST(payload);
  Serial.printf("POST %d  %s\n", status, payload.c_str());
  http.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  scanner.attach(SERVO_PIN);
  scanner.write(angle);
  connectWifi();
}

void loop() {
  scanner.write(angle);
  delay(220); // Give the servo enough time to reach the next ray.
  const float distance = measureDistanceCm();
  if (angle < 65) sonarLeft = distance;
  else if (angle > 115) sonarRight = distance;
  else sonarFront = distance;
  sendTelemetry(distance);

  angle += step;
  if (angle >= 155 || angle <= 25) step = -step;
  delay(180);
}
