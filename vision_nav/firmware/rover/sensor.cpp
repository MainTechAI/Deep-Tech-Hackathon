#include "sensor.h"
#include "robot.h"

static float hist[3] = {999, 999, 999};
static int histIdx = 0;

static float readOnceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 25000);
  if (t == 0) return -1;
  return t * 0.0343 / 2.0;
}

void sensorBegin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

// One ping per call keeps the loop fast enough for Wi-Fi
float sensorRead() {
  float d = readOnceCm();
  hist[histIdx] = (d > 0 && d < 400) ? d : 999;
  histIdx = (histIdx + 1) % 3;
  float a = hist[0], b = hist[1], c = hist[2];
  return max(min(a, b), min(max(a, b), c));
}

float sensorRefresh() {
  float d = 999;
  for (int i = 0; i < 3; i++) { d = sensorRead(); delay(5); }
  return d;
}
