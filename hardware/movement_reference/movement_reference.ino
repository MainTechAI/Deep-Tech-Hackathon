// Obstacle-avoiding robot: manual (with collision protection) + autonomous mode
// L298N:   ENA D5, IN1 D9, IN2 D10 (left) | ENB D6, IN3 D11, IN4 D12 (right)
// HC-SR04: Trig D3, Echo D2
// Serial Monitor 115200, "Newline": w/a/s/d move | x stop | m AUTO/MANUAL

#include "Arduino_LED_Matrix.h"

const int ENA = 5,  IN1 = 9,  IN2 = 10;
const int ENB = 6,  IN3 = 11, IN4 = 12;
const int TRIG_PIN = 3, ECHO_PIN = 2;

const float SAFE_CM  = 15.0;  // manual: blocks forward if closer
const float AVOID_CM = 20.0;  // auto: start avoidance if closer
const int   BACK_MS  = 350;   // auto: back-up time
const int   TURN_MS  = 450;   // auto: spin time (tune for ~90 deg)

// Direction fixes - change these instead of rewiring motors
const bool INVERT_LEFT  = true;   // flip left motor direction
const bool INVERT_RIGHT = true;   // flip right motor direction
const bool SWAP_SIDES   = false;  // true if a/d turn the wrong way

enum Mode { MANUAL, AUTO };
Mode mode = MANUAL;
int curLeft = 0, curRight = 0;
bool turnRightNext = true;

ArduinoLEDMatrix matrix;

uint8_t HAPPY[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};
uint8_t SURPRISED[8][12] = {
  {0,1,1,1,0,0,0,0,1,1,1,0},
  {0,1,0,1,0,0,0,0,1,0,1,0},
  {0,1,1,1,0,0,0,0,1,1,1,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0}
};
uint8_t ARROW_UP[8][12] = {
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,0,1,1,0,1,1,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0}
};
uint8_t ARROW_LEFT[8][12] = {
  {0,0,0,1,0,0,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,0,0,0,0},
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {0,0,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,1,0,0,0,0,0,0,0,0}
};
uint8_t STOP_X[8][12] = {
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0}
};
uint8_t frameBuf[8][12];
int shown = -1;

void show(uint8_t img[8][12], int id) {
  if (shown == id) return;
  matrix.renderBitmap(img, 8, 12);
  shown = id;
}
void showHappy()     { show(HAPPY, 1); }
void showSurprised() { show(SURPRISED, 2); }
void showUp()        { show(ARROW_UP, 3); }
void showLeft()      { show(ARROW_LEFT, 4); }
void showStop()      { show(STOP_X, 5); }
void showDown() {
  if (shown == 6) return;
  for (int r = 0; r < 8; r++) for (int c = 0; c < 12; c++) frameBuf[r][c] = ARROW_UP[7 - r][c];
  matrix.renderBitmap(frameBuf, 8, 12); shown = 6;
}
void showRight() {
  if (shown == 7) return;
  for (int r = 0; r < 8; r++) for (int c = 0; c < 12; c++) frameBuf[r][c] = ARROW_LEFT[r][11 - c];
  matrix.renderBitmap(frameBuf, 8, 12); shown = 7;
}

void motor(int en, int a, int b, int dir) {
  if (dir > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  digitalWrite(en, HIGH); }
  else if (dir < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); digitalWrite(en, HIGH); }
  else              { digitalWrite(a, LOW);  digitalWrite(b, LOW);  digitalWrite(en, LOW); }
}

void drive(int left, int right) {
  curLeft = left; curRight = right;
  int l = left, r = right;
  if (SWAP_SIDES) { int t = l; l = r; r = t; }
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;
  motor(ENA, IN1, IN2, l);
  motor(ENB, IN3, IN4, r);
}
bool movingForward() { return curLeft > 0 && curRight > 0; }

float readOnceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 25000);
  if (t == 0) return -1;
  return t * 0.0343 / 2.0;
}
float readDistanceCm() {
  float v[3]; int n = 0;
  for (int i = 0; i < 3; i++) {
    float d = readOnceCm();
    if (d > 0 && d < 400) v[n++] = d;
    delay(5);
  }
  if (n == 0) return 999;
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[n / 2];
}

void handleKey(char k) {
  if (k == 'm') {
    mode = (mode == MANUAL) ? AUTO : MANUAL;
    drive(0, 0);
    Serial.println(mode == AUTO ? "MODE: AUTO" : "MODE: MANUAL");
    if (mode == MANUAL) showStop();
    return;
  }
  if (k == 'x') {
    mode = MANUAL;
    drive(0, 0); showStop(); Serial.println("STOP");
    return;
  }
  if (mode != MANUAL) return;
  switch (k) {
    case 'w': drive( 1,  1); showUp();    Serial.println("FORWARD");    break;
    case 's': drive(-1, -1); showDown();  Serial.println("BACKWARD");   break;
    case 'a': drive(-1,  1); showLeft();  Serial.println("SPIN LEFT");  break;
    case 'd': drive( 1, -1); showRight(); Serial.println("SPIN RIGHT"); break;
    default: break;
  }
}

void checkSerial() {
  while (Serial.available()) handleKey(tolower(Serial.read()));
}

void waitMs(int ms) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    checkSerial();
    if (mode != AUTO) return;
  }
}

void manualStep(float d) {
  if (movingForward() && d < SAFE_CM) {
    drive(0, 0);
    showSurprised();
    Serial.print("BLOCKED - obstacle at ");
    Serial.print(d, 1);
    Serial.println(" cm");
  }
}

void autoStep(float d) {
  if (d > AVOID_CM) {
    if (!movingForward()) drive(1, 1);
    showHappy();
    return;
  }
  Serial.print("AUTO: obstacle at "); Serial.print(d, 1); Serial.println(" cm");
  showSurprised();
  drive(0, 0);   waitMs(150);     if (mode != AUTO) return;
  drive(-1, -1); waitMs(BACK_MS); if (mode != AUTO) return;
  if (turnRightNext) { drive(1, -1); showRight(); }
  else               { drive(-1, 1); showLeft(); }
  waitMs(TURN_MS); if (mode != AUTO) return;
  drive(0, 0);     waitMs(100);
  turnRightNext = !turnRightNext;
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  drive(0, 0);
  matrix.begin();
  showStop();
  Serial.begin(115200);
  delay(1500);
  Serial.println("Robot ready. w/a/s/d move, x stop, m = AUTO/MANUAL");
}

unsigned long lastPrint = 0;

void loop() {
  checkSerial();
  float d = readDistanceCm();

  if (mode == MANUAL) manualStep(d);
  else                autoStep(d);

  if (millis() - lastPrint > 300) {
    lastPrint = millis();
    Serial.print("dist: ");
    if (d >= 999) Serial.println("clear");
    else { Serial.print(d, 1); Serial.println(" cm"); }
  }
}