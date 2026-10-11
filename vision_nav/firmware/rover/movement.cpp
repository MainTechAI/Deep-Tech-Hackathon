#include "movement.h"
#include "robot.h"

static int targetL = 0, targetR = 0;  // what was requested
static int outL = 0, outR = 0;        // what the motors actually do
static bool blocked = false;
static unsigned long cmdStart = 0, cmdMs = 0;
static float lastDist = 999;

// L298N: direction on IN pins, speed as PWM on the EN pin
static void motor(int en, int a, int b, int speed) {
  if (speed > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (speed < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else                { digitalWrite(a, LOW);  digitalWrite(b, LOW); }
  int s = abs(speed);
  if (USE_PWM) analogWrite(en, s == 0 ? 0 : map(s, 1, 100, MIN_PWM, 255));
  else         digitalWrite(en, s == 0 ? LOW : HIGH);
}

static int shape(int v) {
  v = constrain(v, -100, 100);
  if (USE_PWM) return abs(v) < 10 ? 0 : v;
  if (abs(v) < 40) return 0;            // no PWM: only stop / full speed
  return v > 0 ? 100 : -100;
}

static void apply() {
  int l = targetL, r = targetR;
  blocked = false;
  if (l + r > 0) {                      // has a forward component
    if (lastDist < STOP_CM) {           // too close: drop forward part, keep turning
      int spin = (l - r) / 2;
      l = spin; r = -spin;
      blocked = true;
    } else if (USE_PWM && lastDist < SLOW_CM) {
      // 35% speed at STOP_CM -> 100% at SLOW_CM
      float k = 0.35 + 0.65 * (lastDist - STOP_CM) / (SLOW_CM - STOP_CM);
      l = l * k; r = r * k;
    }
  }
  l = shape(l); r = shape(r);
  if (l == outL && r == outR) return;
  outL = l; outR = r;

  if (SWAP_SIDES) { int t = l; l = r; r = t; }
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;
  motor(ENA, IN1, IN2, l);
  motor(ENB, IN3, IN4, r);
}

void moveBegin() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motor(ENA, IN1, IN2, 0);
  motor(ENB, IN3, IN4, 0);
}

void moveDrive(int left, int right, unsigned long ms) {
  targetL = constrain(left, -100, 100);
  targetR = constrain(right, -100, 100);
  cmdStart = millis();
  cmdMs = ms;
  apply();
}

void moveJoystick(int x, int y, unsigned long ms) {
  moveDrive(y + x, y - x, ms);
}

void moveStop() { moveDrive(0, 0); }

void moveUpdate(float distCm) {
  lastDist = distCm;
  if (cmdMs && millis() - cmdStart >= cmdMs) { targetL = targetR = 0; cmdMs = 0; }
  apply();
}

int  moveLeft()         { return outL; }
int  moveRight()        { return outR; }
bool moveBlocked()      { return blocked; }
bool moveGoingForward() { return outL + outR > 0; }
bool moveGoingBack()    { return outL + outR < 0; }
