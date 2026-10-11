#include "follow.h"
#include "robot.h"
#include "movement.h"
#include "buzzer.h"
#include "face.h"
#include "web.h"

static const char* state = "scan";
static float legStartRssi = -100;
static int legs = 0;   // legs driven since the last scan

const char* followState() { return state; }

static void setState(const char* s, float rssi) {
  state = s;
  Serial.print("FOLLOW "); Serial.print(s);
  Serial.print("  rssi "); Serial.print(rssi, 1);
  Serial.print("  dist "); Serial.println(dist, 1);
}

// Average signal over a few samples (RSSI jumps a lot). Returns -100 if the mode changed.
static float measure(int n) {
  long sum = 0;
  int k = 0;
  for (int i = 0; i < n; i++) {
    int v = wifiRssi();
    if (v < 0 && v > -100) { sum += v; k++; }
    waitMs(FOLLOW_SAMPLE_MS);
    if (mode != FOLLOW) return -100;
  }
  return k ? (float)sum / k : -100;
}

// dir: +1 = right, -1 = left. Returns false if the mode changed.
static bool spinStep(int dir) {
  moveDrive(dir * AUTO_SPEED, -dir * AUTO_SPEED);
  waitMs(FOLLOW_STEP_MS);
  moveStop();
  waitMs(FOLLOW_SETTLE_MS);
  return mode == FOLLOW;
}

static void arrived(float rssi) {
  setState("near", rssi);
  moveStop();
  showHappy();
  buzzerBeep(120, 2500); waitMs(250); buzzerBeep(120, 3200);
  // Wait here until the phone walks away
  while (mode == FOLLOW) {
    float now = measure(FOLLOW_SAMPLES);
    if (now < rssi - FOLLOW_LEAVE_DB) break;
  }
  legs = 0;
}

void followStart() {
  state = "scan";
  legs = 0;
  legStartRssi = -100;
}

void followStep() {
  float here = measure(FOLLOW_SAMPLES);
  if (mode != FOLLOW) return;
  if (here >= FOLLOW_NEAR_RSSI) { arrived(here); return; }

  bool improving = legs > 0 && legs < FOLLOW_MAX_LEGS && here >= legStartRssi + FOLLOW_GAIN_DB;
  if (improving) {
    setState("drive", here);   // getting warmer - keep going the same way
  } else {
    // Scan all around; heading 0 is the current one
    setState("scan", here);
    showRight();
    int bestStep = 0;
    float best = here;
    for (int i = 1; i < FOLLOW_STEPS; i++) {
      if (!spinStep(1)) return;
      float r = measure(FOLLOW_SAMPLES);
      if (mode != FOLLOW) return;
      Serial.print("  heading "); Serial.print(i); Serial.print(": "); Serial.println(r, 1);
      if (r > best) { best = r; bestStep = i; }
    }
    if (!spinStep(1)) return;   // back to heading 0

    // Face the strongest heading, the short way round
    setState("turn", best);
    if (bestStep <= FOLLOW_STEPS / 2) {
      showRight();
      for (int i = 0; i < bestStep; i++) if (!spinStep(1)) return;
    } else {
      showLeft();
      for (int i = 0; i < FOLLOW_STEPS - bestStep; i++) if (!spinStep(-1)) return;
    }
    if (best >= FOLLOW_NEAR_RSSI) { arrived(best); return; }
    here = best;
    legs = 0;
  }

  // Drive one leg toward the signal
  state = "drive";
  showUp();
  legStartRssi = here;
  moveDrive(FOLLOW_SPEED, FOLLOW_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)FOLLOW_LEG_MS && dist >= AVOID_CM) {
    waitMs(20);
    if (mode != FOLLOW) return;
  }
  moveStop();
  legs++;
  if (dist >= AVOID_CM) return;

  // Something in front: if the signal is strong it's probably the person
  float r = measure(FOLLOW_SAMPLES / 2);
  if (mode != FOLLOW) return;
  if (r >= FOLLOW_NEAR_RSSI - 8) { arrived(r); return; }

  // Otherwise go around it: back off, turn 90 deg, short drive, then scan again
  setState("avoid", r);
  showSurprised();
  moveDrive(-AUTO_SPEED, -AUTO_SPEED); waitMs(BACK_MS); if (mode != FOLLOW) return;
  moveDrive(AUTO_SPEED, -AUTO_SPEED);  waitMs(TURN_MS); if (mode != FOLLOW) return;
  moveDrive(FOLLOW_SPEED, FOLLOW_SPEED);
  t0 = millis();
  while (millis() - t0 < (unsigned long)FOLLOW_LEG_MS / 2 && dist >= AVOID_CM) {
    waitMs(20);
    if (mode != FOLLOW) return;
  }
  moveStop();
  legs = 0;
}
