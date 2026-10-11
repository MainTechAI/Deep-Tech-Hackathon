#include "buzzer.h"
#include "robot.h"

static const int PATTERN_FREQ = 2500;
static const int BEEP_LEN = 60;

static bool isOn = false;
static int  onFreq = 0;
static unsigned long shotStart = 0;
static int  shotMs = 0, shotFreq = 0;
static int  period = 0;

static void out(bool on, int freq) {
  if (on == isOn && (!on || freq == onFreq)) return;
  isOn = on; onFreq = freq;
  if (BUZZER_PASSIVE) { if (on) tone(BUZZER_PIN, freq); else noTone(BUZZER_PIN); }
  else digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void buzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerBeep(int ms, int freq) {
  shotStart = millis(); shotMs = ms; shotFreq = freq;
  out(true, freq);
}

void buzzerPattern(int periodMs) { period = periodMs; }

void buzzerUpdate() {
  unsigned long now = millis();
  if (shotMs) {
    if (now - shotStart < (unsigned long)shotMs) { out(true, shotFreq); return; }
    shotMs = 0;
  }
  bool on;
  if (period < 0)       on = true;
  else if (period == 0) on = false;
  else                  on = (now % period) < (unsigned long)min(BEEP_LEN, period / 2);
  out(on, PATTERN_FREQ);
}
