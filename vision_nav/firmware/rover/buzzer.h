#pragma once

void buzzerBegin();
void buzzerBeep(int ms, int freq = 2000);  // one beep, non-blocking
void buzzerPattern(int periodMs);          // repeating beeps: 0 = off, -1 = continuous
void buzzerUpdate();                       // call every loop
