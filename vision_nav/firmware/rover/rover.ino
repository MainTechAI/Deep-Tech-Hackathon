// Obstacle-avoiding robot: Wi-Fi joystick (with front collision protection) + autonomous mode
// Phone: connect to Wi-Fi "RobotCar" (pass robot1234), open http://192.168.4.1
// Pins and settings: robot.h | HTTP API: web.h | motor API: movement.h
// Serial Monitor 115200, "Newline": w/a/s/d move | x stop | m AUTO/MANUAL | f FOLLOW

#include "robot.h"
#include "movement.h"
#include "sensor.h"
#include "buzzer.h"
#include "face.h"
#include "web.h"
#include "follow.h"

Mode mode = MANUAL;
float dist = 999;
bool turnRightNext = true;
bool showingIp = true;   // keep the IP number on the matrix until the robot first moves

const char* modeName() {
  return mode == AUTO ? "AUTO" : mode == FOLLOW ? "FOLLOW" : "MANUAL";
}

void setMode(Mode m) {
  if (m == FOLLOW && !wifiIsSta()) {
    Serial.println("FOLLOW needs the robot joined to the phone hotspot (arduino_secrets.h)");
    buzzerBeep(400, 600);
    m = MANUAL;
  }
  mode = m;
  showingIp = false;
  moveStop();
  Serial.print("MODE: "); Serial.println(modeName());
  if (mode == MANUAL)    { showStop();  buzzerBeep(80, 1500); }
  else if (mode == AUTO) { showHappy(); buzzerBeep(80, 3000); }
  else                   { followStart(); showHappy(); buzzerBeep(80, 2500); }
}

void handleKey(char k) {
  if (k == 'm') { setMode(mode == MANUAL ? AUTO : MANUAL); return; }
  if (k == 'x') { setMode(MANUAL); Serial.println("STOP"); return; }
  if (k == 'f') { setMode(mode == FOLLOW ? MANUAL : FOLLOW); return; }
  if (mode != MANUAL) return;
  switch (k) {
    case 'w': moveDrive( 100,  100); Serial.println("FORWARD");    break;
    case 's': moveDrive(-100, -100); Serial.println("BACKWARD");   break;
    case 'a': moveDrive(-100,  100); Serial.println("SPIN LEFT");  break;
    case 'd': moveDrive( 100, -100); Serial.println("SPIN RIGHT"); break;
    default: break;
  }
}

void checkSerial() {
  while (Serial.available()) handleKey(tolower(Serial.read()));
}

// Parking-sensor beeps: faster as the obstacle gets closer; slow beeps when reversing
// (there is no rear sensor, so we warn people behind the robot)
void updateWarnings() {
  if (moveBlocked())                            buzzerPattern(100);
  else if (moveGoingForward() && dist < WARN_CM)
    buzzerPattern(map((long)dist, (long)STOP_CM, (long)WARN_CM, 120, 450));
  else if (moveGoingBack())                     buzzerPattern(600);
  else                                          buzzerPattern(0);
}

// Everything that must keep running, even during AUTO / FOLLOW maneuvers
void service() {
  checkSerial();
  webPoll();
  updateWarnings();
  buzzerUpdate();
}

void waitMs(int ms) {
  Mode m0 = mode;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    service();
    dist = sensorRead();
    moveUpdate(dist);
    if (mode != m0) return;
  }
}

void manualStep() {
  if (showingIp) {
    if (moveLeft() == 0 && moveRight() == 0) return;
    showingIp = false;
  }
  if (moveBlocked()) {
    static unsigned long lastMsg = 0;
    showSurprised();
    if (millis() - lastMsg > 500) {
      lastMsg = millis();
      Serial.print("BLOCKED - obstacle at "); Serial.print(dist, 1); Serial.println(" cm");
    }
  } else {
    showMotion(moveLeft(), moveRight());
  }
}

void autoStep() {
  if (dist > AVOID_CM) {
    if (!moveGoingForward()) moveDrive(AUTO_SPEED, AUTO_SPEED);
    showHappy();
    return;
  }
  Serial.print("AUTO: obstacle at "); Serial.print(dist, 1); Serial.println(" cm");
  showSurprised();
  moveStop();                             waitMs(150);     if (mode != AUTO) return;
  moveDrive(-AUTO_SPEED, -AUTO_SPEED);    waitMs(BACK_MS); if (mode != AUTO) return;
  if (turnRightNext) { moveDrive(AUTO_SPEED, -AUTO_SPEED); showRight(); }
  else               { moveDrive(-AUTO_SPEED, AUTO_SPEED); showLeft(); }
  waitMs(TURN_MS); if (mode != AUTO) return;
  moveStop();      waitMs(100);
  turnRightNext = !turnRightNext;
  dist = sensorRefresh();   // forget readings taken before the turn
}

void setup() {
  moveBegin();
  sensorBegin();
  buzzerBegin();
  faceBegin();
  showStop();
  Serial.begin(115200);
  delay(1500);
  webBegin();
  dist = sensorRefresh();
  buzzerBeep(100, 2000);
  Serial.println("Robot ready. w/a/s/d move, x stop, m = AUTO/MANUAL, f = FOLLOW");
}

unsigned long lastPrint = 0;

void loop() {
  service();
  dist = sensorRead();
  moveUpdate(dist);

  if (mode == MANUAL)    manualStep();
  else if (mode == AUTO) autoStep();
  else                   followStep();

  if (millis() - lastPrint > 300) {
    lastPrint = millis();
    Serial.print("dist: ");
    if (dist >= 999) Serial.println("clear");
    else { Serial.print(dist, 1); Serial.println(" cm"); }
  }
}
