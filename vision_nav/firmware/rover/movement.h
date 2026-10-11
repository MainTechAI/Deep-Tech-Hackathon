#pragma once

// Motor API. Speeds are -100..100 (% of max), + = forward.
// Every command passes through the front safety check in moveUpdate():
// forward is slowed down near obstacles and blocked closer than STOP_CM,
// while turning on the spot and reversing are always allowed.

void moveBegin();

// ms = 0: keep going until the next command; otherwise stop after ms
void moveDrive(int left, int right, unsigned long ms = 0);

// x: -100 (left) .. 100 (right), y: -100 (back) .. 100 (forward)
void moveJoystick(int x, int y, unsigned long ms = 0);

void moveStop();

// Call every loop with the latest distance: applies safety and timeouts
void moveUpdate(float distCm);

int  moveLeft();          // actual output after safety, -100..100
int  moveRight();
bool moveBlocked();       // forward was requested but the obstacle is too close
bool moveGoingForward();
bool moveGoingBack();
