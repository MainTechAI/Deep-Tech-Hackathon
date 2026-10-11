#pragma once
#include <Arduino.h>

// ================= Pins =================
// L298N:   ENA D5, IN1 D9, IN2 D10 (left) | ENB D6, IN3 D11, IN4 D12 (right)
// HC-SR04: Trig D3, Echo D2
// Buzzer:  D4
// Free:    D7, D8, D13 (e.g. servo for the sensor later)
const int ENA = 5,  IN1 = 9,  IN2 = 10;
const int ENB = 6,  IN3 = 11, IN4 = 12;
const int TRIG_PIN = 3, ECHO_PIN = 2;
const int BUZZER_PIN = 4;

// ================= Motors =================
// Speed control: PWM on ENA (D5) and ENB (D6), direction on IN1-IN4.
// If motors behave strangely, set USE_PWM = false (full speed only).
const bool USE_PWM = true;
const int  MIN_PWM = 110;         // lowest PWM at which the motors still turn

// Direction fixes - change these instead of rewiring motors
const bool INVERT_LEFT  = true;   // flip left motor direction
const bool INVERT_RIGHT = true;   // flip right motor direction
const bool SWAP_SIDES   = false;  // true if left/right turn the wrong way

// ================= Front safety =================
const float STOP_CM = 20.0;  // forward blocked closer than this (turning/back still allowed)
const float SLOW_CM = 50.0;  // forward speed is reduced from here (PWM only)
const float WARN_CM = 45.0;  // buzzer "parking sensor" starts here

// ================= Buzzer =================
const bool BUZZER_PASSIVE = true;  // passive buzzer (needs tone). false = active buzzer

// ================= Auto mode =================
const float AVOID_CM   = 25.0;  // start avoidance if closer
const int   AUTO_SPEED = 70;    // % speed
const int   BACK_MS    = 350;   // back-up time
const int   TURN_MS    = 225;   // spin time (tune for ~90 deg)

// ================= Follow me =================
// Works only when the robot is joined to the phone's hotspot (arduino_secrets.h):
// RSSI is then the signal strength from that phone. Watch Serial to tune.
const int   FOLLOW_STEPS     = 8;            // headings per scan (8 = every 45 deg)
const int   FOLLOW_STEP_MS   = TURN_MS / 2;  // spin time for one heading step
const int   FOLLOW_SPEED     = 60;           // % speed
const int   FOLLOW_LEG_MS    = 1500;         // drive this long toward the signal, then re-check
const int   FOLLOW_MAX_LEGS  = 3;            // legs in a row without re-scanning while it gets warmer
const int   FOLLOW_SAMPLES   = 8;            // RSSI samples averaged per measurement
const int   FOLLOW_SAMPLE_MS = 60;
const int   FOLLOW_SETTLE_MS = 150;          // pause after each spin step before measuring
const int   FOLLOW_NEAR_RSSI = -38;          // "arrived" when the signal is this strong (tune!)
const float FOLLOW_GAIN_DB   = 2.0;          // keep driving straight if signal improved this much
const float FOLLOW_LEAVE_DB  = 6.0;          // start following again once signal drops this much

// ================= Wi-Fi =================
// First tries to join STA_SSID from arduino_secrets.h (IP is shown on the LED matrix
// and in Serial). If that fails or it's empty, starts its own access point:
const char AP_SSID[] = "RobotCar";
const char AP_PASS[] = "robot1234";        // min 8 characters, open http://192.168.4.1
const unsigned long STA_CONNECT_MS = 12000;
const unsigned long WEB_TIMEOUT_MS = 1000; // web drive commands stop after this

// ================= Shared state (defined in robot_wifi.ino) =================
enum Mode { MANUAL, AUTO, FOLLOW };
extern Mode mode;
extern float dist;
void setMode(Mode m);
const char* modeName();
void waitMs(int ms);   // keeps sensor, safety, web and serial running; returns early if mode changes
