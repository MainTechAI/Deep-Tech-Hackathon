#pragma once

// "Follow me": drives toward the phone whose hotspot the robot is joined to,
// using Wi-Fi signal strength (RSSI) - hot/cold style:
//   1. scan: spin around in FOLLOW_STEPS steps, measure the signal at each heading
//   2. turn to the strongest heading and drive a short leg
//   3. signal got stronger -> another leg straight away, otherwise scan again
//   4. signal strong enough (FOLLOW_NEAR_RSSI) -> stop and wait until the phone moves away
// Obstacles: front safety always applies; if something blocks the way and the signal
// is weak, the robot backs off and turns 90 deg to go around it.

void followStart();
void followStep();          // one cycle; blocks, but keeps web/serial alive via waitMs()
const char* followState();  // "scan", "turn", "drive", "near", "avoid"
