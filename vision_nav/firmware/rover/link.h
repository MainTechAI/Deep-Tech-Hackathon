#pragma once

// Persistent TCP link to the vision-nav server (SERVER_HOST:SERVER_PORT in arduino_secrets.h).
// The robot connects out, so it works from any network with internet - no port forwarding.
//
//   robot -> server   HELLO <token>
//                     T <dist_cm> <left%> <right%> <mode> <rssi> <blocked>     (10 Hz)
//   server -> robot   D <left%> <right%> <ms>     drive (MANUAL mode only), expires after ms
//                     S                           stop + MANUAL mode
//                     B <ms> <freq>               beep
//
// Front safety still applies to every drive command. If the link drops, the last command
// simply expires and the robot stops.

void linkBegin();
void linkPoll();        // call from service(): reconnects, reads commands, sends telemetry
bool linkConnected();
