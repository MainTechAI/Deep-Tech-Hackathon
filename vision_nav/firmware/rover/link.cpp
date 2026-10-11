#include "link.h"
#include "robot.h"
#include "movement.h"
#include "buzzer.h"
#include "web.h"
#include "arduino_secrets.h"
#include <WiFiS3.h>

#ifndef SERVER_HOST
#define SERVER_HOST ""
#endif
#ifndef SERVER_PORT
#define SERVER_PORT 9000
#endif
#ifndef ROBOT_TOKEN
#define ROBOT_TOKEN ""
#endif

static WiFiClient client;
static bool connected = false;
static unsigned long lastTry = 0, lastCheck = 0, lastTel = 0, lastRead = 0;
static char line[64];
static int lineLen = 0;

bool linkConnected() { return connected; }

void linkBegin() {
  if (strlen(SERVER_HOST) == 0) Serial.println("Server link off (SERVER_HOST empty)");
}

static void handleLine(char* s) {
  if (s[0] == 'D') {
    int l = 0, r = 0, ms = 0;
    if (sscanf(s + 1, "%d %d %d", &l, &r, &ms) == 3 && mode == MANUAL)
      moveDrive(l, r, constrain(ms, 1, 2000));
  } else if (s[0] == 'S') {
    setMode(MANUAL);
  } else if (s[0] == 'B') {
    int ms = 150, f = 2000;
    sscanf(s + 1, "%d %d", &ms, &f);
    buzzerBeep(constrain(ms, 10, 2000), f);
  }
}

static void tryConnect() {
  lastTry = millis();
  // connecting blocks the loop for a moment: never do it while driving
  if (moveLeft() != 0 || moveRight() != 0) moveStop();
  Serial.print("Server: connecting to "); Serial.print(SERVER_HOST);
  Serial.print(":"); Serial.println(SERVER_PORT);
  if (client.connect(SERVER_HOST, SERVER_PORT)) {
    char hello[48];
    snprintf(hello, sizeof(hello), "HELLO %s\n", ROBOT_TOKEN);
    client.write((const uint8_t*)hello, strlen(hello));
    connected = true;
    lineLen = 0;
    Serial.println("Server: connected");
    buzzerBeep(60, 3000);
  } else {
    Serial.println("Server: connect failed");
  }
}

void linkPoll() {
  if (strlen(SERVER_HOST) == 0 || !wifiIsSta()) return;
  unsigned long now = millis();

  if (!connected) {
    if (now - lastTry > 3000) tryConnect();
    return;
  }
  // every WiFiClient call is a round-trip to the Wi-Fi chip, so poll at a sane rate
  if (now - lastCheck > 1000) {
    lastCheck = now;
    if (!client.connected()) {
      connected = false;
      client.stop();
      Serial.println("Server: disconnected");
      return;
    }
  }

  if (now - lastRead >= 20) {
    lastRead = now;
    int a = client.available();
    if (a > 0) {
      uint8_t buf[128];
      int n = client.read(buf, min(a, (int)sizeof(buf)));
      for (int i = 0; i < n; i++) {
        char ch = buf[i];
        if (ch == '\n') {
          line[lineLen] = 0;
          handleLine(line);
          lineLen = 0;
        } else if (ch != '\r' && lineLen < (int)sizeof(line) - 1) {
          line[lineLen++] = ch;
        }
      }
    }
  }

  if (now - lastTel >= 100) {
    lastTel = now;
    char t[64];
    int n = snprintf(t, sizeof(t), "T %.1f %d %d %s %d %d\n", dist, moveLeft(), moveRight(),
                     modeName(), wifiLastRssi(), moveBlocked() ? 1 : 0);
    if (client.write((const uint8_t*)t, n) != (size_t)n) {
      connected = false;
      client.stop();
      Serial.println("Server: write failed, reconnecting");
    }
  }
}
