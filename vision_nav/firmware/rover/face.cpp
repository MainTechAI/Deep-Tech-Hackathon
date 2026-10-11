#include "face.h"
#include "Arduino_LED_Matrix.h"

static ArduinoLEDMatrix matrix;

static uint8_t HAPPY[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};
static uint8_t SURPRISED[8][12] = {
  {0,1,1,1,0,0,0,0,1,1,1,0},
  {0,1,0,1,0,0,0,0,1,0,1,0},
  {0,1,1,1,0,0,0,0,1,1,1,0},
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0}
};
static uint8_t ARROW_UP[8][12] = {
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,1,0,1,1,0,1,1,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0}
};
static uint8_t ARROW_LEFT[8][12] = {
  {0,0,0,1,0,0,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,0,0,0,0},
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {0,0,1,1,0,0,0,0,0,0,0,0},
  {0,0,0,1,0,0,0,0,0,0,0,0}
};
static uint8_t STOP_X[8][12] = {
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,0,0,1,0,0,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0}
};
static uint8_t frameBuf[8][12];
static int shown = -1;

static void show(uint8_t img[8][12], int id) {
  if (shown == id) return;
  matrix.renderBitmap(img, 8, 12);
  shown = id;
}

void faceBegin()     { matrix.begin(); }
void showHappy()     { show(HAPPY, 1); }
void showSurprised() { show(SURPRISED, 2); }
void showUp()        { show(ARROW_UP, 3); }
void showLeft()      { show(ARROW_LEFT, 4); }
void showStop()      { show(STOP_X, 5); }
void showDown() {
  if (shown == 6) return;
  for (int r = 0; r < 8; r++) for (int c = 0; c < 12; c++) frameBuf[r][c] = ARROW_UP[7 - r][c];
  matrix.renderBitmap(frameBuf, 8, 12); shown = 6;
}
void showRight() {
  if (shown == 7) return;
  for (int r = 0; r < 8; r++) for (int c = 0; c < 12; c++) frameBuf[r][c] = ARROW_LEFT[r][11 - c];
  matrix.renderBitmap(frameBuf, 8, 12); shown = 7;
}

// 3x5 digits, each row is 3 bits (MSB = left column)
static const uint8_t DIGITS[10][5] = {
  {7,5,5,5,7}, {2,6,2,2,7}, {7,1,7,4,7}, {7,1,7,1,7}, {5,5,7,1,1},
  {7,4,7,1,7}, {7,4,7,5,7}, {7,1,2,2,2}, {7,5,7,5,7}, {7,5,7,1,7}
};

void showNumber(int n) {
  n = constrain(n, 0, 999);
  memset(frameBuf, 0, sizeof(frameBuf));
  int digits[3] = {n / 100, (n / 10) % 10, n % 10};
  int first = n >= 100 ? 0 : n >= 10 ? 1 : 2;    // skip leading zeros
  for (int i = first; i < 3; i++)
    for (int r = 0; r < 5; r++)
      for (int c = 0; c < 3; c++)
        frameBuf[r + 1][i * 4 + c] = (DIGITS[digits[i]][r] >> (2 - c)) & 1;
  matrix.renderBitmap(frameBuf, 8, 12);
  shown = -1;
}

void showMotion(int left, int right) {
  if (left == 0 && right == 0)                          showStop();
  else if (left > 0 && right > 0 && abs(left - right) < 30) showUp();
  else if (left < 0 && right < 0)                       showDown();
  else if (left < right)                                showLeft();
  else                                                  showRight();
}
