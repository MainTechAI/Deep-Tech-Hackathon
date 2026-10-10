// Tethered motor bench for the L298N rover: timed pulses only, no Wi-Fi, no autonomy.
// Serial 115200, "Newline". Each line: <cmd> [duty 0-255] [ms]
//   f/b forward/backward | a/d spin left/right | l/r one wheel only | x stop | s sonar
// Example: "f 150 1000" drives forward at duty 150 for 1 s. Defaults: duty 255, 500 ms.
// A move always ends on its own; any new line stops the current move first.
// Start with the wheels raised.

// ENA/ENB must be on PWM pins (UNO R4: D3, D5, D6, D9, D10, D11). Set USE_PWM to false
// only if they are wired to non-PWM pins; the duty is then ignored (full power).
constexpr uint8_t ENA = 5, IN1 = 9, IN2 = 10;
constexpr uint8_t ENB = 6, IN3 = 11, IN4 = 12;
constexpr bool USE_PWM = true;
constexpr uint8_t TRIG_PIN = 3, ECHO_PIN = 2;

constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;
constexpr uint32_t MAX_MOVE_MS = 5000;

bool moving = false;
uint32_t moveStartedAt = 0, moveMs = 0;
String line;

void setMotor(uint8_t enable, uint8_t forwardPin, uint8_t reversePin, int direction, int duty) {
  digitalWrite(forwardPin, direction > 0 ? HIGH : LOW);
  digitalWrite(reversePin, direction < 0 ? HIGH : LOW);
  if (direction == 0) duty = 0;
  if (USE_PWM) analogWrite(enable, duty);
  else digitalWrite(enable, duty > 0 ? HIGH : LOW);
}

void drive(int left, int right, int duty) {
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
  setMotor(ENA, IN1, IN2, left, duty);
  setMotor(ENB, IN3, IN4, right, duty);
}

void stopMove() {
  drive(0, 0, 0);
  if (moving) {
    Serial.print("stopped after ms: ");
    Serial.println(millis() - moveStartedAt);
  }
  moving = false;
}

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const uint32_t width = pulseIn(ECHO_PIN, HIGH, 25000);
  return width ? width * 0.0343f / 2.0f : -1.0f;
}

void handleLine(const String& text) {
  stopMove();
  char cmd = 0;
  int duty = 255, ms = 500;
  if (sscanf(text.c_str(), " %c %d %d", &cmd, &duty, &ms) < 1) return;
  duty = constrain(duty, 0, 255);
  ms = constrain(ms, 0, static_cast<int>(MAX_MOVE_MS));

  int left = 0, right = 0;
  switch (tolower(cmd)) {
    case 'f': left = 1; right = 1; break;
    case 'b': left = -1; right = -1; break;
    case 'a': left = -1; right = 1; break;
    case 'd': left = 1; right = -1; break;
    case 'l': left = 1; break;
    case 'r': right = 1; break;
    case 's': Serial.print("front cm: "); Serial.println(readDistanceCm(), 1); return;
    default: Serial.println("STOP"); return;
  }
  Serial.print("move "); Serial.print(cmd);
  Serial.print(" duty "); Serial.print(USE_PWM ? duty : 255);
  Serial.print(" ms "); Serial.println(ms);
  moveMs = ms;
  moveStartedAt = millis();
  moving = true;
  drive(left, right, duty);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  drive(0, 0, 0);
  Serial.begin(115200);
  delay(1500);
  Serial.println("Motor bench ready. <cmd> [duty] [ms]; f b a d l r x s");
}

void loop() {
  if (moving && millis() - moveStartedAt >= moveMs) stopMove();
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (line.length()) handleLine(line);
      line = "";
    } else if (line.length() < 32) line += c;
  }
}
