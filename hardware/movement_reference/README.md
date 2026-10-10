# L298N movement reference

This is the latest standalone movement and obstacle-avoidance sketch supplied
for the physical chassis. It is retained as a hardware reference for its motor
and sonar wiring:

| Module | Pins |
| --- | --- |
| L298N left motor | ENA D5 (PWM), IN1 D9, IN2 D10 |
| L298N right motor | ENB D6 (PWM), IN3 D11, IN4 D12 |
| Fixed HC-SR04 | TRIG D3, ECHO D2 |

The active Wi-Fi seeker is in `../wayfinder_rover/wayfinder_rover.ino` and
uses the same wiring.
