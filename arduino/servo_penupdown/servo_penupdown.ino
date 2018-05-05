#include <Servo.h>

// const int SERVO_PIN = 9;
const int SERVO_PIN = 27; // PA8

// const int CMD_PIN = 2;
const int CMD_PIN = 18; // PB4
const int CMD_UP = LOW;
const int CMD_DOWN = HIGH;
const int ANGLE_UP = 105;
const int ANGLE_DOWN = 99;

Servo servo;

void setup() {
  pinMode(CMD_PIN, INPUT_PULLUP);
  servo.attach(SERVO_PIN);
  servo.write(ANGLE_UP);
}

void loop() {
  auto const cmd = digitalRead(CMD_PIN);
  auto const req = 
    cmd == CMD_UP
    ? ANGLE_UP
    : ANGLE_DOWN;
  if(servo.read()!=req) {
      servo.write(req);
  }
}

