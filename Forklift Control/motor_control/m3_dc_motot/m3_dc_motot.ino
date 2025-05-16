#include <Arduino.h>

#define MOTOR_PIN 18

const int pwmChannel = 0;
const int pwmFreq = 1000;
const int pwmResolution = 8;

void setup() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN, pwmChannel);
}

void loop() {
  ledcWrite(pwmChannel, 128);  // 50% 속도
  delay(2000);
  ledcWrite(pwmChannel, 0);    // 정지
  delay(1000);
}
