#include <Arduino.h>
#include "DebugUtils.h"
#include "MotorEncoder.h"
#include "TimingEventManager.h"
#include "MotorSpeedController.h"

#define ENCODER0_PHASE_A_PIN 2
#define ENCODER0_PHASE_B_PIN 8
#define RIGHT_MOTOR_FORWARD_PIN 4
#define RIGHT_MOTOR_BACKWARD_PIN 6

MotorEncoder encoder0(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);
MotorSpeedController speedController0(encoder0, RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN);


void calculateEncoderSpeed() {
  debugSendTimestamp();
  debugSend(ANGULAR_VELOCITY_STR, speedController0.getCurrentSpeedDeg());
  debugSend(PWM_STR, speedController0.getOutputPWM());
}

void setup() {
  Serial.begin(112500);
  TimingEventManager::getInstance().setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed);
  speedController0.setSpeedDeg(500);
}

void loop() {
  TimingEventManager::getInstance().update();
}