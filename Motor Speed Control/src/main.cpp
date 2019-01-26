#include <Arduino.h>
#include "DebugUtils.h"
#include "MotorEncoder.h"
#include "TimingEventManager.h"
#include "MotorSpeedController.h"

#define ENCODER0_PHASE_A_PIN 3
#define ENCODER0_PHASE_B_PIN 10
#define ENCODER1_PHASE_A_PIN 2
#define ENCODER1_PHASE_B_PIN 9

#define RIGHT_MOTOR_FORWARD_PIN 8
#define RIGHT_MOTOR_BACKWARD_PIN 7
#define LEFT_MOTOR_FORWARD_PIN 4
#define LEFT_MOTOR_BACKWARD_PIN 6

MotorEncoder encoder0(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);
MotorSpeedController speedController0(encoder0, RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN);

MotorEncoder encoder1(ENCODER1_PHASE_A_PIN, ENCODER1_PHASE_B_PIN);
MotorSpeedController speedController1(encoder1, LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN);

void calculateEncoderSpeed() {
  debugSendTimestamp();
  debugSend(ANGULAR_VELOCITY_STR, speedController1.getCurrentSpeedDeg());
  debugSend(PWM_STR, speedController1.getOutputPWM());
}

void setup() {
  Serial.begin(112500);
  TimingEventManager::getInstance().setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed);
  encoder1.negate();
  speedController0.setSpeedDeg(0);
  speedController1.setSpeedDeg(0);
}

void loop() {
  TimingEventManager::getInstance().update();
}