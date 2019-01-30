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

double setSpeed = 2000;

void calculateEncoderSpeed() {
  debugSendTimestamp();
  // debugSend(PWM_STR, speedController0.getOutputPWM());
  debugSend(MOTOR_SPEED_STR, speedController0.getCurrentSpeedDeg());
}

void startMotor();

void stopMotor() {
  speedController0.setSpeedDeg(0);
  TimingEventManager::getInstance().setTimeout(3000, &startMotor);
}

void startMotor() {
  speedController0.setSpeedDeg(setSpeed);
  TimingEventManager::getInstance().setTimeout(3000, &stopMotor);
  setSpeed = -setSpeed;
}

void setup() {
  Serial.begin(112500);
  TimingEventManager::getInstance().setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed);
  TimingEventManager::getInstance().setTimeout(3000, &startMotor);
  encoder1.negate();
}

void loop() {
  TimingEventManager::getInstance().update();
}