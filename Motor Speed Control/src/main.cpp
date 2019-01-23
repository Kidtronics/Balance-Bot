#include <Arduino.h>
#include "DebugUtils.h"
#include "MotorEncoder.h"
#include "TimingEventManager.h"
#include "MotorSpeedController.h"

#define ENCODER0_PHASE_A_PIN 2
#define ENCODER0_PHASE_B_PIN 8
#define ENCODER_SPEED_SAMPLE_RATE 4 // in milliseconds.

double totalRotation = 0;
bool output = false;

MotorEncoder encoder0(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);
MotorSpeedController speedController0(encoder0);

// Rotational speed in RPM.
double encoder0Speed = 0;
void calculateEncoderSpeed();

void calculateEncoderSpeed() {
  debugSendTimestamp();
  debugSend(PITCH_STR, speedController0.getCurrentSpeedDeg());
}

void setup() {
  Serial.begin(112500);
  encoder0.start();
  TimingEventManager::getInstance().setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed);
}

void loop() {
  TimingEventManager::getInstance().update();
}