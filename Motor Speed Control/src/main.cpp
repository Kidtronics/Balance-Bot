#include <Arduino.h>
#include "DebugUtils.h"
#include "MotorEncoder.h"
#include "TimingEvent.h"

#define ENCODER0_PHASE_A_PIN 2
#define ENCODER0_PHASE_B_PIN 8
#define ENCODER_SPEED_SAMPLE_RATE 4 // in milliseconds.

double totalRotation = 0;
bool output = false;

MotorEncoder encoder0 = MotorEncoder(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);

// Rotational speed in RPM.
double encoder0Speed = 0;
void calculateEncoderSpeed();
TimingEvent encoder0SpeedEvent = TimingEvent::setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed);

void calculateEncoderSpeed() {
  encoder0Speed = encoder0.getDegrees() * 166.667 / ENCODER_SPEED_SAMPLE_RATE;
  encoder0.clear();
  debugSendTimestamp();
  debugSend(PITCH_STR, encoder0Speed);
}

void setup() {
  Serial.begin(112500);
  encoder0.start();
}

void loop() {
  encoder0SpeedEvent.update();
}