#include <Arduino.h>
#include "DebugUtils.h"
#include "MotorEncoder.h"

#define ENCODER0_PHASE_A_PIN 2
#define ENCODER0_PHASE_B_PIN 8

double totalRotation = 0;
bool output = false;

MotorEncoder encoder0 = MotorEncoder(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);

void setup() {
  Serial.begin(112500);
  encoder0.start();
  encoder0.negate();
}

void loop() {
  Serial.println(encoder0.getDegrees());
}