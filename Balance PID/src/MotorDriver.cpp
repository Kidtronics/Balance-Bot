#include "MotorDriver.h"
#include <Arduino.h>

void MotorDriver::initializeDriver() {
    pinMode(RIGHT_MOTOR_FORWARD_OUTPUT_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD_OUTPUT_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_FORWARD_OUTPUT_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD_OUTPUT_PIN, OUTPUT);
}

void MotorDriver::driverMotorWithPWM(int pwm) {
  if (pwm > 0) {
    driverMotorFoward(pwm);
  }
  else if (pwm < 0) {
    driverMotorBackward(abs(pwm));
  }
  else {
    driverMotorStop();
  }
}

void MotorDriver::driverMotorFoward(unsigned int pwm) {
    analogWrite(RIGHT_MOTOR_FORWARD_OUTPUT_PIN, pwm + MOTOR_PWM_DRIVING_THRESHOLD);
    analogWrite(LEFT_MOTOR_FORWARD_OUTPUT_PIN, pwm + MOTOR_PWM_DRIVING_THRESHOLD);
    analogWrite(RIGHT_MOTOR_BACKWARD_OUTPUT_PIN, 0);
    analogWrite(LEFT_MOTOR_BACKWARD_OUTPUT_PIN, 0);
}

void MotorDriver::driverMotorBackward(unsigned int pwm) {
  analogWrite(RIGHT_MOTOR_FORWARD_OUTPUT_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_OUTPUT_PIN, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD_OUTPUT_PIN, pwm + MOTOR_PWM_DRIVING_THRESHOLD);
  analogWrite(LEFT_MOTOR_BACKWARD_OUTPUT_PIN, pwm + MOTOR_PWM_DRIVING_THRESHOLD);
}

void MotorDriver::driverMotorStop() {
  analogWrite(RIGHT_MOTOR_FORWARD_OUTPUT_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_OUTPUT_PIN, 0);
  analogWrite(RIGHT_MOTOR_BACKWARD_OUTPUT_PIN, 0);
  analogWrite(LEFT_MOTOR_BACKWARD_OUTPUT_PIN, 0);
}