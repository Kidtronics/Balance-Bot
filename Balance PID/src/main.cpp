#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include "TimingEventManager.h"
#include "DebugUtils.h"
#include "BNO.h"
#include "MotorEncoder.h"
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

// PID parameters.
double pitchSetpoint = 0;
double robotPitch = 0;
double angularVelocity = 0;
double lastMeasuredPitch = 0;
double motorSpeedSetpoint = 0;

const double Kp = 200;
const double Ki = 0;
const double Kd = 2;

const double PID_SAMPLE_TIME = BNO055_SAMPLERATE_DELAY_MS;

PID pitchController = PID(&robotPitch, &motorSpeedSetpoint, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

void setupPitchPIDController() {
  encoder1.negate();
  pitchController.SetSampleTime(PID_SAMPLE_TIME);
  pitchController.SetOutputLimits(-1000, 1000);
  pitchController.SetMode(AUTOMATIC);
}

bool deadband(double value, double lower, double upper) {
  return value >= lower && value <= upper;
}

void getGyroDataAndComputePID() {
  sensors_event_t event;
  BNO.getEvent(&event);
  robotPitch = event.orientation.z;
  angularVelocity = (robotPitch - lastMeasuredPitch) / PID_SAMPLE_TIME;
  lastMeasuredPitch = robotPitch;

  debugSendTimestamp();
  // debugSend(PITCH_STR, robotPitch);
  // debugSend(MOTOR_SPEED, speedController0.getCurrentSpeedDeg());

  if (abs(robotPitch) <= 45 && pitchController.Compute()) {
    // if (deadband(robotPitch, -0.1, 0.1)) {
    //   motorSpeedSetpoint = 0;
    // }
    speedController0.setSpeedDeg(motorSpeedSetpoint);
    speedController1.setSpeedDeg(motorSpeedSetpoint);
  }
  else {
    speedController0.setSpeedDeg(0);
    speedController1.setSpeedDeg(0);
  }
  debugSend(MOTOR_SPEED_SETPOINT_STR, motorSpeedSetpoint);
  debugSend(MOTOR_SPEED_STR, encoder0.getDegrees());
  debugSend(PWM_STR, speedController0.getOutputPWM());
}

void setup() {
  Serial.begin(115200);
  setupBNO();
  setupPitchPIDController();
  TimingEventManager::getInstance().setInterval(BNO055_SAMPLERATE_DELAY_MS, &getGyroDataAndComputePID);
}

void loop() {
  TimingEventManager::getInstance().update();
}