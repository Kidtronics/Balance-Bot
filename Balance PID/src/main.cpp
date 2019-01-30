#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include "TimingEventManager.h"
#include "DebugUtils.h"
#include "BNO.h"
#include "MotorEncoder.h"
#include "MotorSpeedController.h"
#include "BufferedSlidingWindowFilter.h"

#define ENCODER0_PHASE_A_PIN 3
#define ENCODER0_PHASE_B_PIN 10
#define ENCODER1_PHASE_A_PIN 2
#define ENCODER1_PHASE_B_PIN 9

#define RIGHT_MOTOR_FORWARD_PIN 8
#define RIGHT_MOTOR_BACKWARD_PIN 7
#define LEFT_MOTOR_FORWARD_PIN 4
#define LEFT_MOTOR_BACKWARD_PIN 6

//-------------------------------------Motor Speed Controller-------------------------------------------

MotorEncoder encoder0(ENCODER0_PHASE_A_PIN, ENCODER0_PHASE_B_PIN);
MotorSpeedController speedController0(encoder0, RIGHT_MOTOR_FORWARD_PIN, RIGHT_MOTOR_BACKWARD_PIN);

MotorEncoder encoder1(ENCODER1_PHASE_A_PIN, ENCODER1_PHASE_B_PIN);
MotorSpeedController speedController1(encoder1, LEFT_MOTOR_FORWARD_PIN, LEFT_MOTOR_BACKWARD_PIN);

//----------------------------------------Robot Pitch PID-------------------------------------------------

// Pitch PID parameters.
double pitchSetpoint = 0;
double robotPitch = 0;
double lastMeasuredPitch = 0;
double motorSpeedSetpoint = 0;
double speedSetPointLimit = 1500;

const double Kp = 150;
const double Ki = 20;
const double Kd = 0;

const double PID_SAMPLE_TIME = BNO055_SAMPLERATE_DELAY_MS;

PID pitchController = PID(&robotPitch, &motorSpeedSetpoint, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

//----------------------------------------Robot Speed PID-------------------------------------------------
const double bufferSize = 50;
double bufferSpeed[50];
BufferedSlidingWindowFilter averageSpeed(bufferSpeed, bufferSize);

// Speed PID parameters
double robotSpeed = 0;
double desiredSpeed = 0;

const double SpeedKp = 0.01;
const double SpeedKi = 0;
const double SpeedKd = 0;

const double ROBOT_SPEED_PID_SAMPLE_TIME = ENCODER_SPEED_SAMPLE_RATE * bufferSize;

// PID robotSpeedController = PID(&robotSpeed, &pitchSetpoint, &desiredSpeed, SpeedKp, SpeedKi, SpeedKd, DIRECT);

void setupPitchPIDController() {
  encoder1.negate();
  pitchController.SetSampleTime(PID_SAMPLE_TIME);
  pitchController.SetOutputLimits(-speedSetPointLimit, speedSetPointLimit);
  pitchController.SetMode(AUTOMATIC);
}

void setupRobotSpeedController() {
  // robotSpeedController.SetSampleTime(ROBOT_SPEED_PID_SAMPLE_TIME);
  // robotSpeedController.SetOutputLimits(-3, 3);
  // robotSpeedController.SetMode(AUTOMATIC);
}

void calculateRobotSpeedAndComputePID() {
  // averageSpeed.addNewSample((speedController0.getCurrentSpeedDeg() + speedController1.getCurrentSpeedDeg()) / 2);
  // robotSpeed = averageSpeed.getAverageValue();
  // robotSpeedController.Compute();
}

void getGyroDataAndComputePID() {
  sensors_event_t event;
  BNO.getEvent(&event);
  robotPitch = event.orientation.z;
  lastMeasuredPitch = robotPitch;

  if (abs(robotPitch) <= 45 && pitchController.Compute()) {
    speedController0.setSpeedDeg(motorSpeedSetpoint);
    speedController1.setSpeedDeg(motorSpeedSetpoint);
  }
  else {
    speedController0.setSpeedDeg(0);
    speedController1.setSpeedDeg(0);
  }

  debugSendTimestamp();
  debugSend(PITCH_STR, robotPitch);
  // debugSend(PITCH_SETPOINT_STR, pitchSetpoint);
  // debugSend(MOTOR_SPEED_SETPOINT_STR, motorSpeedSetpoint);
  debugSend(MOTOR_SPEED_STR, speedController0.getCurrentSpeedDeg());
  // debugSend(PWM_STR, speedController0.getOutputPWM());
}

void setup() {
  Serial.begin(115200);
  setupBNO();
  setupPitchPIDController();
  setupRobotSpeedController();
  TimingEventManager::getInstance().setInterval(BNO055_SAMPLERATE_DELAY_MS, &getGyroDataAndComputePID);
  // TimingEventManager::getInstance().setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateRobotSpeedAndComputePID);
}

void loop() {
  TimingEventManager::getInstance().update();
}