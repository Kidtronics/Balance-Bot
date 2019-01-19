#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include "MotorDriver.h"
#include "TimingEvent.h"
#include "DebugUtils.h"
#include "BNO.h"
#include "BufferedSlidingWindowFilter.h"


// Motor Driver.
MotorDriver motorDriver = MotorDriver();

// PID parameters.
double pitchSetpoint = 0;
double angVelSetpoint = 0;

double robotPitch = 0;
double angularVelocity = 0;
double lastMeasuredPitch = 0;
double motorPWM = 0;

const double PITCH_Kp = 0.025;
const double PITCH_Ki = 0;
const double PITCH_Kd = 0;

const double ANGVEL_Kp = 600;
const double ANGVEL_Ki = 0;
const double ANGVEL_kd = 0.6;
const double PID_SAMPLE_TIME = BNO055_SAMPLERATE_DELAY_MS;

double angularVelocityBuffer[3];
BufferedSlidingWindowFilter angularVelocityFilter = 
  BufferedSlidingWindowFilter(angularVelocityBuffer, 3);

PID pitch2AngVelController =
  PID(&robotPitch, &angVelSetpoint, &pitchSetpoint, PITCH_Kp, PITCH_Ki, PITCH_Kd, DIRECT);

PID angVel2PWMController = 
  PID(&angularVelocity, &motorPWM, &angVelSetpoint, ANGVEL_Kp, ANGVEL_Ki, ANGVEL_kd, DIRECT);

void getGyroDataAndComputePID();
TimingEvent gyroPIDEvent = 
  TimingEvent::setInterval(BNO055_SAMPLERATE_DELAY_MS, &getGyroDataAndComputePID);


void setupMotorPIDController() {
  angVel2PWMController.SetSampleTime(PID_SAMPLE_TIME);
  angVel2PWMController.SetOutputLimits(
    -1023+MOTOR_PWM_DRIVING_THRESHOLD, 
    1023-MOTOR_PWM_DRIVING_THRESHOLD);
  angVel2PWMController.SetMode(AUTOMATIC);

  pitch2AngVelController.SetSampleTime(PID_SAMPLE_TIME);
  // This limit is experimentally determied.
  pitch2AngVelController.SetOutputLimits(-1, 1);
  pitch2AngVelController.SetMode(AUTOMATIC);
}

void getGyroDataAndComputePID() {
  sensors_event_t event;
  BNO.getEvent(&event);
  robotPitch = event.orientation.z;
  
  double measuredAngularVelocity = (robotPitch - lastMeasuredPitch) / PID_SAMPLE_TIME;
  angularVelocityFilter.addNewSample(measuredAngularVelocity);
  angularVelocity = angularVelocityFilter.getAverageValue();
  lastMeasuredPitch = robotPitch;

  debugSendTimestamp();
  debugSend(PITCH_STR, robotPitch);
  debugSend(ANGULAR_VELOCITY_STR, angularVelocity);

  if (abs(robotPitch) <= 45) {
    if (pitch2AngVelController.Compute() && angVel2PWMController.Compute()) {
      // Ouput the pwm signal.
      motorDriver.driverMotorWithPWM(motorPWM);
    }
  }
  else {
    motorDriver.driverMotorStop();
  }
  debugSend(PWM_STR, motorPWM);
  debugSend(ANGULAR_VELOCITY_SETPOINT_STR, angVelSetpoint);
}

void setup() {
  Serial.begin(115200);
  setupBNO();
  setupMotorPIDController();
  motorDriver.initializeDriver();
}

void loop() {
  gyroPIDEvent.update();
}