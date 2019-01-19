#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include "MotorDriver.h"
#include "TimingEvent.h"
#include "DebugUtils.h"
#include "BNO.h"


// Motor Driver.
MotorDriver motorDriver = MotorDriver();

// PID parameters.
double pitchSetpoint = 0;
double robotPitch = 0;
double angularVelocity = 0;
double lastMeasuredPitch = 0;
double motorPWM = 0;

const double Kp = 15;
const double Ki = 0;
const double Kd = 0.6;

const double PID_SAMPLE_TIME = BNO055_SAMPLERATE_DELAY_MS;

PID motorController = PID(&robotPitch, &motorPWM, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

void getGyroDataAndComputePID();
TimingEvent gyroPIDEvent = TimingEvent::setInterval(BNO055_SAMPLERATE_DELAY_MS, &getGyroDataAndComputePID);


void setupMotorPIDController() {
  motorController.SetSampleTime(PID_SAMPLE_TIME);
  motorController.SetOutputLimits(
    -1023+MOTOR_PWM_DRIVING_THRESHOLD, 
    1023-MOTOR_PWM_DRIVING_THRESHOLD);
  motorController.SetMode(AUTOMATIC);
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
  debugSend(PITCH_STR, robotPitch);
  debugSend(ANGULAR_VELOCITY_STR, angularVelocity);

  if (abs(robotPitch) <= 45 && motorController.Compute()) {
    if (deadband(robotPitch, -0.5, 0.5)) {
      motorPWM = 0;
    }
    // Ouput the pwm signal.
    motorDriver.driverMotorWithPWM(motorPWM);
  }
  else {
    motorDriver.driverMotorStop();
  }
  debugSend(PWM_STR, motorPWM);
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