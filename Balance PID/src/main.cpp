#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include "MotorDriver.h"
#include "TimingEvent.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (5)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Motor Driver.
MotorDriver motorDriver = MotorDriver();

// PID parameters.
double pitchSetpoint = 0;
double robotPitch = 0;
double motorPWM = 0;

const double Kp = 15;
const double Ki = 0;
const double Kd = 0.6;
const double PID_SAMPLE_TIME = BNO055_SAMPLERATE_DELAY_MS;

PID motorController = PID(&robotPitch, &motorPWM, &pitchSetpoint, Kp, Ki, Kd, DIRECT);

void getGyroDataAndComputePID();
TimingEvent gyroPIDEvent = TimingEvent::setInterval(BNO055_SAMPLERATE_DELAY_MS, &getGyroDataAndComputePID);

void setupBNO() {
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void setupMotorPIDController() {
  motorController.SetSampleTime(PID_SAMPLE_TIME);
  motorController.SetOutputLimits(
    -1023+MOTOR_PWM_DRIVING_THRESHOLD, 
    1023-MOTOR_PWM_DRIVING_THRESHOLD);
  motorController.SetMode(AUTOMATIC);
}

void getGyroDataAndComputePID() {
  sensors_event_t event;
  bno.getEvent(&event);

  robotPitch = event.orientation.z;

  if (abs(robotPitch) <= 45 && motorController.Compute()) {
    // Ouput the pwm signal.
    if (motorPWM >= 0) {
      motorDriver.driverMotorFoward(motorPWM);
    }
    else {
      motorDriver.driverMotorBackward(abs(motorPWM));
    }
  }
  else {
    motorDriver.driverMotorStop();
  }
}

void setup() {
  // Serial.begin(115200);
  setupBNO();
  setupMotorPIDController();
  motorDriver.initializeDriver();
}

void loop() {
  gyroPIDEvent.update();

  // Serial.print("Robot Pitch: ");
  // Serial.print(robotPitch);
  // Serial.print("\tPWM: ");
  // Serial.println(motorPWM);
}