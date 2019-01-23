#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "MotorEncoder.h"

#define ENCODER_SPEED_SAMPLE_RATE 4 // in milliseconds.

class MotorSpeedController {
private:
    MotorEncoder* m_encoder;
    double m_speedSetpoint;
    double m_currentSpeed;
    /** A callabck function to calculate speed in an interval. */
    friend void calculateEncoderSpeed(void* ptr);
public:
    MotorSpeedController(MotorEncoder& encoder);
    /** Set speed in degrees per second. */
    void setSpeedDeg(double speed);
    /** Get current speed in degrees per second. */
    double getCurrentSpeedDeg();

    MotorSpeedController(MotorSpeedController const&) = delete;
    void operator=(MotorSpeedController const&)  = delete;
};

#endif