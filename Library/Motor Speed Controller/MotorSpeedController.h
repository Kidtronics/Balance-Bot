#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "MotorEncoder.h"
#include "BufferedSlidingWindowFilter.h"
#include "PID_v1.h"

#define ENCODER_SPEED_SAMPLE_RATE 4 // in milliseconds.
#define MOTOR_SPEED_FILTER_BUFFER_SIZE 4
#define MOTOR_PWM_THRESHOLD 70 // A threshold to enable motor to move.
#define MOTOR_KP 0.4
#define MOTOR_KI 0
#define MOTOR_KD 0
#define PWM_OUTPUT_LIMIT 1000

class MotorSpeedController {
private:
    MotorEncoder* m_encoder;
    double m_speedSetpoint;
    double m_currentSpeed;
    double m_outputPWM;
    unsigned int m_forwardPin;
    unsigned int m_backwardPin;
    /** Bufer that slidng window filter uses. */
    double m_filterBuffer[MOTOR_SPEED_FILTER_BUFFER_SIZE];
    BufferedSlidingWindowFilter m_filter;
    PID m_motorSpeedPIDController;

    /** A callabck function to calculate speed in an interval. */
    friend void calculateEncoderSpeedAndPID(void* ptr);
public:
    MotorSpeedController(MotorEncoder& encoder, unsigned int forwardPin, unsigned int backwardPin);
    /** Set speed in degrees per second. */
    void setSpeedDeg(double speed);
    /** Get current speed in degrees per second. */
    double getCurrentSpeedDeg();
    /** Get current output pwm. */
    double getOutputPWM();

    MotorSpeedController(MotorSpeedController const&) = delete;
    void operator=(MotorSpeedController const&)  = delete;
};

#endif