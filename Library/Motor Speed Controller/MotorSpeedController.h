#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "MotorEncoder.h"
#include "BufferedSlidingWindowFilter.h"
#include "PID_v1.h"

#define ENCODER_SPEED_SAMPLE_RATE 4 // in milliseconds.
#define MOTOR_SPEED_FILTER_BUFFER_SIZE 4
#define MOTOR_PWM_THRESHOLD 100 // A threshold to enable motor to move.
#define MOTOR_KP 0.2
#define MOTOR_KI 1
#define MOTOR_KD 0
#define PWM_OUTPUT_LIMIT 200
#define DEFAULT_MOTOR_SPEED_DEAD_BAND 100

class MotorSpeedController {
private:
    MotorEncoder* m_encoder;
    double m_speedSetpoint;
    double m_currentSpeed;
    double m_outputPWM;
    double m_motorSpeedDeadband;
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
    /** Set the dead band speed, in which output pwm will be zero. */
    void setDeadband(double deadband);
    /** Get current speed in degrees per second. */
    double getCurrentSpeedDeg();
    /** Get current output pwm. */
    double getOutputPWM();

    MotorSpeedController(MotorSpeedController const&) = delete;
    void operator=(MotorSpeedController const&)  = delete;
};

#endif