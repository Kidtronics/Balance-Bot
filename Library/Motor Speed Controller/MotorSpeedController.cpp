#include <Arduino.h>
#include "MotorSpeedController.h"
#include "TimingEventManager.h"

void calculateEncoderSpeedAndPID(void* ptr) {
    MotorSpeedController* speedController = (MotorSpeedController*) ptr;
    double degrees = speedController->m_encoder->getDegrees();

    // Remember to clear the encoder counts when starting next cycle of sample;
    speedController->m_encoder->clear();

    // Calculate instantaneous speed by dividing encoder counts
    // by encoder sample rate. Then add this value to the filter.
    speedController->m_filter.addNewSample(
        degrees * 1000 / ENCODER_SPEED_SAMPLE_RATE);

    // Get the average speed from the filter.
    speedController->m_currentSpeed = speedController->m_filter.getAverageValue();

    // Compute pid and output the pwm.
    if (speedController->m_motorSpeedPIDController.Compute()) {
        if (speedController->m_outputPWM > 0) {
            analogWrite(speedController->m_forwardPin, speedController->m_outputPWM + MOTOR_PWM_THRESHOLD);
            analogWrite(speedController->m_backwardPin, 0);
        }
        else if (speedController->m_outputPWM < 0) {
            analogWrite(speedController->m_forwardPin, 0);
            analogWrite(speedController->m_backwardPin, abs(speedController->m_outputPWM) + MOTOR_PWM_THRESHOLD);
        }
        else {
            analogWrite(speedController->m_forwardPin, 0);
            analogWrite(speedController->m_backwardPin, 0);
        }
    }
}

MotorSpeedController::MotorSpeedController(MotorEncoder& encoder, unsigned int forwardPin, unsigned int backwardPin)
    : m_speedSetpoint(0), m_currentSpeed(0),m_outputPWM(0), m_filter(m_filterBuffer, MOTOR_SPEED_FILTER_BUFFER_SIZE),
    m_motorSpeedPIDController(&m_currentSpeed, &m_outputPWM, &m_speedSetpoint, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT)
{
    m_encoder = &encoder;
    m_encoder->start();
    TimingEventManager::getInstance()
        .setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeedAndPID, this);
    m_forwardPin = forwardPin;
    m_backwardPin = backwardPin;
    pinMode(m_forwardPin, OUTPUT);
    pinMode(m_backwardPin, OUTPUT);

    m_motorSpeedPIDController.SetSampleTime(ENCODER_SPEED_SAMPLE_RATE);
    m_motorSpeedPIDController.SetMode(AUTOMATIC);
    m_motorSpeedPIDController.SetOutputLimits(-PWM_OUTPUT_LIMIT, PWM_OUTPUT_LIMIT);
}

void MotorSpeedController::setSpeedDeg(double speed) {
    m_speedSetpoint = speed;
}

double MotorSpeedController::getCurrentSpeedDeg() {
    return m_currentSpeed;
}

double MotorSpeedController::getOutputPWM() {
    return m_outputPWM;
}