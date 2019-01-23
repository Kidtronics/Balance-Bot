#include "MotorSpeedController.h"
#include "TimingEventManager.h"

void calculateEncoderSpeed(void* ptr) {
    MotorSpeedController* speedController = (MotorSpeedController*) ptr;
    speedController->m_currentSpeed = 
        speedController->m_encoder->getDegrees() * 1000 / ENCODER_SPEED_SAMPLE_RATE;
    speedController->m_encoder->clear();
}

MotorSpeedController::MotorSpeedController(MotorEncoder& encoder)
    : m_speedSetpoint(0), m_currentSpeed(0)
{
    m_encoder = &encoder;
    TimingEventManager::getInstance()
        .setInterval(ENCODER_SPEED_SAMPLE_RATE, &calculateEncoderSpeed, this);
}

void MotorSpeedController::setSpeedDeg(double speed) {
    m_speedSetpoint = speed;
}

double MotorSpeedController::getCurrentSpeedDeg() {
    return m_currentSpeed;
}

