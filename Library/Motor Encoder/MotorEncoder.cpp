#include <Arduino.h>
#include "MotorEncoder.h"

MotorEncoder* encoders[MAX_NUM_ENCODERS];
int encoderId = 0;

MotorEncoder::MotorEncoder(
    unsigned int phaseAPin, 
    unsigned int phaseBPin, 
    double degreePerIncrement)
    : m_currentCounts(0), m_running(false)
{
    m_phaseAPin = phaseAPin;
    m_phaseBPin = phaseBPin;
    m_degreePerIncrement=degreePerIncrement;
    pinMode(phaseAPin, INPUT_PULLUP);
    pinMode(phaseBPin, INPUT_PULLUP);

    switch(encoderId) {
        case 0:
            attachInterrupt(m_phaseAPin, &interruptCallbackEncoder0, RISING);
            break;
        case 1:
            attachInterrupt(m_phaseAPin, &interruptCallbackEncoder1, RISING);
            break;
    }
    encoders[encoderId] = this;
    encoderId = (encoderId+1) % MAX_NUM_ENCODERS;
}

void MotorEncoder::interruptCallback() {
    if (!m_running) {
        return;
    }
    int phaseBVoltage = digitalRead(m_phaseBPin);
    if (phaseBVoltage) {
        m_currentCounts--;
    }
    else {
        m_currentCounts++;
    }
}

void MotorEncoder::start() {
    m_running = true;
}

void MotorEncoder::clear() {
    m_currentCounts = 0;
}

int MotorEncoder::getCount() {
    return m_negate
        ? -m_currentCounts
        : m_currentCounts;
}

double MotorEncoder::getDegrees() {
    return m_negate 
        ? -m_currentCounts * m_degreePerIncrement 
        : m_currentCounts * m_degreePerIncrement;
}

void MotorEncoder::negate() {
    m_negate = true;
}

void interruptCallbackEncoder0() {
    encoders[0]->interruptCallback();
}

void interruptCallbackEncoder1() {
    encoders[1]->interruptCallback();
}