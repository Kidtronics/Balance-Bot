#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <Arduino.h>

#define PITCH_STR "Pitch"
#define PITCH_SETPOINT_STR "PitchS"
#define PWM_STR "PWM"
#define ANGULAR_VELOCITY_STR "AngVel"
#define ANGULAR_VELOCITY_SETPOINT_STR "AngVelS"
#define ANGULAR_ACCELERATION_STR "AngAcc"
#define MOTOR_SPEED_STR "MSpeed"
#define MOTOR_SPEED_SETPOINT_STR "MSpeedS"

extern Serial_ Serial;

template <class T>
void debugSend(String name, T data) {
    Serial.print(name);
    Serial.print(":");
    Serial.println(data);
}

void debugSendTimestamp() {
    Serial.print("Time:");
    Serial.println(millis());
}

template void debugSend<int>(String name, int data);
template void debugSend<unsigned int>(String name, unsigned int data);
template void debugSend<long>(String name, long data);
template void debugSend<unsigned long>(String name, unsigned long data);
template void debugSend<String>(String name, String data);
template void debugSend<float>(String name, float data);
template void debugSend<double>(String name, double data);

#endif