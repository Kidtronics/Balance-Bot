#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <Arduino.h>

#define PITCH_STR "Pitch"
#define PWM_STR "PWM"

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