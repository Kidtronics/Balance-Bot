#include "TimingEvent.h"
#include <Arduino.h>

TimingEvent::TimingEvent(unsigned long interval, EventType eventType, void (*callback) ()) 
    :m_stopEvent(false) 
{
    m_interval = interval;
    m_eventType = eventType;
    m_callback = callback;
}

TimingEvent TimingEvent::setInterval(
    unsigned long interval, 
    void (*callback) ()) 
{
    return TimingEvent(interval, INTERVAL, callback);
}

TimingEvent TimingEvent::setTimeout(
    unsigned long interval, 
    void (*callback) ()) 
{
    return TimingEvent(interval, TIMEOUT, callback);
}

bool TimingEvent::update() {
    if (m_stopEvent) {
        return false;
    }

    unsigned long now = millis();
    if (now - m_lastTime > m_interval) {
        m_callback();
        m_lastTime = now;

        if (m_eventType == TIMEOUT) {
            m_stopEvent = true;
        }
        return true;
    }
    else {
        return false;
    }
}