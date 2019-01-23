#include "TimingEvent.h"
#include <Arduino.h>

TimingEvent::TimingEvent(unsigned long interval, EventType eventType, void (*callback) ()) 
    :m_stopEvent(false) 
{
    m_interval = interval;
    m_eventType = eventType;
    m_callback = callback;
    m_callbackType = NO_ARGUMENT;
}

TimingEvent::TimingEvent(
    unsigned long interval, 
    EventType eventType, 
    void (*callback) (void*), 
    void* arg)
    :m_stopEvent(false) 
{
    m_interval = interval;
    m_eventType = eventType;
    m_callabckWithPointer = callback;
    m_callbackType = POINTER;
    m_callbackArg = arg;
}

TimingEvent TimingEvent::setInterval(
    unsigned long interval, 
    void (*callback) ()) 
{
    return TimingEvent(interval, INTERVAL, callback);
}

TimingEvent TimingEvent::setInterval(
    unsigned long interval, 
    void (*callback) (void*), 
    void* arg)
{
    return TimingEvent(interval, INTERVAL, callback, arg);
}

TimingEvent TimingEvent::setTimeout(
    unsigned long interval, 
    void (*callback) ()) 
{
    return TimingEvent(interval, TIMEOUT, callback);
}

TimingEvent TimingEvent::setTimeout(
    unsigned long interval, 
    void (*callback) (void*), 
    void* arg)
{
    return TimingEvent(interval, TIMEOUT, callback, arg);
}

bool TimingEvent::update() {
    if (m_stopEvent) {
        return false;
    }

    unsigned long now = millis();
    if (now - m_lastTime > m_interval) {
        invokeCallback();
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

TimingEvent::EventType TimingEvent::getEventType() {
    return m_eventType;
}

void TimingEvent::invokeCallback() {
    if (m_callbackType == NO_ARGUMENT) {
        m_callback();
    }
    else if(m_callbackType == POINTER) {
        m_callabckWithPointer(m_callbackArg);
    }
}