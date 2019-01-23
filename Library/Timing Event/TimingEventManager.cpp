#include "TimingEventManager.h"

TimingEventManager::TimingEventManager() : m_eventsSize(0)
{}

TimingEventManager& TimingEventManager::getInstance() {
    static TimingEventManager instance;
    return instance;
}

bool TimingEventManager::setInterval(unsigned long interval, void (*callback) ()) {
    if (m_eventsSize == MAX_NUM_EVENTS) {
        return false;
    }
    m_events[m_eventsSize] = TimingEvent::setInterval(interval, callback);
    m_eventsSize++;
    return true;
}

bool TimingEventManager::setInterval(
    unsigned long interval, 
    void (*callback) (void*), 
    void* arg) 
{
    if (m_eventsSize == MAX_NUM_EVENTS) {
        return false;
    }
    m_events[m_eventsSize] = TimingEvent::setInterval(interval, callback, arg);
    m_eventsSize++;
    return true;
}

bool TimingEventManager::setTimeout(unsigned long interval, void (*callback) ()) {
    if (m_eventsSize == MAX_NUM_EVENTS) {
        return false;
    }
    m_events[m_eventsSize] = TimingEvent::setTimeout(interval, callback);
    m_eventsSize++;
    return true;
}

bool TimingEventManager::setTimeout(
    unsigned long interval, 
    void (*callback) (void*), 
    void* arg)
{
    if (m_eventsSize == MAX_NUM_EVENTS) {
        return false;
    }
    m_events[m_eventsSize] = TimingEvent::setTimeout(interval, callback, arg);
    m_eventsSize++;
    return true;
}

void TimingEventManager::update() {
    int idx = 0;
    while (idx < m_eventsSize) {
        // Timeout event can only be run once.
        // We delete this event after is is run.
        if (m_events[idx].update() && m_events[idx].getEventType() == TimingEvent::TIMEOUT) {
            for (int i=idx; i < m_eventsSize-1; i++) {
                m_events[idx] = m_events[idx+1];
            }
            m_eventsSize--;
        }
        else {
            idx++;
        }
    }
}