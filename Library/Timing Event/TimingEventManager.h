#ifndef TIMING_EVENT_MANAGER_H
#define TIMING_EVENT_MANAGER_H

/**
 * This is a singleton manager for all the timing events.
 * Right now, it has maximum number of events. When exceed,
 * no more events can be added.
 * Events are added through TimingEventManager::getInstance().setInterval(...).
 * It is required to run update() inside the loop() function.
*/

#include "TimingEvent.h"

class TimingEventManager {
private:
    static const int MAX_NUM_EVENTS = 20;
    // This could be a vector, but using vector gives compile error.
    // So I opted for array.
    TimingEvent m_events[MAX_NUM_EVENTS];
    int m_eventsSize;
public:
    TimingEventManager();
    static TimingEventManager& getInstance();
    /** 
     * Set the interval event. Return true on success, 
     * false when the events array is exceeded.
     */
    bool setInterval(unsigned long interval, void (*callback) ());
    /** 
     * Set the timeout event. Return true on success, 
     * false when the events array is exceeded.
     */
    bool setTimeout(unsigned long interval, void (*callback) ());
    void update();

    TimingEventManager(TimingEventManager const&) = delete;
    void operator=(TimingEventManager const&)  = delete;
};

#endif