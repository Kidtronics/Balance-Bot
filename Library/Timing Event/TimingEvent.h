#ifndef TIMING_EVENT_H
#define TIMING_EVENT_H

/** 
 * This Timing Event simulates timer interrupt.
 * You can set an event (or function) to be executed
 * at a certain interval or to be executed once
 * after a certain time period.
 * You have to call update function in the
 * microcontroller's loop function.
 * Caveat: If a callback takes too long to execute,
 * the other event will not be able to be executed in time.
 * The short yet time-critical code should be executed through
 * real timer interrupt.
 */

class TimingEvent {
private:
    enum EventType {INTERVAL, TIMEOUT};

    void (*m_callback)();
    unsigned long m_lastTime;
    unsigned long m_interval;
    EventType m_eventType;
    bool m_stopEvent;

    TimingEvent(unsigned long interval, EventType eventType, void (*callback) ());
public:
    /** Execute the callback at an interval in milliseconds. */
    static TimingEvent setInterval(unsigned long interval, void (*callback) ());
    /** Execute the callback only once, interval is in in milliseconds. */
    static TimingEvent setTimeout(unsigned long interval, void (*callback) ());
    /** Update event in the loop. It returns if the callback is executed.*/
    bool update();
};

#endif