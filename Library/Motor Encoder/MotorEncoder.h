#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

/** 
 * A library for encoder.
 * You can only instantiate two instances of the encoder class,
 * extra instances of the encoder will overwrite the interrupt callback functions
 * of the previous instances.
*/

#define DEFAULT_DEGREE_PER_INCREMENT 0.6 // In degrees.
#define MAX_NUM_ENCODERS 2

class MotorEncoder {    
private:
    double m_degreePerIncrement;
    unsigned int m_phaseAPin;
    unsigned int m_phaseBPin;
    int m_currentCounts;
    bool m_running;
    bool m_negate;
public:
    MotorEncoder(unsigned int phaseAPin, unsigned int phaseBPin, double degreePerIncrement=DEFAULT_DEGREE_PER_INCREMENT);
    MotorEncoder(){}
    /** Start counting steps. */
    void start();
    /** Clear count. */
    void clear();
    /** Get number of steps in since start. */
    int getCount();
    /** Get rotation in degrees. */
    double getDegrees();
    /** Negate the count and degrees when using getter. */
    void negate();
    /** Callback function when interrupt on phase A pin occurs. */
    void interruptCallback();

    /** 
     * Don't use copy constructor and assignment operator,
     * because we only want maximum of two encoders.
     */
    MotorEncoder(MotorEncoder const&) = delete;
    void operator=(MotorEncoder const&)  = delete;
};

void interruptCallbackEncoder0();
void interruptCallbackEncoder1();

#endif