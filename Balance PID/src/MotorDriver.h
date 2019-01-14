#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// Motor controlling pins.
#define LEFT_MOTOR_FORWARD_OUTPUT_PIN 2
#define LEFT_MOTOR_BACKWARD_OUTPUT_PIN 3
#define RIGHT_MOTOR_FORWARD_OUTPUT_PIN 4
#define RIGHT_MOTOR_BACKWARD_OUTPUT_PIN 5
#define MOTOR_PWM_DRIVING_THRESHOLD 100

class MotorDriver {
public:
    void initializeDriver();
    void driverMotorFoward(unsigned int pwm);
    void driverMotorBackward(unsigned int pwm);
    void driverMotorStop();
};

#endif