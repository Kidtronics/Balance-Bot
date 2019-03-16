# Balance-Bot
This is a C++ code for balancing robot. It is based on Arduino framework. The code is created with Platform IO IDE, 
therefore it is easiest to compile and run the code with platform IO installed either in Visual Studio Code or Atom.

## What does the code do
The code can drive the two-wheel robot that has Inertial Measurement Unit (IMU), speed encoders and a motor to balance itself.
The IMU used is BNO055, the encoder is 600 steps/revolution generic encoder, and the motor is a hobby geared DC motor.

## Architecture Preview
### Scheduling
The scheduling is timing event architecture. All the event or repeated calculations are done 
within corresponding callback functions. A global singleton called TimingEventManager manages all the callback functions.
For example, the IMU (BNO055) is sampled at 100 Hz, therefore, we need a callback function containing the sampling logic and
have it called once per 10 ms. First, this sampling callback is registered to the TimingEventManager. Then the TimingEventManager,
while keeping track of other callbacks, will invoke the sampling callabck every 10 ms interval. With this architecture,
you can execute a piece of code at the rate you want it. The currently highest rate is 1 kHz, this is limited by
the software. The code is tested on SAMD51, which has 120 Mhz clock.

### Control Architecture
The control architecture is three layers cascade PID controllers.

The outer layer is robot speed controller, it has input
of robot speed , which is compared against desired speed, and output is the desired robot pitch angle. To understand this, imagine the robot is now standing still,
and it wants to move forward. The robot will want to pitch forward, and thus the robot will accelerates to the desired speed.
Then this controller will command robot to pitch to neutral, stopping the acceleration.

The middle layer is pitch controller. It takes the desired pitch reference from robot speed controller, and compares it against
the current pitch angle, then output the desired motor speed for inner most layer PID controller.

The inner most controller is the motor speed controller. It takes the desired motor speed from the pitch controller, and
compares it with current motor speed from the encoder, and outputs the pwm signal to the motor driver.

## Folder structure
The folders "Balance PID", "Blink", "Gyroscope" and "Motor Speed Control" are individual Platform IO projects. 
Thus, you can compile and run each folder independently. The "Library" folder contains the code that is shared
by those Platform IO projects.
