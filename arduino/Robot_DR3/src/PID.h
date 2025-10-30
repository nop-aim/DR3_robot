/*****************************************************************
 * PID Controller Header File
 * @authors: Your Name
    @date: June 2024
    @version: 1.0
 * @brief  Controller PID for robotic applications
            Hardware: Arduino mega2560
*******************************************************************/
#ifndef PID_H
#define PID_H
#include <Arduino.h>
class PID {
     PID();
    ~PID();
    public :
        float Kp, Ki, Kd;
        float prev_error, integral;
        PID(float kp, float ki, float kd)
        float compute(float setpoint, float measured_value, float dt)
};
#endif

