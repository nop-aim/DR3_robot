/*****************************************************************
 * PID Controller Implementation File
 * @authors: Your Name
    @date: June 2024
    @version: 1.0
 * @brief  Controller PID for robotic applications
            Hardware: Arduino mega2560
*******************************************************************/
#include "PID.h"
PID::PID() {}
PID::~PID() {}
/******************************************************************
 * Hàm khởi tạo bộ điều khiển PID với các hệ số Kp, Ki, Kd
 * @param kp: Hệ số tỷ lệ
 * @param ki: Hệ số tích phân
 * @param kd: Hệ số đạo hàm
 * @return: Không có
 ******************************************************************/
PID::PID(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    prev_error = 0;
    integral = 0;
}
float PID::compute(float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    return output;
}
