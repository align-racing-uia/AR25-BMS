#include "pid.h"

PID_HandleTypeDef PID_Init(float k_p, float k_i, float k_d, float setpoint, uint16_t sample_time){
    PID_HandleTypeDef pid;
    pid.Kp = k_p;
    pid.Ki = k_i;
    pid.Kd = k_d;
    pid.Setpoint = setpoint;
    pid.Integral = 0.0f;
    pid.PreviousError = 0.0f;
    pid.Output = 0.0f;
    pid.SampleTime = sample_time;
    pid.LastTime = HAL_GetTick();
    
    return pid;
}

float PID_Compute(PID_HandleTypeDef *pid, float input){
    uint32_t now = HAL_GetTick();
    float error = pid->Setpoint - input;
    float delta_time = (now - pid->LastTime) / 1000.0f; // Convert to seconds

    if (delta_time >= pid->SampleTime / 1000.0f) {
        pid->Integral += error * delta_time;
        float derivative = (error - pid->PreviousError) / delta_time;

        pid->Output = (pid->Kp * error) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);

        pid->PreviousError = error;
        pid->LastTime = now;
    }

    return pid->Output;
}