#ifndef PID_H
#define PID_H

#include "stdint.h"
#include "stdbool.h"

typedef struct
{
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float Setpoint; // Desired value
    float Integral; // Integral term
    float PreviousError; // Previous error for derivative calculation
    float Output; // Output value
    uint16_t SampleTime; // Time interval for PID update in ms
    uint32_t LastTime; // Last time the PID was updated
} PID_HandleTypeDef;

PID_HandleTypeDef PID_Init(float k_p, float k_i, float k_d, float setpoint, uint16_t sample_time);
float PID_Compute(PID_HandleTypeDef *pid, float input);

#endif /* PID_H */