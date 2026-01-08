#include "pid.h"


void PID_Init(PIDController* pid, float kp, float ki, float kd, float dt, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}


void PID_SetSetpoint(PIDController* pid, float setpoint) {
    pid->setpoint = setpoint;
}


float PID_Compute(PIDController* pid, float feedback) {
    float error = pid->setpoint - feedback;

    // Integral term
    pid->integral += error * pid->dt;

    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;

    // PID output
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Clamp output
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;
    return output;
}


void PID_Reset(PIDController* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
