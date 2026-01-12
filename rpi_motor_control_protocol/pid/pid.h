#ifndef PID_H
#define PID_H


typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float setpoint;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PIDController;


void PID_Init(PIDController* pid, float kp, float ki, float kd, float dt, float out_min, float out_max);

void PID_SetSetpoint(PIDController* pid, float setpoint);

float PID_Compute(PIDController* pid, float feedback);

void PID_Reset(PIDController* pid);

#endif
