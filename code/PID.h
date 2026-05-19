/*
 * PID.h
 *
 * Position PID controller for the steering angle motor.
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct {
    float Target;
    float Current;
    float Error;
    float LastError;
    float Integral;
    float Kp;
    float Ki;
    float Kd;
    float Output;
    float MaxOutput;
    float IntegralMax;
} PID_TypeDef;

void  PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output);
void  PID_Reset(PID_TypeDef *pid);
float PID_Compute(PID_TypeDef *pid, float target, float current);

#endif