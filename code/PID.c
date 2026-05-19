/*
 * PID.c
 *
 * Position PID controller for the steering angle motor.
 */

#include "zf_common_headfile.h"
#include "PID.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->MaxOutput = max_output;
    pid->IntegralMax = 2000;
    PID_Reset(pid);
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->Target = 0;
    pid->Current = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Output = 0;
}

float PID_Compute(PID_TypeDef *pid, float target, float current)
{
    pid->Target = target;
    pid->Current = current;
    pid->Error = pid->Target - pid->Current;

    pid->Integral += pid->Error;
    if (pid->Integral > pid->IntegralMax) {
        pid->Integral = pid->IntegralMax;
    }
    if (pid->Integral < -pid->IntegralMax) {
        pid->Integral = -pid->IntegralMax;
    }

    pid->Output = (pid->Kp * pid->Error) +
                  (pid->Ki * pid->Integral) +
                  (pid->Kd * (pid->Error - pid->LastError));

    pid->LastError = pid->Error;

    if (pid->Output > pid->MaxOutput) {
        pid->Output = pid->MaxOutput;
    }
    if (pid->Output < -pid->MaxOutput) {
        pid->Output = -pid->MaxOutput;
    }

    return pid->Output;
}