#include "pid.h"
#include "stddef.h"

// Pid_T Pid;
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PidInit(Pid_T *pid, float Kp, float Ki, float Kd, float max_out, float max_dout)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_out = max_out;
    pid->max_dout = max_dout;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}

float PidCalculate(Pid_T *pid, float set, float feedback)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->Set = set;
    pid->feedback = feedback;
    pid->error[0] = pid->Set - pid->feedback;
    pid->pout = pid->Kp * pid->error[0];
    pid->iout += pid->Ki * pid->error[0];
    pid->dout = pid->Kd * (pid->error[0] - pid->error[1]);
    LimitMax(pid->dout, pid->max_dout);
    pid->out = pid->pout + pid->iout + pid->dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}
