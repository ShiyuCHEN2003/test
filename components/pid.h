
#ifndef _PID_H
#define _PID_H
#include "math.h"
#include "stdint.h"


typedef struct
{
    float Kp;   
	  float Ki;
    float Kd;

    float max_out;  //������
    float max_dout; //���������

    float Set;
    float feedback;

    float out;
    float pout;
    float iout;
    float dout;

    float error[3]; // 0��ǰʱ��1ǰһʱ��2ǰ��ʱ��
} Pid_T;

// extern Pid_T Pid;

void PidInit(Pid_T *pid, float Kp, float Ki, float Kd, float max_out, float max_iout); //�趨PID��������ʼֵ

float PidCalculate(Pid_T *pid, float set, float feedback); // PID�������ֵ
#endif
