#include "main.h"
#include "can.h"
#include "pid.h"
#define MotorInit(pid, Kp, Ki, Kd, max_out, max_dout) PidInit(pid, Kp, Ki, Kd, max_out, max_dout)

typedef struct
{
    uint16_t mechanical_angle; //机械角度
    float angle;
    int16_t speed;    //转子转速
    uint16_t current; //转矩电流
    uint8_t temprate; //电机温度
	
	  Pid_T Speedpid;
	  Pid_T Anglepid;
	  float out;
} Motor_t;

extern Motor_t motor[4];

void MotorInit(Pid_T *pid, float Kp, float Ki, float Kd, float max_out, float max_dout);

void GetMotorInformation(uint8_t *data, Motor_t *motor); //由电机传回量转为物理量

float SpeedControl(Motor_t *motor,float SpeedSet);//速度环

float AngleControl(Motor_t *motor,float AngleSet);
