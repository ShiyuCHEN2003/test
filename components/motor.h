#include "main.h"
#include "can.h"
#include "pid.h"
#define MotorInit(pid, Kp, Ki, Kd, max_out, max_dout) PidInit(pid, Kp, Ki, Kd, max_out, max_dout)

typedef struct
{
    uint16_t mechanical_angle; //��е�Ƕ�
    float angle;
    int16_t speed;    //ת��ת��
    uint16_t current; //ת�ص���
    uint8_t temprate; //����¶�
	
	  Pid_T Speedpid;
	  Pid_T Anglepid;
	  float out;
} Motor_t;

extern Motor_t motor[4];

void MotorInit(Pid_T *pid, float Kp, float Ki, float Kd, float max_out, float max_dout);

void GetMotorInformation(uint8_t *data, Motor_t *motor); //�ɵ��������תΪ������

float SpeedControl(Motor_t *motor,float SpeedSet);//�ٶȻ�

float AngleControl(Motor_t *motor,float AngleSet);
