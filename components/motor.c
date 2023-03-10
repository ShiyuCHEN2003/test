#include "motor.h"

Motor_t motor[4];
//电机数据转换为物理量
//*data：电机传回数据   *motor电机参数

void GetMotorInformation(uint8_t *data, Motor_t *motor)
{
	motor->mechanical_angle = (uint16_t)data[1] + (uint16_t)(data[0] << 8);
	motor->speed = (int16_t)((uint16_t)data[3] + (uint16_t)(data[2] << 8));
	motor->current = (uint16_t)data[5] + (uint16_t)(data[4] << 8);
	motor->temprate = data[7];
	motor->angle = (float)motor->mechanical_angle / 8191 * 360;
}

//速度环 SpeedSet/RPM
float SpeedControl(Motor_t *motor,float SpeedSet)
{
	float feedback;
	feedback = (float)motor->speed;
	PidCalculate(&motor->Speedpid, SpeedSet, feedback);
  motor->out = motor->Speedpid.out;
	return motor->out;
}


float AngleControl(Motor_t *motor,float AngleSet)
{
	float feedback;
	feedback = (float)motor->angle;
	PidCalculate(&motor->Anglepid,AngleSet,feedback);
	SpeedControl(motor,motor->Anglepid.out);
  motor->out = motor->Speedpid.out;
	return motor->out;
}
