
/**
 ******************************************************************************
 * @file	 mpu6500 driver.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/3/14
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _MPU6500_DRIVER_H
#define _MPU6500_DRIVER_H

#define SPI_MODE

#include "mpu6500 reg.h"
#include "stdint.h"

#ifdef SPI_MODE
#include "spi.h"
#define MPU6500_SPI hspi1

#define MPU6500_NSS_Pin GPIO_PIN_4
#define MPU6500_NSS_GPIO_Port GPIOA
#define MPU6500_SPI_NSS_LOW HAL_GPIO_WritePin(MPU6500_NSS_GPIO_Port, MPU6500_NSS_Pin, GPIO_PIN_RESET)
#define MPU6500_SPI_NSS_HIGH HAL_GPIO_WritePin(MPU6500_NSS_GPIO_Port, MPU6500_NSS_Pin, GPIO_PIN_SET)

#define MPU6500_Write_Byte(reg, data) MPU_Write_Byte(reg, data)
#define MPU6500_Read_Byte(reg) MPU_Read_Byte(reg)
#define MPU6500_Read_Multi_Byte(reg, len, buf) MPU_Read_Multi_Byte(reg, len, buf)
#else
#include "i2c.h"
#define MPU6500_I2C hi2c1

#define MPU6500_Write_Byte(reg, data) MPU_Write_Byte(MPU6500_ADDR_I2C_HAL, reg, data)
#define MPU6500_Read_Byte(reg) MPU_Read_Byte(MPU6500_ADDR_I2C_HAL, reg)
#define MPU6500_Read_Multi_Byte(reg, len, buf) MPU_Read_Multi_Byte(MPU6500_ADDR_I2C_HAL, reg, len, buf)
#endif

#define MPU_ACCEL_3G_SEN 0.0008974358974f
#define MPU_ACCEL_6G_SEN 0.00179443359375f
#define MPU_ACCEL_8G_SEN 0.00239420166015625f
#define MPU_ACCEL_12G_SEN 0.0035888671875f
#define MPU_ACCEL_24G_SEN 0.007177734375f

#define MPU_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define MPU_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define MPU_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define MPU_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define MPU_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef struct
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  float temp;

  float ax_offset;
  float ay_offset;
  float az_offset;

  float gx_offset;
  float gy_offset;
  float gz_offset;

} IMU_Data_t;

extern IMU_Data_t MPU6500;

void MPU6500_Init(void);

void MPU6500_Get_Data(void);
#ifdef SPI_MODE
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU_Read_Byte(uint8_t reg);
uint8_t MPU_Read_Multi_Byte(uint8_t reg, uint8_t len, uint8_t *buf);
#else
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg);
uint8_t MPU_Read_Multi_Byte(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
#endif

void Calibrate_MPU_Offset(IMU_Data_t *mpu);

uint8_t MPU_Get_Gyroscope(IMU_Data_t *mpu);
uint8_t MPU_Get_Accelerometer(IMU_Data_t *mpu);
uint8_t MPU_Get_Magnetometer(int16_t *mx, int16_t *my, int16_t *mz);
int16_t MPU_Get_Temperature(float *temp);

#endif
