/**
 ******************************************************************************
 * @file	 mpu6500 driver.c
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/3/14
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "mpu6500 driver.h"

IMU_Data_t MPU6500;
static uint8_t tx, rx, tx_buff[14];
uint8_t deviceID = 0;
uint32_t temp_val = 0x64;
uint8_t debug_num = 0;

static void MPU6500_Delay(void);
static void MPU6500_chang(IMU_Data_t *mpu);

void MPU6500_Init(void)
{
    //        UNUSED(tx);
    //        UNUSED(rx);
    //        UNUSED(tx_buff);

    HAL_Delay(200);
    MPU6500_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6500
    HAL_Delay(200);
    MPU6500_Write_Byte(MPU_SIGPATH_RST_REG, 0x07);
    HAL_Delay(200);

    //    do
    //    {
    //        deviceID = MPU6500_Read_Byte(MPU_DEVICE_ID_REG);
    //    } while (deviceID != 0x70 && deviceID != 0x71 && deviceID != 0x74);

    MPU6500_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6500

    MPU6500_Write_Byte(MPU_SAMPLE_RATE_REG, 0x00); //不分频
    MPU6500_Write_Byte(MPU_CFG_REG, 0x02);         //陀螺仪带宽
    MPU6500_Write_Byte(MPU_GYRO_CFG_REG, 0x18);    //±250dps(00) ±500dps(01) ±1000dps(10) ±2000dps(11)

    MPU6500_Write_Byte(MPU_ACCEL_CFG_REG, 0x10);  //±2g(00) ±4g(01) ±8g(10) ±16g(11)
    MPU6500_Write_Byte(MPU_ACCEL_CFG_REG2, 0x00); //加速度计滤波

    MPU6500_Write_Byte(MPU_INT_EN_REG, 0X00);    //关闭所有中断
    MPU6500_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU6500_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //关闭FIFO
    MPU6500_Write_Byte(MPU_INTBP_CFG_REG, 0X02);

    deviceID = MPU6500_Read_Byte(MPU_DEVICE_ID_REG); //读取MPU6500的ID
    if (deviceID == MPU6500_ID)                      //器件ID正确
    {
        MPU6500_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为时钟源
        MPU6500_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
    }
    MPU6500_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为时钟源
    MPU6500_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作

    HAL_Delay(500);
    Calibrate_MPU_Offset(&MPU6500);
}

//较准零飘
void Calibrate_MPU_Offset(IMU_Data_t *mpu)
{
//    	 uint8_t buf[6], res;
//    	for (uint32_t i = 0; i < 5000; i++)
//    	{
//        MPU6500_Read_Multi_Byte(MPU_GYRO_XOUTH_REG, 6, buf);
//        mpu->gx = (float)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / 16.4;
//        mpu->gy = (float)(int16_t)(((uint16_t)buf[2] << 8) | buf[3]) / 16.4;
//        mpu->gz = (float)(int16_t)(((uint16_t)buf[4] << 8) | buf[5]) / 16.4;
//    
//    		mpu->gx_offset += mpu->gx;
//    		mpu->gy_offset += mpu->gy;
//        mpu->gz_offset += mpu->gz;
//    		HAL_Delay(1);
//    	}
//    
//    	   mpu->gx_offset /= 5000;
//         mpu->gy_offset /= 5000;
//         mpu->gz_offset /= 5000;

    mpu->gx_offset = -0.096876204;
    mpu->gy_offset = -0.385591418;
    mpu->gz_offset = 1.40309978;

    // mpu->gx_offset = 0;
    // mpu->gy_offset = 0;
    // mpu->gz_offset = 0;
}

void MPU6500_Get_Data(void)
{
    MPU_Get_Gyroscope(&MPU6500);
    MPU_Get_Accelerometer(&MPU6500);
    MPU_Get_Temperature(&MPU6500.temp);
}

//得到陀螺仪值(实际值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(IMU_Data_t *mpu)
{
    uint8_t buf[6], res;
    res = MPU6500_Read_Multi_Byte(MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == HAL_OK)
    {
        mpu->gx = (float)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / 16.4 - mpu->gx_offset;
        mpu->gy = (float)(int16_t)(((uint16_t)buf[2] << 8) | buf[3]) / 16.4 - mpu->gy_offset;
        mpu->gz = (float)(int16_t)(((uint16_t)buf[4] << 8) | buf[5]) / 16.4 - mpu->gz_offset;

        mpu->gx *= 3.14159 / 180;
        mpu->gy *= 3.14159 / 180;
        mpu->gz *= 3.14159 / 180;
    }
    return res;
}
//得到加速度值(实际值)
// gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//其他,错误代码
uint8_t MPU_Get_Accelerometer(IMU_Data_t *mpu)
{
    uint8_t buf[6], res;
    res = MPU6500_Read_Multi_Byte(MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == HAL_OK)
    {
        mpu->ax = (float)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / 4096 * 9.8;
        mpu->ay = (float)(int16_t)(((uint16_t)buf[2] << 8) | buf[3]) / 4096 * 9.8;
        mpu->az = (float)(int16_t)(((uint16_t)buf[4] << 8) | buf[5]) / 4096 * 9.8;
    }
    return res;
}

//得到温度值
int16_t MPU_Get_Temperature(float *temp)
{
    uint8_t buf[2];
    int16_t raw;
    MPU6500_Read_Multi_Byte(MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint8_t)buf[0] << 8) | buf[1];
    *temp = 21 + ((double)raw) / 333.87;
    return (int16_t)*temp * 100;
}

#ifdef SPI_MODE
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    MPU6500_SPI_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU6500_SPI, &tx, &rx, 1, 55);
    tx = data;
    if (HAL_SPI_TransmitReceive(&MPU6500_SPI, &tx, &rx, 1, 55) == HAL_OK)
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return HAL_OK;
    }
    else
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return HAL_ERROR;
    }
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    MPU6500_SPI_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU6500_SPI, &tx, &rx, 1, 2);
    if (HAL_SPI_TransmitReceive(&MPU6500_SPI, &tx, &rx, 1, 2) == HAL_OK)
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return rx;
    }
    else
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return HAL_ERROR;
    }
}

uint8_t MPU_Read_Multi_Byte(uint8_t reg, uint8_t len, uint8_t *buf)
{
    //	debug_num = 0;
    //	do
    //    {
    //		debug_num++;
    //        deviceID = MPU6500_Read_Byte(MPU_DEVICE_ID_REG);
    //    } while (deviceID != MPU6500_ID);
    //res = MPU6500_Read_Multi_Byte(MPU_ACCEL_XOUTH_REG, 6, buf);
    MPU6500_SPI_NSS_LOW;
    tx = reg | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU6500_SPI, &tx, &rx, 1, 55);
    if (HAL_SPI_TransmitReceive(&MPU6500_SPI, tx_buff, buf, len, 55) == HAL_OK)
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return HAL_OK;
    }
    else
    {
        MPU6500_SPI_NSS_HIGH;
        MPU6500_Delay();
        return HAL_ERROR; 
    }
}
#else // I2C MODE
uint8_t MPU_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (HAL_I2C_Mem_Write(&MPU6500_I2C, (uint16_t)addr, (uint16_t)reg, 1, &data, 1, UINT32_MAX) == HAL_OK)
        return HAL_OK;
    else
        return HAL_ERROR;
}

uint8_t MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    if (HAL_I2C_Mem_Read(&MPU6500_I2C, (uint16_t)addr, (uint16_t)reg, 1, &data, 1, UINT32_MAX) == HAL_OK)
        return data;
    else
        return HAL_ERROR;
}

uint8_t MPU_Read_Multi_Byte(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (HAL_I2C_Mem_Read(&MPU6500_I2C, (uint16_t)addr, (uint16_t)reg, 1, buf, len, 1000) == HAL_OK)
        return HAL_OK;
    else
        return HAL_ERROR;
}
#endif

static void MPU6500_Delay(void)
{
    for (uint32_t i = temp_val; i > 1; i--)
    {
    }
}
