#ifndef __AS5600_H
#define __AS5600_H
#include "stm32G4xx_hal.h"
#include"math.h"
#define AS5600_I2C_ADDR    0x36    // 7位地址
#define AS5600_RAW_ANG_H   0x0C    // 原始角度高字节寄存器
#define AS5600_RAW_ANG_L   0x0D    // 原始角度低字节寄存器
#define AS5600_ERROR       0xFFFF  // 错误返回值
#define AS5600_DEG_ERROR   NAN     // 使用标准NaN表示错误
#define i2c_AS5600         hi2c2
// 函数声明
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c);
float AS5600_GetAngleDegrees(I2C_HandleTypeDef *hi2c);
int8_t AS5600_Get_LR(I2C_HandleTypeDef *hi2c);
float AS5600_Get_Turns(I2C_HandleTypeDef *hi2c);
float AS5600_Get_Speed(I2C_HandleTypeDef *hi2c);
extern int full_ration;
extern float angle_points;
extern float speeds;
#endif
