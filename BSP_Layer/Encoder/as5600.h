#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#define AS5600_I2C_ADDR    0x36    // 7位地址
#define AS5600_RAW_ANG_H   0x0C    // 原始角度高字节寄存器
#define AS5600_RAW_ANG_L   0x0D    // 原始角度低字节寄存器
#define AS5600_ERROR       0xFFFF  // 错误返回值
#define AS5600_DEG_ERROR   NAN     // 使用标准NaN表示错误
#define AS5600_ANGLE_BUFF  128     // 得到角度存入的缓冲区大小
typedef struct {
    uint8_t tim_index;
    uint8_t calc_flag;
}Tim_Enc_Data;
/**********求解速度计算结构体***********/
typedef struct {
    float cal_now_angle;
    float cal_last_angle;
    float tick;             //计算周期或者是时间常量
    float last_tick;
}Calc_Speed;
/**********总as5600结构体***********/
typedef struct {
    float raw_data;                             //从dma得出的原始数据
    float Enc_angle;                            //编码器角度
    float Enc_speed;                            //编码器速度
    uint8_t Enc_Raw_buf[2];                     //编码器原始数据缓冲区
    float Enc_angle_buf[AS5600_ANGLE_BUFF];     //编码器解算角度缓冲区
    uint8_t Enc_Index;                          //编码器缓冲区索引
    Calc_Speed Enc_calc_speed;
    Tim_Enc_Data tim_enc_data;
}AS5600_Enc_DATA;



// 函数声明
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c);
float AS5600_GetAngleDegrees(I2C_HandleTypeDef *hi2c);
int8_t AS5600_Get_LR(I2C_HandleTypeDef *hi2c);
float AS5600_Get_Turns(I2C_HandleTypeDef *hi2c);
float AS5600_CalcSpeed_MovAvg(float mech_angle_deg);
float AS5600_Get_Speed(AS5600_Enc_DATA *calc);
void FOC_ENC_Update(AS5600_Enc_DATA *calc);
extern AS5600_Enc_DATA Enc_data;
#endif
