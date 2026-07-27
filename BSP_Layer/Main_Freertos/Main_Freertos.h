//
// Created by 34969 on 26-3-26.
//

#ifndef MAIN_FREERTOS_H
#define MAIN_FREERTOS_H
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "as5600.h"
#include "ottohesl_OLED.h"
#include "FOC_FB.h"
#include "FOC_Math.h"
#include "FOC_PID.h"
#include "task.h"
typedef struct {
    float A;            //A相电压或电流
    float B;            //B相电压或电流
    float C;            //C相电压或电流
}U_Idata;
typedef struct {
    float now_speed;
    float targe_speed;  //目标速度
    float now_angle;
    float targe_angle;   //目标角度
    U_Idata Udata;       //ABC相电压值
    U_Idata Idata;       //ABC相电流值
    float uq;           //q轴电压
    float ud;           //d轴电压
    float iq;           //q轴电流
    float id;           //d轴电流
    float elect_angle;   //电角度
}VOFA_DATA;
typedef struct {
    VOFA_DATA FOC_VOFA_DATA;
    FOC_FB FOC_CUR_PARAM;
    SVPWM FOC_SVPWM;
    SPWM FOC_SPWM;
    AS5600_Enc_DATA FOC_ENC_DATA;
    FOC_PID FOC_PID;
    uint32_t notify_overfull;   //foc任务堆积量，越多说明中断间隔远小于任务运行时间
}FOC_DRIVE;
typedef struct {
    float target_speed;
    float target_angle;
    float target_iq;
    float target_id;
}FOC_CONTROL;   //foc参数输入（使用串口、按键）-----后续移至app层
extern TaskHandle_t FocTask_Control;
extern FOC_DRIVE MOTOR;
#endif //MAIN_FREERTOS_H
