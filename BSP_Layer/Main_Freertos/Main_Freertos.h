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
#include "LED/Stream_LED.h"
#define FOC_RUN_FLAG   (1U << 0)
typedef struct {
    float A;            //A相电压或电流
    float B;            //B相电压或电流
    float C;            //C相电压或电流
}U_Idata;
typedef struct {
    float iq;
    float speed;
    float pos;
    uint8_t ON_OFF;    //电机开机和关机
}FOC_PARAM_IN;
typedef struct {
    FOC_FB FOC_CUR_PARAM;
    SVPWM FOC_SVPWM;
    SPWM FOC_SPWM;
    AS5600_Enc_DATA FOC_ENC_DATA;
    FOC_PID FOC_PID;
    FOC_PARAM_IN FOC_PARAM;
    uint32_t notify_overfull;   //foc任务堆积量，越多说明中断间隔远小于任务运行时间
}FOC_DRIVE;
extern TaskHandle_t FocTask_Control;
extern FOC_DRIVE MOTOR;
extern BaseType_t FocTask_Busy;
#endif //MAIN_FREERTOS_H
