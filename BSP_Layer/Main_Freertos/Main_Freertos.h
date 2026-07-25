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
#endif //MAIN_FREERTOS_H
