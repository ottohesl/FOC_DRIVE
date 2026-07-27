//
// Created by 34969 on 26-5-30.
//

#ifndef UI_COMPONENT_H
#define UI_COMPONENT_H
#include <stdint.h>
#include <string.h>
#include "Main_Freertos.h"
#ifdef __cplusplus
extern "C" {
#endif
    // 【纯C声明】外部C文件调用的更新电压函数
    void Updata_Voltage(float value);
    void Updata_Angle(float value1,float value2);
    void Updata_Cur(float value1,float value2);
    void Updata_Mode(const char* ch1,const char* ch2);
    void Updata_RPS(float value1,float value2);
#ifdef __cplusplus
}
#endif
typedef struct LCD_UI_ENUM {
    float real_Angle;
    float real_Cur;
    float real_Speed;
    char real_Mode[10];
}UI;
typedef struct MOTOR_UI_Type {
    float Voltage;
    UI ui;
}MUY;
void UI_Update();

#endif //UI_COMPONENT_H
