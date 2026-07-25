
#ifndef FOC_RUN_H
#define FOC_RUN_H
#include "FOC_Math.h"
#include "FOC_PID.h"
#include "Main_Freertos.h"
#include "as5600.h"
#include "cmsis_os2.h"
#include "FOC_FB.h"
#include "i2c.h"
#include <tgmath.h>
#include "Main_Freertos.h"
#include "tim.h"
#include "usart.h"
#include "ottohesl.h"
#define PID_FIEQ  10000   //pid运算频率
#define Increment_Limit   3.0   //电压最大增量幅值（v）
typedef enum {
    FOC_SPWM_OPEN_MODE,
    FOC_SPWM_SPEED_MODE,
    FOC_SPWM_POSTION_MODE,
    FOC_SPWM_STOP_MODE,
    FOC_SVPWM_OPEN_MODE,
    FOC_SVPWM_SPEED_MODE,
    FOC_SVPWM_POSTION_MODE,
    FOC_SVPWM_STOP_MODE,
}FOC_RUN_STATE;
void SPWM_RUN(SPWM *spwm, FOC_RUN_STATE state);
void SVPWM_RUN(SVPWM *svpwm, FOC_RUN_STATE state) ;
void SVPWM_TIM_RUN(SVPWM *svpwm,float target_speed);
extern VOFA_DATA vofa_data_run;
#endif //FOC_RUN_H
