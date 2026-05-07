
#ifndef FOC_RUN_H
#define FOC_RUN_H
#include "FOC_Math.h"
#include "FOC_PID.h"
#include "ottohesl.h"
#define PID_FIEQ  50   //pid运算频率
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
#endif //FOC_RUN_H
