
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
typedef enum {
    FOC_OPEN_MODE,
    FOC_SPEED_MODE,
    FOC_POSTION_MODE,
    FOC_STOP_MODE,
}FOC_RUN_STATE;
void SVPWM_TIM_RUN(SVPWM *svpwm,float target_speed);
void FOC_MOTOR_RUN(FOC_DRIVE *run_type, FOC_RUN_STATE state);
extern VOFA_DATA vofa_data_run;
#endif //FOC_RUN_H
