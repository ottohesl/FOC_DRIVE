#ifndef FOC_PID_H
#define FOC_PID_H
#include "main.h"
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float prev_error;
    float sum_error;
    float increment;    //积分累加项
    float integral;
    float integral_limit;
    float increment_limit;
    float output_max;
    float _output;
    float Ts;
}FOC_PID;
void PID_Init(FOC_PID *foc_pid, float pid_freq);
float foc_pid_speed(float target ,float current);
void FOC_PID_SPEED(FOC_PID* S,float target_speed,float current_speed) ;
void FOC_PID_CUR(FOC_PID *C, float target_cur, float current_cur);
#endif //FOC_PID_H
