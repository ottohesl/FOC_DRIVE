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
    float increment_limit;
    float output_max;
    float _output;
    float Ts;
}FOC_PID;
void PID_Init(FOC_PID *foc_pid, float pid_freq);
void FOC_PID_SPEED(FOC_PID* S,float target_speed,float current_speed) ;
#endif //FOC_PID_H
