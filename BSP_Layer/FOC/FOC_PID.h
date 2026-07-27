#ifndef FOC_PID_H
#define FOC_PID_H
#include "main.h"
#define Increment_Limit   3.0   //电压最大增量幅值（v）
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float prev_error;
    float increment;    //积分累加项
    float increment_limit;
    float output_max;
    float _output;
    float Ts;
}INC_PID;             //增量式pid
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float integral_limit;
    float output_max;
    float _output;
    float Ts;
}POS_PID;           //位置式pid

typedef struct {
    INC_PID spe_pid;        //速度环
    POS_PID iq_pid;         //q轴电流环
    POS_PID id_pid;         //d轴电流环
    POS_PID pos_pid;        //位置环
}FOC_PID;
void FOC_PID_Init(FOC_PID *foc_pid);
void FOC_INC_PID(INC_PID* S,float target_val,float current_val);
void FOC_POS_PID(POS_PID *C, float target_cur, float current_cur);
#endif //FOC_PID_H
