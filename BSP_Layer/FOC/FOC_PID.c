#include "FOC_PID.h"

#include <math.h>

#include "FOC_Math.h"
#include "ottohesl.h"
static void PID_Speed_Init(INC_PID *inc) {
    inc->kp = 0.18f;
    inc->ki = 50.0f;
    inc->kd = 0.002f;
    inc->error = 0.0f;
    inc->last_error = 0.0f;
    inc->prev_error = 0.0f;
    inc->increment = 0.0f;
    inc->increment_limit = Increment_Limit;
    inc->Ts = 0.0001f;
    inc->output_max = LIN_V;
    inc->_output = 0.0f;
}
static void PID_Iq_Init(POS_PID *pos) {
    pos->kp = 3.5f;
    pos->ki = 300.0f;
    pos->kd = 0.0f;
    pos->error = 0.0f;
    pos->last_error = 0.0f;
    pos->integral = 0.0f;
    pos->integral_limit = Increment_Limit;
    pos->Ts = 0.0001f;
    pos->output_max = LIN_V;
    pos->_output = 0.0f;
}
static void PID_Id_Init(POS_PID *pos) {
    pos->kp = 4.4f;
    pos->ki = 500.0f;
    pos->kd = 0.0f;
    pos->error = 0.0f;
    pos->last_error = 0.0f;
    pos->integral = 0.0f;
    pos->integral_limit = Increment_Limit;
    pos->Ts = 0.0001f;
    pos->output_max = LIN_V;
    pos->_output = 0.0f;
}
static void PID_POS_Init(POS_PID *pos) {
    pos->kp = 0.0f;
    pos->ki = 0.0f;
    pos->kd = 0.0f;
    pos->error = 0.0f;
    pos->last_error = 0.0f;
    pos->integral = 0.0f;
    pos->integral_limit = LIN_V * 10.0f;
    pos->Ts = 0.0001f;
    pos->output_max = LIN_V;
    pos->_output = 0.0f;
}
/**
 * @brief PID初始化
 * @param foc_pid
 */
void FOC_PID_Init(FOC_PID *foc_pid)
{
    PID_Speed_Init(&foc_pid->spe_pid);
    PID_Iq_Init(&foc_pid->iq_pid);
    PID_Id_Init(&foc_pid->id_pid);
    PID_POS_Init(&foc_pid->pos_pid);
}
/**
 * @brief 增量式速度环pid，可用于速度环
 * @param S   增量式结构体的速度指针
 * @param target_val    目标速度
 * @param current_val   当前速度
 */
void FOC_INC_PID(INC_PID* S,float target_val,float current_val)
{
    S->error = target_val-current_val;
    float P = S->kp*(S->error-S->last_error);
    float I = S->ki*S->error*S->Ts;
    float D = S->kd*(S->error-2*S->last_error+S->prev_error)*S->Ts;
    float increment = P + I + D;
    //增量限幅
    if (increment > S->increment_limit) increment = S->increment_limit;
    else if(increment < -S->increment_limit) increment = -S->increment_limit;
    S->_output += increment;
    S->prev_error = S->last_error;
    S->last_error = S->error;
    //输出限幅
    if(S->_output > S->output_max) S->_output=S->output_max;
    else if (S->_output < -S->output_max) S->_output = -S->output_max;
}

/**
 * @brief 位置式pid，可用于电流环、位置环
 * @param C 位置式pid结构体句柄
 * @param target_cur 目标值
 * @param current_cur 当前值
 */
void FOC_POS_PID(POS_PID *C, float target_cur, float current_cur)
{
    C->error = target_cur-current_cur;
    C->integral += C->error ;  //积分累加项
    if(C->integral > C->integral_limit) C->integral = C->integral_limit;
    if(C->integral < -C->integral_limit) C->integral = -C->integral_limit;
    float P = C->kp * C->error;
    float I = C->ki * C->integral * C->Ts; //积分累加
    float D = C->kd * (C->error-C->last_error )* C->Ts;
    float output = P + I + D;
    if (output > C->output_max) output = C->output_max;
    else if (output < -C->output_max) output = -C->output_max;
    C->last_error = C->error;
    C->_output = output;
}