#include "FOC_PID.h"
/**
 * @brief PID初始化
 * @param foc_pid
 * @param pid_freq PID执行频率
 */
void PID_Init(FOC_PID *foc_pid, float pid_freq)
{
    foc_pid->kp = 0.2f;
    foc_pid->ki = 10.0f;
    foc_pid->kd = 0.0f;
    foc_pid->_output        = 0.0f;
    foc_pid->error          = 0.0f;
    foc_pid->last_error     = 0.0f;
    foc_pid->prev_error     = 0.0f;
    foc_pid->sum_error      = 0.0f;
    foc_pid->Ts             = 1.0f/pid_freq;
}
/**
 * @brief  增量式速度环pid
 * @param S               FOC_PID结构体的速度指针
 * @param target_speed    目标速度
 * @param current_speed   当前速度
 */
void FOC_PID_SPEED(FOC_PID* S,float target_speed,float current_speed)
{
    S->error = target_speed-current_speed;
    float P = S->kp*(S->error-S->last_error);
    float I = S->ki*S->error*S->Ts;
    float D = S->kd*(S->error-2*S->last_error+S->prev_error)*S->Ts;
    float increment = P+I+D;
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