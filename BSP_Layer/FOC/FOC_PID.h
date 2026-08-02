#ifndef FOC_PID_H
#define FOC_PID_H
#include "main.h"
#define Increment_Limit   2.0f    //增量式单次增量最大值
#define Integral_Limit   100.0f    //位置式积分最大限幅
#define LIN_V  12.4f          // 设定基础母线电压
#define SPE_LIN  74.0f        //速度环力矩最大值(当前母线电压最大值*10)
#define cur_kp 0.1f           //iq稳定建议kp=1，ki=52.5
#define cur_ki 0.3f           //iq越不稳定越要降低kp和ki的力矩影响，以提升速度环的精准度
#define spe_kp 0.15f
#define spe_ki 80.5f
#define spe_kd 10.00f
#define pos_kp 0.0f
#define pos_ki 0.0f
#define pos_kd 0.0f
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
    float dead_line;    //死区时间
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
    float dead_line;    //死区区间
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
