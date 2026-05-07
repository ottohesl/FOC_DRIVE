#include "FOC_RUN.h"
#include "as5600.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
// 新增静态变量保存时间戳（放在函数外或用SPWM结构体存储）
static uint32_t last_tick = 0;

void FOC_SPWM_OPEN_RUN(SPWM *spwm, double target_speed) {

    spwm->qd.uq = 6.0f;       // 转矩电压（V），可根据负载调整（2~3.5V）
    spwm->qd.ud = 0.0f;       // 励磁电压，开环必须设0
    double dt = 0.001f;    // 默认时间间隔（1ms）
    static int first = 0;
    uint32_t current_tick = HAL_GetTick();
    if(last_tick != 0) {
        dt = (current_tick - last_tick) / 1000.0f; // 转换为秒
        // 防异常：时间间隔超过100ms则重置（避免电机猛冲）
        if(dt > 0.1f) {
            dt = 0.001f;
            spwm->elect_angle = Limit_Angle(spwm->elect_angle); // 保持角度归一化
        }
    }
    if(first == 0) {
        last_tick = current_tick; // 更新时间戳
        double angle=AS5600_GetAngleDegrees(&i2c_AS5600 );
        spwm->elect_angle = Solve_Electrical_Angle(angle);
        first = 1;
    }
    spwm->elect_angle += target_speed * dt;
    spwm->elect_angle = Limit_Angle(spwm->elect_angle); // 归一化到0~2π

    FOC_Spwm_Solve(spwm);
    Set_Spwm(spwm);

    OTTO_uart(&huart_debug,
              "%.2f,%.2f,%.2f,%.2f,%.2f",
              spwm->abc_v.ua,spwm->abc_v.ub,spwm->abc_v.uc,
              spwm->qd.uq,spwm->elect_angle);
}
void FOC_SVPWM_OPEN_RUN(SVPWM *svpwm,double target_speed) {
    uint8_t N;
    svpwm->qd.uq = 3.0f;
    svpwm->qd.ud = 0.0f;
    double dt = 0.001f;    // 默认时间间隔（1ms）
    static int first = 0;
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    svpwm->Tpwm = ARR + 1;
    svpwm->K = 1.0f; // 过调制系数初始化（默认1.0）
    uint32_t current_tick = HAL_GetTick();
    if(last_tick != 0) {
        dt = (current_tick - last_tick) / 1000.0f; // 转换为秒
        // 防异常：时间间隔超过100ms则重置（避免电机猛冲）
        if(dt > 0.1f) {
            dt = 0.001f;
            svpwm->elect_angle = Limit_Angle(svpwm->elect_angle); // 保持角度归一化
        }
    }
    if(first == 0) {
        last_tick = current_tick; // 更新时间戳
        double angle=AS5600_GetAngleDegrees(&i2c_AS5600 );
        svpwm->elect_angle = Solve_Electrical_Angle(angle);
        first = 1;
    }
    last_tick = current_tick; // 更新时间戳
    svpwm->elect_angle += target_speed * dt;
    svpwm->elect_angle = Limit_Angle(svpwm->elect_angle); // 归一化到0~2π
    FOC_Svpwm_Solve(svpwm);
    N = Sector_Judgment(svpwm, &svpwm->sector);
    // 矢量时间计算
    VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
    OTTO_uart(&huart_debug,
          "%f,%f,%f,%f,%f",
          svpwm->_output.ua,svpwm->_output.ub,svpwm->_output.uc,
          svpwm->qd.uq,svpwm->elect_angle);
}
FOC_PID speed_pid={
    .kp = 2.0f,
    .ki = 1.0f,
    .kd = 0.0f,
    ._output        = 0.0f,
    .error          = 0.0f,
    .last_error     = 0.0f,
    .prev_error     = 0.0f,
    .sum_error      = 0.0f,
    .Ts             = 1.0f/PID_FIEQ,
};
float FOC_SPEED_FILL(float x) {
    float Ts = 1.0f/PID_FIEQ;
    uint32_t current_tick = HAL_GetTick();
    static uint32_t last_tick=0;
    static float last_x=0.0f;
    uint32_t times=(current_tick - last_tick)/ 1000;
    if (times>0.3) {
        last_x = x;
        last_tick = current_tick;
        return x;
    }
    float diff = Ts/(Ts+times);
    float y = diff * x + last_x * (1-diff);
    last_x = x;
    last_tick = current_tick;
    return y;
}
void FOC_SVPWM_SPEED_RUN(SVPWM *svpwm,float target_speed_RPS) {
    //初始化pid与svpwm
    speed_pid.increment_limit = Increment_Limit;
    speed_pid.output_max = LIN_V;
    //获取当前速度
    float current_speed_RPS = AS5600_Get_Speed(&i2c_AS5600);
   // float actual_speed=FOC_SPEED_FILL(current_speed_RPS);
    FOC_PID_SPEED(&speed_pid,target_speed_RPS,current_speed_RPS);
    //输出限幅
    speed_pid._output=constrain(speed_pid._output,-LIN_V,LIN_V);
    svpwm->qd.uq = speed_pid._output;
    svpwm->qd.ud = 0;
    //计算电角度
    double current_angle=AS5600_GetAngleDegrees(&i2c_AS5600);
    svpwm->elect_angle = -Solve_Electrical_Angle(current_angle);
    //svpwm实现
    FOC_Svpwm_Solve(svpwm);
    uint8_t N = Sector_Judgment(svpwm, &svpwm->sector);
    // 矢量时间计算
    VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
    OTTO_uart(&huart_debug,
          "%f,%f,%f,%f,%f,%f,%f",
          svpwm->_output.ua+LIN_V/2,svpwm->_output.ub+LIN_V/2,svpwm->_output.uc+LIN_V/2,
          svpwm->qd.uq,svpwm->elect_angle,current_speed_RPS,target_speed_RPS);
}

void SPWM_RUN(SPWM *spwm, FOC_RUN_STATE state) {
    switch (state) {
        case FOC_SPWM_OPEN_MODE:
            FOC_SPWM_OPEN_RUN(spwm,300);
            break;
        case FOC_SPWM_SPEED_MODE:
            break;
        case FOC_SPWM_POSTION_MODE:
            break;
        case FOC_SPWM_STOP_MODE:
            break;
        default:
            break;
    }
}
void SVPWM_RUN(SVPWM *svpwm, FOC_RUN_STATE state) {
    switch (state) {
        case FOC_SVPWM_OPEN_MODE:
            FOC_SVPWM_OPEN_RUN(svpwm,100);
            break;
        case FOC_SVPWM_SPEED_MODE:
            FOC_SVPWM_SPEED_RUN(svpwm,2);
            break;
        case FOC_SVPWM_POSTION_MODE:
            break;
        case FOC_SVPWM_STOP_MODE:
            break;
        default:
            break;
    }
}
