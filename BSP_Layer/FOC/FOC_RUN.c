#include "FOC_RUN.h"

#include <tgmath.h>

#include "as5600.h"
#include "cmsis_os2.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
// 新增静态变量保存时间戳（放在函数外或用SPWM结构体存储）
static uint32_t last_tick = 0;

void FOC_SPWM_OPEN_RUN(SPWM *spwm, double target_speed) {

    spwm->qd.uq = 3.0f;
    spwm->qd.ud = 0.0f;
    double dt = 0.001f;
    static int first = 0;
    static uint32_t last_tick = 0; // 静态保存

    uint32_t current_tick = HAL_GetTick();
    if(last_tick != 0) {
        dt = (current_tick - last_tick) / 1000.0f;
        if(dt > 0.1f) {
            dt = 0.001f;
        }
    }
    if(first == 0) {
        double angle=AS5600_GetAngleDegrees(&i2c_AS5600 );
        spwm->elect_angle = Solve_Electrical_Angle(angle);
        first = 1;
    }
    // 角度平滑累加
    spwm->elect_angle += target_speed * dt;
    // 平滑取模归一化，杜绝垂直跳变
    spwm->elect_angle = fmod(spwm->elect_angle, 2.0 * M_PI);
    if(spwm->elect_angle < 0) spwm->elect_angle += 2*M_PI;

    FOC_Spwm_Solve(spwm);
    Set_Spwm(spwm);

    last_tick = current_tick; // 【关键补全】每次循环更新时间戳

    FOC_Spwm_Solve(spwm);
    Set_Spwm(spwm);
    // OTTO_uart(&huart_debug,
    //           "%.2f,%.2f,%.2f,%.2f,%.2f",
    //           spwm->abc_v.ua,spwm->abc_v.ub,spwm->abc_v.uc,
    //           spwm->qd.uq,spwm->elect_angle);
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
    svpwm->K = 0.95f; // 过调制系数初始化（默认1.0）
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
    svpwm->elect_angle += target_speed * dt;
    svpwm->elect_angle = Limit_Angle(svpwm->elect_angle); // 归一化到0~2π
    FOC_Svpwm_Solve(svpwm);
    N = Sector_Judgment(svpwm, &svpwm->sector);
    // 矢量时间计算
    VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
    SVPWM temp = *svpwm;
    // OTTO_uart(&huart_bluetooth,
    //       "%f,%f,%f,%f,%f",
    //       svpwm->_output.ua,svpwm->_output.ub,svpwm->_output.uc,
    //       svpwm->qd.uq,svpwm->elect_angle);
    last_tick = current_tick; // 更新时间戳
}
void SVPWM_CloseLoop(SVPWM *svpwm, double target_speed, double dt) {
    // 1. 生成平滑目标角度（和开环累加逻辑一致）
    static double theta_ref = 0.0;
    const double twoPI = 2 * M_PI;
    theta_ref += target_speed * dt * 100;
    theta_ref = fmod(theta_ref, twoPI);
    if(theta_ref < 0) theta_ref += twoPI;

    // 2. 读取当前真实电角度（来自AS5600）
    double theta_real = svpwm->elect_angle;

    // 3. 角度偏差（最简单定向补偿，无PID）
    // 直接用偏差修正参与反Park变换的角度，让dq轴贴合转子
    double theta_err = theta_ref - theta_real;

    // 角度偏差限幅 -π ~ +π，防止跳变
    if(theta_err > M_PI) theta_err -= twoPI;
    if(theta_err < -M_PI) theta_err += twoPI;

    // ========== 核心：直接修正电角度，实现角度闭环 ==========
    // 把偏差叠加到真实角度上，定子磁场自动跟随目标角度
    svpwm->elect_angle = theta_real + theta_err;
    svpwm->elect_angle = fmod(svpwm->elect_angle, twoPI);
    if(svpwm->elect_angle < 0) svpwm->elect_angle += twoPI;

    // 4. 恒定直交轴电压（沿用你开环参数，无需调PID）
    svpwm->qd.ud = 0.0f;
    svpwm->qd.uq = 2.0f; // 转矩电压，和之前一致，改这里调快慢
    // FOC_Svpwm_Solve(svpwm);
    // uint8_t N = Sector_Judgment(svpwm, &svpwm->sector);
    // // 矢量时间计算
    // VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    // Set_Svpwm(svpwm);
}
void SVPWM_TIM_RUN(SVPWM *svpwm,float target_speed) {
    // ========== 1. 固定dt，彻底消除周期抖动 ==========
    // f_pwm=10kHz → dt=0.0001s，恒定不变，不再用HAL_GetTick计时
    const double dt = 0.0001;

    // ========== 2. 实时读取编码器电角度（I2C放临界区，极快） ==========
    double raw_mech_angle = AS5600_GetAngleDegrees(&i2c_AS5600);
    svpwm->elect_angle = Solve_Electrical_Angle(raw_mech_angle);

    // ========== 3. 无PID角度闭环（第二部分代码） ==========
    SVPWM_CloseLoop(svpwm, target_speed, dt);

    // ========== 4. 标准SVPWM计算 ==========
    FOC_Svpwm_Solve(svpwm);
    uint8_t sector = Sector_Judgment(svpwm, &svpwm->sector);
    VectorActionTime(svpwm, sector, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
}
FOC_PID speed_pid={
    .kp = 2.2f,
    .ki = 250.0f,
    .kd = 1.0f,
    ._output        = 0.0f,
    .error          = 0.0f,
    .last_error     = 0.0f,
    .prev_error     = 0.0f,
    .sum_error      = 0.0f,
    .Ts             = 0.0001f,
    .increment_limit = Increment_Limit,
    .output_max =LIN_V,
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
float speed=0;
void FOC_SVPWM_SPEED_RUN(SVPWM *svpwm,float target_speed_RPS) {
    //获取当前速度
    float current_angle=AS5600_GetAngleDegrees(&i2c_AS5600);
    float current_speed_RPS = AS5600_CalcSpeed_MovAvg(current_angle);
    speed=current_speed_RPS;
   // float actual_speed=FOC_SPEED_FILL(current_speed_RPS);
    FOC_PID_SPEED(&speed_pid,target_speed_RPS,current_speed_RPS);
    //输出限幅
    speed_pid._output=constrain(speed_pid._output,-LIN_V,LIN_V);
    svpwm->qd.uq = speed_pid._output;
    svpwm->qd.ud =0;
    //计算电角度
    svpwm->elect_angle = Solve_Electrical_Angle(current_angle);
    //svpwm实现
    FOC_Svpwm_Solve(svpwm);
    uint8_t N = Sector_Judgment(svpwm, &svpwm->sector);
    // 矢量时间计算
    VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
    SVPWM temp = *svpwm;
}

void SPWM_RUN(SPWM *spwm, FOC_RUN_STATE state) {
    switch (state) {
        case FOC_SPWM_OPEN_MODE:
            FOC_SPWM_OPEN_RUN(spwm,100);
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
            SVPWM_TIM_RUN(svpwm,10);
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
