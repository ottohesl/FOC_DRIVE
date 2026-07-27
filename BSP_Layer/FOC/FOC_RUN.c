#include "FOC_RUN.h"
// 新增静态变量保存时间戳（放在函数外或用SPWM结构体存储）
static uint32_t last_tick = 0;
VOFA_DATA vofa_data_run ={
.now_angle = 0.0f,
    .now_speed = 0.0f,
    .targe_angle = 0.0f,
    .targe_speed =  0.0f,
    .elect_angle = 0.0f,
    .Udata.A = 0.0f,
    .Udata.B = 0.0f,
    .Udata.C = 0.0f,
    .Idata.A = 0.0f,
    .Idata.B = 0.0f,
    .Idata.C = 0.0f,
    .uq = 0.0f,
    .ud = 0.0f,
    .iq = 0.0f,
    .id = 0.0f
};

/**
 * 动态一阶RC滤波
 * @param x 输入值
 * @return 滤波值
 */
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

/**
 * @brief spwm类的开环
 * @param spwm spwm句柄
 * @param target_speed 目标速度输入（非精确值）
 */
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
    FOC_Set_Spwm(spwm,LIN_V);
    last_tick = current_tick;
}

/**
 * @brief svpwm类的开环
 * @param svpwm svpwm句柄
 * @param target_speed 目标速度（非精确值）
 */
void FOC_SVPWM_OPEN_RUN(SVPWM *svpwm,double target_speed) {
    uint8_t N;
    svpwm->qd.uq = 3.0f;
    svpwm->qd.ud = 0.0f;
    double dt = 0.001f;    // 默认时间间隔（1ms）
    static int first = 0;
    uint32_t current_tick = HAL_GetTick();
    if(last_tick != 0) {
        dt = (current_tick - last_tick) / 1000.0f; // 转换为秒
        // 防异常：时间间隔超过100ms则重置（避免电机猛冲）
        if(dt > 0.1f) {
            dt = 0.001f;
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
    VectorActionTime(svpwm, N, LIN_V);
    FOC_Set_Svpwm(svpwm,LIN_V);
}
/**
 * @brief 速度环
 * @param motor 电机结构体句柄
 * @param target_speed 目标每秒转速
 * @return 输出给电流环
 */
float FOC_SPEED_DRIVE(FOC_DRIVE *motor,float target_speed) {
    //求解电角度
    motor->FOC_SVPWM.elect_angle = Solve_Electrical_Angle(motor->FOC_ENC_DATA.Enc_angle);
    //pid求解
    FOC_INC_PID(&motor->FOC_PID.spe_pid,target_speed,motor->FOC_ENC_DATA.Enc_speed);
    return -motor->FOC_PID.spe_pid._output;
}

/**
 * @brief 电流环
 * @param motor 电机结构体句柄
 * @param Iq 目标电流
 */
void FOC_CUR_DRIVE(FOC_DRIVE *motor,float Iq){
    //克拉克变换
    float Ialpha = motor->FOC_CUR_PARAM.Ia;
    float Ibeta = (motor->FOC_CUR_PARAM.Ib-motor->FOC_CUR_PARAM.Ic) * SQRT_3_DIV_2;
    //帕克变换
    float elect_angle =motor->FOC_SVPWM.elect_angle;
    motor->FOC_SVPWM.iqd.id = Ialpha * cosf(elect_angle) + Ibeta * sinf(elect_angle);
    motor->FOC_SVPWM.iqd.iq = -Ialpha * sinf(elect_angle) + Ibeta * cosf(elect_angle);
    //将给定需要的值进行pi调节
    FOC_POS_PID(&motor->FOC_PID.iq_pid,Iq,motor->FOC_SVPWM.iqd.iq);
    FOC_POS_PID(&motor->FOC_PID.id_pid,0,motor->FOC_SVPWM.iqd.id);
    //输出Uq，作用于电机
    motor->FOC_SVPWM.qd.uq = motor->FOC_PID.iq_pid._output;
    motor->FOC_SVPWM.qd.ud = motor->FOC_PID.id_pid._output;
    //反park变化获取alpha和beta
    FOC_Svpwm_Solve(&motor->FOC_SVPWM);
    //给svpwm让电机旋转
    // 矢量时间计算
    uint8_t N = Sector_Judgment(&motor->FOC_SVPWM, &motor->FOC_SVPWM.sector);
    VectorActionTime(&motor->FOC_SVPWM, N, motor->FOC_CUR_PARAM.Bus_Voltage);
    FOC_Set_Svpwm(&motor->FOC_SVPWM,motor->FOC_CUR_PARAM.Bus_Voltage);
}

void FOC_MOTOR_RUN(FOC_DRIVE *run_type, FOC_RUN_STATE state) {
    switch (state) {
        case FOC_OPEN_MODE:
            FOC_SVPWM_OPEN_RUN(&run_type->FOC_SVPWM,100);
            break;
        case FOC_SPEED_MODE:
            float speed=FOC_SPEED_DRIVE(run_type,5);
            FOC_CUR_DRIVE(run_type,speed);
            break;
        case FOC_POSTION_MODE:
            break;
        default:
            break;
    }
}
