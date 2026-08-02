#include "FOC_RUN.h"

#include "Filter_Tool/Filter_Tool.h"
static uint32_t last_tick = 0;
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
        double angle=svpwm->elect_angle;
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
    //pid求解
    float dead_spe = 0.05f;
    if (fabsf(motor->FOC_ENC_DATA.Enc_speed)<dead_spe) motor->FOC_ENC_DATA.Enc_speed = 0.0f;    //限制速度值抖动
    FOC_INC_PID(&motor->FOC_PID.spe_pid,target_speed,motor->FOC_ENC_DATA.Enc_speed);
    return -motor->FOC_PID.spe_pid._output;
}
FILTER_TOOL q_filter = {
    .fit_last_data = 0,
    .filtered_data = 0,
    .fit_allow_max = 20,
    .filter_size = 30,
    .fit_index = 0,
    .fit_buf = {0}
};
FILTER_TOOL d_filter = {
    .fit_last_data = 0,
    .filtered_data = 0,
};
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
    //求解电角度
    motor->FOC_SVPWM.elect_angle = Solve_Electrical_Angle(motor->FOC_ENC_DATA.Enc_angle);
    motor->FOC_SVPWM.iqd.id = Ialpha * cosf(motor->FOC_SVPWM.elect_angle) + Ibeta * sinf(motor->FOC_SVPWM.elect_angle);
    motor->FOC_SVPWM.iqd.iq = -Ialpha * sinf(motor->FOC_SVPWM.elect_angle) + Ibeta * cosf(motor->FOC_SVPWM.elect_angle);
    //对q、d轴电流进行rc一阶滤波
    q_filter.raw_data = motor->FOC_SVPWM.iqd.iq;
    d_filter.raw_data = motor->FOC_SVPWM.iqd.id;
    FILTER_RC(&q_filter);
    FILTER_RC(&d_filter);
    //对iq进行均值滤波
    q_filter.raw_data = q_filter.filtered_data;
    FILTER_Sliding_Mean(&q_filter);
    motor->FOC_SVPWM.iqd.iq = q_filter.filtered_data;
    //将给定需要的值进行pi调节
    motor->FOC_PID.iq_pid.output_max = motor->FOC_CUR_PARAM.Bus_Voltage;  //将当前的实际adc电压作为输出上限
    motor->FOC_PID.id_pid.output_max = motor->FOC_CUR_PARAM.Bus_Voltage;  //将当前的实际adc电压作为输出上限
    FOC_POS_PID(&motor->FOC_PID.iq_pid,Iq,motor->FOC_SVPWM.iqd.iq);
    FOC_POS_PID(&motor->FOC_PID.id_pid,0,motor->FOC_SVPWM.iqd.id);
    //输出Uq，作用于电机
    motor->FOC_SVPWM.qd.uq = motor->FOC_PID.iq_pid._output;
    motor->FOC_SVPWM.qd.ud = 0;
    //反park变化获取alpha和beta
    FOC_Svpwm_Solve(&motor->FOC_SVPWM);
    //给svpwm让电机旋转
    // 矢量时间计算
    uint8_t N = Sector_Judgment(&motor->FOC_SVPWM, &motor->FOC_SVPWM.sector);
    VectorActionTime(&motor->FOC_SVPWM, N, motor->FOC_CUR_PARAM.Bus_Voltage);
    FOC_Set_Svpwm(&motor->FOC_SVPWM,motor->FOC_CUR_PARAM.Bus_Voltage);
}
void FOC_MOTOR_STOP() {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    // // 互补通道
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}
void FOC_MOTOR_OPEN() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // // 互补通道
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
void FOC_MOTOR_RUN(FOC_DRIVE *run_type, FOC_RUN_STATE state) {
    switch (state) {
        case FOC_OPEN_MODE:
            FOC_SVPWM_OPEN_RUN(&run_type->FOC_SVPWM,50);
            break;
        case FOC_SPEED_MODE:
            MOTOR.FOC_PARAM.iq=FOC_SPEED_DRIVE(run_type,run_type->FOC_PARAM.speed);
            FOC_CUR_DRIVE(run_type,MOTOR.FOC_PARAM.iq);
            break;
        case FOC_POSTION_MODE:
            break;
        default:
            break;
    }
}
