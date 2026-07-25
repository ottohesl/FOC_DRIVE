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

    vofa_data_run.elect_angle = svpwm->elect_angle;
    vofa_data_run.Udata.A = svpwm->abc_v.ua;
    vofa_data_run.Udata.B = svpwm->abc_v.ub;
    vofa_data_run.Udata.C = svpwm->abc_v.uc;
    vofa_data_run.uq = svpwm->qd.uq;
    vofa_data_run.ud = svpwm->qd.ud;
    //vofa_data_run.now_speed = ;

    //osMessageQueuePut(VOFAHandle,(&temp),0,0);
    OTTO_usb_cdc("%f,%f,%f,%f,%f", svpwm->_output.ua,svpwm->_output.ub,svpwm->_output.uc,
  svpwm->qd.uq,svpwm->elect_angle);
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
    svpwm->qd.ud = 1.0f;
    svpwm->qd.uq = 3.0f; // 转矩电压，和之前一致，改这里调快慢
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
    SVPWM temp = *svpwm;
    //osMessageQueuePut(VOFAHandle,(&temp),0,0);
    OTTO_usb_cdc(
      "%f,%f,%f,%f,%f",
      svpwm->_output.ua,svpwm->_output.ub,svpwm->_output.uc,
      svpwm->qd.uq,svpwm->elect_angle);
}
FOC_PID speed_pid={
    .kp = 0.18f,
    .ki = 50.0f,
    .kd = 0.002f,
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
float FOC_SVPWM_SPEED_RUN(SVPWM *svpwm,float target_speed_RPS) {
    //获取当前速度
    //float current_angle=AS5600_GetAngleDegrees(&i2c_AS5600);
    float current_angle = 0.0f;
    static float last_valid_angle = 0.0f;
    uint32_t timeout = 100; // 循环超时阈值
    while(timeout--)
    {
        current_angle = AS5600_GetAngleDegrees(&i2c_AS5600);
        if(!isnan(current_angle)) break;
    }
    // 超时仍为NaN，使用上一帧合法角度兜底，防止测速NaN
    if(isnan(current_angle))
    {
        current_angle = last_valid_angle;
    }
    else
    {
        last_valid_angle = current_angle;
    }
    float current_speed_RPS = AS5600_CalcSpeed_MovAvg(current_angle);
    //求解电角度
    FOC_SPEED_FILL(current_speed_RPS);
    svpwm->elect_angle = Solve_Electrical_Angle(current_angle);
   // float actual_speed=FOC_SPEED_FILL(current_speed_RPS);
    //float uq=foc_pid_speed(target_speed_RPS,current_speed_RPS);
    //输出限幅
    FOC_PID_SPEED(&speed_pid,target_speed_RPS,current_speed_RPS);
    vofa_data_run.now_speed = current_speed_RPS;
    vofa_data_run.targe_speed = target_speed_RPS;
    return -speed_pid._output;
    // svpwm->qd.uq = speed_pid._output;
    // svpwm->qd.ud =0;
    // //计算电角度
    // svpwm->elect_angle = Solve_Electrical_Angle(current_angle);
    // //svpwm实现
    // // 矢量时间计算
    // VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    // Set_Svpwm(svpwm);
    // vofa_data_run.elect_angle = svpwm->elect_angle;
    // vofa_data_run.Udata.A = svpwm->abc_v.ua;
    // vofa_data_run.Udata.B = svpwm->abc_v.ub;
    // vofa_data_run.Udata.C = svpwm->abc_v.uc;
    // vofa_data_run.uq = svpwm->qd.uq;
    // vofa_data_run.ud = svpwm->qd.ud;
}
FOC_PID cur_pi_q={
    .kp = 3.5f,
    .ki = 300.0f,
    ._output        = 0.0f,
    .error          = 0.0f,
    .last_error     = 0.0f,
    .Ts             = 0.0001f,
    .increment = 0.0f,
    .integral = 0.0f,
    .integral_limit = LIN_V * 10,
    .increment_limit = Increment_Limit,
    .output_max = LIN_V,
};
FOC_PID cur_pi_d={
    .kp = 4.4f,
    .ki = 500.0f,
    ._output        = 0.0f,
    .error          = 0.0f,
    .last_error     = 0.0f,
    .Ts             = 0.0001f,
    .increment = 0.0f,
    .integral = 0.0f,
    .integral_limit = LIN_V * 10,
    .increment_limit = Increment_Limit,
    .output_max = LIN_V,
};
//测试状态，电压实际是电流

void FOC_SVPWM_CUR_RUN(SVPWM *svpwm,float Iq) {
    //获取当前角度
    // float current_angle = 0.0f;
    // static float last_valid_angle = 0.0f;
    // uint32_t timeout = 100; // 循环超时阈值
    // while(timeout--)
    // {
    //     current_angle = AS5600_GetAngleDegrees(&i2c_AS5600);
    //     if(!isnan(current_angle)) break;
    // }
    // 超时仍为NaN，使用上一帧合法角度兜底，防止测速NaN
    // if(isnan(current_angle))current_angle = last_valid_angle;
    // else last_valid_angle = current_angle;
    //获取当前扇区
    uint8_t N = Sector_Judgment(svpwm, &svpwm->sector);
    //获取当前电流
    float Ia=0,Ib=0,Ic=0;

    Get_Phase_Sequence(&Ia,&Ib,&Ic,N);
    //克拉克变换
    float Ialpha = Ia;
    float Ibeta = (Ib-Ic) * SQRT_3_DIV_2;
    //帕克变换
    float Idr  =  Ialpha * cosf(svpwm->elect_angle) + Ibeta * sinf(svpwm->elect_angle);
    float Iqr = -Ialpha * sinf(svpwm->elect_angle) + Ibeta * cosf(svpwm->elect_angle);
    // FOC_SPEED_FILL(Iqr);
    // FOC_SPEED_FILL(Idr);
    //将给定需要的值进行pi调节
    FOC_PID_CUR(&cur_pi_q,Iq,Iqr);
    FOC_PID_CUR(&cur_pi_d,0,Idr);
    //输出Uq，作用于电机
    //float Uq = cur_pi._output;
    svpwm->qd.uq=cur_pi_q._output;
    svpwm->qd.ud=0;
    //反park变化获取alpha和beta
    FOC_Svpwm_Solve(svpwm);
    //给svpwm让电机旋转
    // 矢量时间计算
    VectorActionTime(svpwm, N, svpwm->Tpwm, LIN_V);
    Set_Svpwm(svpwm);
    //vofa数据传递
    vofa_data_run.elect_angle = Iq;
    vofa_data_run.Udata.A = svpwm->_output.ua;
    vofa_data_run.Udata.B = svpwm->_output.ub;
    vofa_data_run.Udata.C = svpwm->_output.uc;
    vofa_data_run.uq = svpwm->qd.uq;
    vofa_data_run.ud = svpwm->qd.ud;
    vofa_data_run.iq = Iqr;
    vofa_data_run.id = Idr;
    vofa_data_run.Idata.A = Ia;
    vofa_data_run.Idata.C = Ic;
    vofa_data_run.Idata.B = Ib;
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
            //SVPWM_TIM_RUN(svpwm,10);
            FOC_SVPWM_OPEN_RUN(svpwm,100);
            break;
        case FOC_SVPWM_SPEED_MODE:
            float speed=FOC_SVPWM_SPEED_RUN(svpwm,0);
            FOC_SVPWM_CUR_RUN(svpwm,speed);
            break;
        case FOC_SVPWM_POSTION_MODE:
            break;
        case FOC_SVPWM_STOP_MODE:
            break;
        default:
            break;
    }
}
