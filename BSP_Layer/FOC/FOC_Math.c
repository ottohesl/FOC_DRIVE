
/****************************************************************************
 *  @author:      ottohesl
 *  @data  :      2026/3/9
 *  @details      基于stm32内核的foc算法，支持常见三相无刷直流电机（BLDC）
 *                该库实现底层FOC算法，包含帕克逆变换和克拉克逆变换
 *                支持spwm和svpwm算法
 * @version        V1.0
 */
#include "FOC_Math.h"
#include "arm_math.h"
#include "as5600.h"
#include "i2c.h"
#include "tim.h"
#include "main.h"
#include "ottohesl.h"
#include "usart.h"
/**************************基本使用函数*********************************/
volatile uint16_t elect_offset = 0;
inline float cosf(float x) { return arm_cos_f32(x); }
inline float sinf(float x) { return arm_sin_f32(x); }

/**
 * @brief 定时器基础常量获取（预分频器、自动重装载值）
 * @param tim_data SVPWM句柄
 * @param htim pwm产生定时器句柄
 */
static void FOC_TIM_INIT(TIM_DATA *tim_data,TIM_HandleTypeDef *htim) {
    tim_data->PSC = htim->Instance->PSC ;
    tim_data->ARR = htim->Instance->ARR ;
}
/**
 * @brief  角度归一化函数
 * @param  angle  输入的弧度角度
 */
double Limit_Angle(double angle) {
    const double para = fmod(angle, 2*M_PI);
    return para>=0 ? para : (para+2*M_PI);
}
/**
 * @brief  求解电角度函数
 * @param  angle  输入的当前机械角度
 */
double Solve_Electrical_Angle(double angle) {
    angle -= Read_E_Deviation;
    if (angle < 0) angle += 360;
    else if (angle > 360) angle -= 360;
    angle *=  (M_PI / 180.0f);
    return Limit_Angle(angle * POLE_PAIRS);
}
/**
 * @brief  求解电PWM的频率
 * @return pwm的频率值
 */
float Solve_PWM_Freq(TIM_DATA tim_data) {
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    uint32_t clk_val = (RCC->CFGR >> 8) & 0x07;
    if(clk_val != 0) {
        tim_clk *= 2;
    }
    float pwm_freq = (float)tim_clk / ((tim_data.PSC + 1) * (tim_data.ARR + 1));
    return pwm_freq;
}
/**
 * @brief  求解零偏纠正角
 * @param  spwm   SPWM结构体的句柄，访问里面数据进行读写
 */
void Calibrate_Zero_angle(SPWM *spwm) {
    spwm->elect_angle=0;
    spwm->qd.uq=0;
    spwm->qd.ud=3;
    FOC_Spwm_Solve(spwm);
    FOC_Set_Spwm(spwm,7.4f);
    //HAL_Delay(300);
    elect_offset=AS5600_ReadRawAngle(&i2c_AS5600);
}
/**************************spwm实现*********************************/
void FOC_SPWM_Init(SPWM *spwm) {
    FOC_TIM_INIT(&spwm->tim_data,FOC_TIM);
    spwm->elect_angle=0.0f;
    spwm->qd.uq=0.0f;
    spwm->qd.ud=0.0f;
}
/**
 * @brief  求解SPWM的相关系数
 * @param  spwm  SPWM结构体的句柄，访问里面数据进行读写
 */
void FOC_Spwm_Solve(SPWM *spwm) {
    //帕克逆变换
    double ualp = -spwm->qd.uq * sin(spwm->elect_angle) + spwm->qd.ud * cos(spwm->elect_angle);
    double ubet = spwm->qd.uq * cos(spwm->elect_angle) + spwm->qd.ud * sin(spwm->elect_angle);
    //克拉克逆变换
    spwm->vol_abc.ua=ualp;
    spwm->vol_abc.ub=(SQRT_3*ubet-ualp)/2;
    spwm->vol_abc.uc=(-ualp-SQRT_3*ubet)/2;
}
/**
 * @brief  求解电角度函数
 * @param  spwm  SPWM结构体的句柄，访问里面数据进行读写
 */
void FOC_Set_Spwm(SPWM *spwm,float bus_voltage){
    //抬升电压
    spwm->vol_abc.ua = spwm->vol_abc.ua + bus_voltage/2;
    spwm->vol_abc.ub = spwm->vol_abc.ub + bus_voltage/2;
    spwm->vol_abc.uc = spwm->vol_abc.uc + bus_voltage/2;
    double ua=constrain(spwm->vol_abc.ua,0.0f,bus_voltage);
    double ub=constrain(spwm->vol_abc.ub,0.0f,bus_voltage);
    double uc=constrain(spwm->vol_abc.uc,0.0f,bus_voltage);
    // 电压转化为占空比
    float dc_a=constrain(ua/bus_voltage,0.0f,1.0f);
    float dc_b=constrain(ub/bus_voltage,0.0f,1.0f);
    float dc_c=constrain(uc/bus_voltage,0.0f,1.0f);
    // 设置PWM比较值
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_1, dc_a * spwm->tim_data.ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_2, dc_b * spwm->tim_data.ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_3, dc_c * spwm->tim_data.ARR);
}
/**************************svpwm实现*********************************/
void FOC_SVPWM_Init(SVPWM *svpwm) {
    FOC_TIM_INIT(&svpwm->tim_data,FOC_TIM);
    svpwm->elect_angle=0.0f;
    svpwm->qd.uq=0.0f;
    svpwm->qd.ud=0.0f;
    svpwm->iqd.iq=0.0f;
    svpwm->iqd.id=0.0f;
    svpwm->tim_pwm = svpwm->tim_data.ARR + 1;
    svpwm->tim_scale = 0.95f; // 过调制系数初始化（0.92-1，越大零向量越小）
    svpwm->ab.alpha = 0.0f;
    svpwm->ab.beta = 0.0f;
    svpwm->svpwm_val1=0.0f;
    svpwm->svpwm_val2=0.0f;
}
/**
 * @brief 扇区判断
 * @param svpwm  SVPWM控制结构体
 * @param sector 输出扇区号
 * @return N值
 */
uint8_t Sector_Judgment(SVPWM *svpwm, uint8_t *sector)
{
    svpwm->svpwm_val1 = svpwm->ab.alpha * SQRT_3_DIV_2;
    svpwm->svpwm_val2 = svpwm->ab.beta / 2.0f;

    float A = svpwm->ab.beta;
    float B = svpwm->svpwm_val1 - svpwm->svpwm_val2;
    float C = -svpwm->svpwm_val1 - svpwm->svpwm_val2;

    uint8_t N = 0;
    if(A > 0)      N += 1;
    if(B > 0)      N += 2;
    if(C > 0)      N += 4;

    // 将N转换为扇区号(1-6)
    switch(N){
        case 3: *sector = 1; break;
        case 1: *sector = 2; break;
        case 5: *sector = 3; break;
        case 4: *sector = 4; break;
        case 6: *sector = 5; break;
        case 2: *sector = 6; break;
        default: *sector = 1; break; // 默认扇区1
    }
    return N;
}
/**
 * @brief 矢量作用时间计算
 * @param foc_pwm SVPWM控制结构体
 * @param sector 扇区
 * @param bus_voltage 母线电压
 */
void VectorActionTime(SVPWM *foc_pwm, uint8_t sector,float bus_voltage)
{
    bus_voltage = bus_voltage * 1.0f;
    float K = (float)foc_pwm->tim_pwm * SQRT_3 / bus_voltage;

    float X = K * foc_pwm->ab.beta;
    float Y = K * (foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);
    float Z = K * (-foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);

    uint32_t T4 = 0, T6 = 0;
    uint32_t Ta = 0, Tb = 0, Tc = 0;
    uint32_t T1 = 0, T2 = 0, T3 = 0;

    // 根据扇区计算矢量时间
    switch(sector){
        case 1: T4 = Z;  T6 = Y;  break;
        case 2: T4 = Y;  T6 = -X; break;
        case 3: T4 = -Z; T6 = X;  break;
        case 4: T4 = -X; T6 = Z;  break;
        case 5: T4 = X;  T6 = -Y; break;
        case 6: T4 = -Y; T6 = -Z; break;
    }

    // 过调制处理
    if(T4 + T6 > ((float)foc_pwm->tim_pwm * foc_pwm->tim_scale)){
        float ratio = ((float)foc_pwm->tim_pwm * foc_pwm->tim_scale) / (T4 + T6);
        T4 *= ratio;
        T6 *= ratio;
    }

    // 计算各相作用时间
    Ta = (foc_pwm->tim_pwm - T4 - T6) / 4;
    Tb = Ta + T4 / 2;
    Tc = Tb + T6 / 2;

    // 根据扇区分配时间到各相
    switch(sector){
        case 1: T1 = Tb; T2 = Ta; T3 = Tc; break;
        case 2: T1 = Ta; T2 = Tc; T3 = Tb; break;
        case 3: T1 = Ta; T2 = Tb; T3 = Tc; break;
        case 4: T1 = Tc; T2 = Tb; T3 = Ta; break;
        case 5: T1 = Tc; T2 = Ta; T3 = Tb; break;
        case 6: T1 = Tb; T2 = Tc; T3 = Ta; break;
    }
    // 死区补偿(根据实际硬件调整)
    uint32_t deadtime_comp = 0;
    T1 += deadtime_comp;
    T2 += deadtime_comp;
    T3 += deadtime_comp;
    // 保存各相时间
    foc_pwm->tim_abc.ua = T1;
    foc_pwm->tim_abc.ub = T2;
    foc_pwm->tim_abc.uc = T3;
}
/**
 * @brief 帕克逆变换
 * @param svpwm 计算结果记录于该结构体内
 */
void FOC_Svpwm_Solve(SVPWM *svpwm) {
    //帕克逆变换
    svpwm->ab.alpha = -svpwm->qd.uq * sin(svpwm->elect_angle) + svpwm->qd.ud * cos(svpwm->elect_angle);
    svpwm->ab.beta = svpwm->qd.uq * cos(svpwm->elect_angle) + svpwm->qd.ud * sin(svpwm->elect_angle);
}
void FOC_Set_Svpwm(SVPWM *svpwm,float bus_voltage){
    // 计算占空比
    svpwm->out_abc.ua = (float)svpwm->tim_abc.ua / (svpwm->tim_pwm/2.0f) * bus_voltage;
    svpwm->out_abc.ub = (float)svpwm->tim_abc.ub / (svpwm->tim_pwm/2.0f) * bus_voltage;
    svpwm->out_abc.uc = (float)svpwm->tim_abc.uc / (svpwm->tim_pwm/2.0f) * bus_voltage;
    double ua=constrain((svpwm->out_abc.ua),0.0f,bus_voltage);
    double ub=constrain((svpwm->out_abc.ub),0.0f,bus_voltage);
    double uc=constrain((svpwm->out_abc.uc),0.0f,bus_voltage);
    // 电压转化为占空比
    float dc_a=constrain(ua/bus_voltage,0.0f,1.0f);
    float dc_b=constrain(ub/bus_voltage,0.0f,1.0f);
    float dc_c=constrain(uc/bus_voltage,0.0f,1.0f);
    // 设置PWM比较值
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_1, dc_a * svpwm->tim_data.ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_2, dc_b * svpwm->tim_data.ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_3, dc_c * svpwm->tim_data.ARR);
}
