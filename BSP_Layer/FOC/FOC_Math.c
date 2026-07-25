
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
 * @brief  获取定时器pwm下的预分频器和自动重装载值
 * @param   htim 定时器句柄
 * @param   psc  预分频器
 * @param   arr  自动重装载值
 */
void Get_PSC_ARR(TIM_HandleTypeDef *htim , uint32_t *psc ,uint32_t *arr) {
    *psc = htim->Instance->PSC ;
    *arr = htim->Instance->ARR ;
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
 * @param  无
 * @return pwm的频率值
 */
float Solve_PWM_Freq() {
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x07;
    if(ppre1 != 0) {
        tim_clk *= 2;
    }
    float pwm_freq = (float)tim_clk / ((PSC + 1) * (ARR + 1));
    return pwm_freq;
}
/**
 * @brief  求解电角度函数
 * @param  spwm  SPWM结构体的句柄，访问里面数据进行读写
 */
void Set_Spwm(SPWM *spwm){
    double ua=constrain(spwm->abc_v.ua,0.0f,LIN_V);
    double ub=constrain(spwm->abc_v.ub,0.0f,LIN_V);
    double uc=constrain(spwm->abc_v.uc,0.0f,LIN_V);
    // 电压转化为占空比
    float dc_a=constrain(ua/LIN_V,0.0f,1.0f);
    float dc_b=constrain(ub/LIN_V,0.0f,1.0f);
    float dc_c=constrain(uc/LIN_V,0.0f,1.0f);
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    // 设置PWM比较值
    // OTTO_uart(&huart_debug,"%.2f,%.2f,%.2f,%d",dc_a*ARR,dc_b*ARR,dc_c*ARR,ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_1, dc_a*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_2, dc_b*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_3, dc_c*ARR);
}
/**
 * @brief  求解零偏纠正角
 * @param  spwm   SPWM结构体的句柄，访问里面数据进行读写
 */
void Calibrate_Zero_Eangle(SPWM *spwm) {
    spwm->elect_angle=0;
    spwm->qd.uq=0;
    spwm->qd.ud=3;
    FOC_Spwm_Solve(spwm);
    Set_Spwm(spwm);
    //HAL_Delay(300);
    elect_offset=AS5600_ReadRawAngle(&i2c_AS5600);
    //OTTO_uart(&huart_debug,"原始编码器值为：%f",elect_offset);
    OTTO_usb_cdc("编码器角度为：%d",elect_offset);
}
/**************************spwm实现*********************************/
void Init_SVPWM(SVPWM *svpwm) {
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    svpwm->elect_angle=0.0f;
    svpwm->qd.uq=0.0f;
    svpwm->qd.ud=0.0f;
    svpwm->Tpwm = ARR + 1;
    svpwm->K = 0.95f; // 过调制系数初始化
    svpwm->ab.alpha = 0.0f;
    svpwm->ab.beta = 0.0f;
    svpwm->svpwm_val1=0.0f;
    svpwm->svpwm_val2=0.0f;
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
    spwm->abc_v.ua=ualp+LIMIT_V/2;
    spwm->abc_v.ub=(SQRT_3*ubet-ualp)/2+LIMIT_V/2;
    spwm->abc_v.uc=(-ualp-SQRT_3*ubet)/2+LIMIT_V/2;
}
/**************************svpwm实现*********************************/
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
 * @param alphabeta αβ轴电压
 * @param Tpwm PWM周期(计数值)
 * @param Udc 母线电压
 */
void VectorActionTime(SVPWM *foc_pwm, uint8_t sector, uint32_t Tpwm, float Udc)
{
    Udc = Udc * 1.5f;
    float K = (float)Tpwm * SQRT_3 / Udc;

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
    if(T4 + T6 > (Tpwm * foc_pwm->K)){
        float ratio = (Tpwm * foc_pwm->K) / (T4 + T6);
        T4 *= ratio;
        T6 *= ratio;
    }

    // 计算各相作用时间
    Ta = (Tpwm - T4 - T6) / 4;
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

    // 保存各相时间
    foc_pwm->T_abc.ua = T1;
    foc_pwm->T_abc.ub = T2;
    foc_pwm->T_abc.uc = T3;

    // 死区补偿(根据实际硬件调整)
    float deadtime_comp = 0;
    T1 += deadtime_comp;
    T2 += deadtime_comp;
    T3 += deadtime_comp;

    // 计算占空比
    foc_pwm->_output.ua = (float)T1 / (Tpwm/2.0f) * LIN_V;
    foc_pwm->_output.ub = (float)T2 / (Tpwm/2.0f) * LIN_V;
    foc_pwm->_output.uc = (float)T3 / (Tpwm/2.0f) * LIN_V;
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
void Set_Svpwm(SVPWM *svpwm){
    double ua=constrain((svpwm->_output.ua),0.0f,LIN_V);
    double ub=constrain((svpwm->_output.ub),0.0f,LIN_V);
    double uc=constrain((svpwm->_output.uc),0.0f,LIN_V);

    // 电压转化为占空比
    float dc_a=constrain(ua/LIN_V,0.0f,1.0f);
    float dc_b=constrain(ub/LIN_V,0.0f,1.0f);
    float dc_c=constrain(uc/LIN_V,0.0f,1.0f);
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    // 设置PWM比较值
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_1, dc_a*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_2, dc_b*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_3, dc_c*ARR);
}

/**************************BSP层接口*********************************/

