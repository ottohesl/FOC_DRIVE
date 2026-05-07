#ifndef FOC_MATH_H
#define FOC_MATH_H
#include "stm32G4xx_hal.h"
/**************************基本变量定义*********************************/
#define FOC_TIM           &htim1
#define LIN_V  7.4f               // 母线电压
#define LIMIT_V 7.4f         // 电压限制
#define POLE_PAIRS 7         // 极对数
#define idr 1               // 方向
#define SQRT_3 1.73205080757f// 根号3
#define E_Deviation (187.0f * 360.0f / 4096.0f)
#define SQRT_3_DIV_2 0.86602540378f// 根号3/2
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef struct {
    double ua;  //三相电压A
    double ub;  //三相电压A
    double uc;  //三相电压A
} Phase_Voltage;
typedef struct {
    double uq;  //q轴电压
    double ud;  //d轴电压
}QD;
typedef struct {
    double alpha;
    double beta;
}alphabeta_t;
typedef  struct {
    Phase_Voltage  abc_v;
    QD qd;
    double elect_angle;
}SPWM;
typedef struct {
    QD qd;
    uint8_t sector;          // 当前扇区
    alphabeta_t ab;
    uint32_t Tpwm;
    Phase_Voltage T_abc;             // 各相时间
    Phase_Voltage abc_v;
    Phase_Voltage _output;           // 输出占空比
    float K;                 // 过调制系数
    // SVPWM计算中间变量
    float svpwm_val1;
    float svpwm_val2;
    double elect_angle;
}SVPWM;
void Init_SVPWM(SVPWM *svpwm) ;
void Calibrate_Zero_Eangle(SPWM *spwm) ;
void Get_PSC_ARR(TIM_HandleTypeDef *htim , uint32_t *psc ,uint32_t *arr);
double Limit_Angle(double angle);
double Solve_Electrical_Angle(double angle) ;
float Solve_PWM_Freq();
void Set_Spwm(SPWM *spwm);
void Set_Svpwm(SVPWM *svpwm);
void FOC_Spwm_Solve(SPWM *spwm) ;
uint8_t Sector_Judgment(SVPWM *svpwm, uint8_t *sector);
void VectorActionTime(SVPWM *foc_pwm, uint8_t sector, uint32_t Tpwm, float Udc);
void FOC_Svpwm_Solve(SVPWM *svpwm);
#endif //FOC_MATH_H
