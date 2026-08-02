#ifndef FOC_MATH_H
#define FOC_MATH_H
#include "stm32G4xx_hal.h"
/**************************基本变量定义*********************************/
#define FOC_TIM           &htim1
#define LIMIT_V 7.4f         // 电压限制
#define POLE_PAIRS 7         // 极对数
#define idr 1                // 方向
#define SQRT_3 1.73205080757f// 根号3
#define E_Deviation(n) (n * 360.0f / 4096.0f)           //用于动态获取电角度offset偏差值
#define Read_E_Deviation (393 * 360.0f / 4096.0f)       //用于已知电角度偏移
#define SQRT_3_DIV_2 0.86602540378f // 根号3/2
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/**************************运算结构体定义*********************************/
typedef struct {
    uint32_t PSC;
    uint32_t ARR;
}TIM_DATA;
typedef struct {
    double ua;  //三相电压A
    double ub;  //三相电压B
    double uc;  //三相电压C
} Phase_Voltage;
typedef struct {
    double uq;  //q轴电压
    double ud;  //d轴电压
}QD_t;
typedef struct {
    double iq;  //q轴电压
    double id;  //d轴电压
}IQD_t;
typedef struct {
    double alpha;
    double beta;
}alphabeta_t;
typedef  struct {
    Phase_Voltage  vol_abc;   //三相电压
    QD_t qd;                  //q轴和d轴
    TIM_DATA tim_data;        //定时器基本数据
    double elect_angle;       //电角度
}SPWM;
typedef struct {
    QD_t qd;                    // 电压q轴和d轴
    IQD_t iqd;                  // 电流q轴和d轴
    uint8_t sector;             // 当前扇区
    alphabeta_t ab;             // 帕克逆变换后的alpha和beta轴
    uint32_t tim_pwm;           // pwm时间量控制
    Phase_Voltage tim_abc;      // 各相时间
    Phase_Voltage out_abc;      // 各相输出电压
    TIM_DATA tim_data;          // 定时器基本数据
    float tim_scale;            // 过调制系数
    float svpwm_val1;           // 中间计算量
    float svpwm_val2;           // 中间计算量
    double elect_angle;         // 电角度
}SVPWM;
/****************基础功能函数***************/
double Limit_Angle(double angle);
float Solve_PWM_Freq(TIM_DATA tim_data);
void Calibrate_Zero_angle(SPWM *spwm);
double Solve_Electrical_Angle(double angle);
/****************SPWM类函数***************/
void FOC_SPWM_Init(SPWM *spwm);
void FOC_Spwm_Solve(SPWM *spwm);
void FOC_Set_Spwm(SPWM *spwm,float bus_voltage);
/****************SVPWM类函数***************/
void FOC_SVPWM_Init(SVPWM *svpwm);
void FOC_Svpwm_Solve(SVPWM *svpwm);
void FOC_Set_Svpwm(SVPWM *svpwm,float bus_voltage);
uint8_t Sector_Judgment(SVPWM *svpwm, uint8_t *sector);
void VectorActionTime(SVPWM *foc_pwm, uint8_t sector,float Udc);
#endif //FOC_MATH_H
