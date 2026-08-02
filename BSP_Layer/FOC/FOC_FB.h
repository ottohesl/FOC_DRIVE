#ifndef FOC_FB_H
#define FOC_FB_H
#include "stm32G4xx_hal.h"
#include "adc.h"
/********************ADC总参数设置**************************/
#define ADC1_TOTAL_CH    4   //总共通道数
#define VBUS_FILTER_LEN  16
#define ADC_VREF        3.3f
#define ADC_12BIT_MAX   4095.0f
#define CH_A_CUR 0
#define CH_B_CUR 1
#define CH_C_CUR 2
#define CH_VBUS  3     //电压采集通道
#define CH_ABC   3     //三相通道数
/********************ADC_IN8电压参数**************************/
#define ERROR_DIVER     1.0338f
#define R_UP_TOTAL     (100.0f + 100000.0f)
#define R_DOWN         22000.0f
#define VOLT_SCALE     (((R_UP_TOTAL + R_DOWN) / R_DOWN) * ERROR_DIVER) // 5.55
#define ADC_VREF       3.3f
#define ADC_12BIT_MAX  4096.0f
/********************DRV8323电流检测参数**************************/
#define DRV_VREF        3.27f        // 内部基准电压
#define DRV_GAIN        20.0f       // GAIN接DVDD，40倍增益
#define R_SHUNT         0.002f       // 替换你实际采样电阻，例10mΩ填0.01Ω
#define VREF_HALF       (DRV_VREF / 2.0f) // 1.65V零电流中点
/********************ADC滑动均值滤波**************************/
#define  FILTER_BUFF    20         //滤波均值大小
#define  FILTER_CALC    200        //校准缓存最大数量
enum FILTER_MODE{
    NOMINAL_MODE = 0,     //普通滑动均值滤波模式
    CALIBRA_MODE = 1,     //初始化校准模式calibration
};  //滤波模式
typedef struct {
    float Ia;               //a相电流
    float Ib;               //b相电流
    float Ic;               //c相电流
    float Bus_Voltage;          //母线电压
}FOC_FB;
void FOC_Calc_Cur();
void ADC1_DMA_InitStart();
float Get_Bus_Voltage();
void Get_CUR_ABC(float *CUR);
void Get_Phase_Sequence(float *Ia, float *Ib, float *Ic);
uint8_t CUR_filter(const float *CUR, float *Filter_Result,enum FILTER_MODE mode);
void FOC_FB_Update(FOC_FB *fb);
extern uint8_t adc_data_ready;
extern  uint16_t adc_dma_buf[ADC1_TOTAL_CH];
#endif //FOC_FB_H
