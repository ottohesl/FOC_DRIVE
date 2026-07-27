#include "FOC_FB.h"
#include "FOC_RUN.h"
#include "ottohesl.h"
#include "Filter_Tool/Filter_Tool.h"
uint16_t adc_dma_buf[ADC1_TOTAL_CH] = {0};
uint16_t vbus_filter[VBUS_FILTER_LEN] = {0};
uint8_t filter_ptr = 0;
float  adc_filter_buf[CH_ABC][FILTER_BUFF] = {0};
float  adc_filter_result[CH_ABC] = {0};  //获取电机静止状态下偏移值
// 全局标志位
uint8_t adc_data_ready = 0;
void ADC1_DMA_InitStart()
{
    // 循环DMA搬运全部通道采样值
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC1_TOTAL_CH);
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);
}
/**
 * @brief 三相电流专属滑动均值滤波函数
 * @param CUR 当前三相电流值
 * @param Filter_Result 滑动均值结果
 * @param mode 分为普通滑动均值模式和初始化均值偏移量
 * @return 1表示获取成功
 */
uint8_t CUR_filter(const float *CUR, float *Filter_Result,enum FILTER_MODE mode) {
    switch (mode) {
        case NOMINAL_MODE:
            static uint8_t Index = 0;
            float filter_sum[CH_ABC] = {0};
            static float last_filter_sum[CH_ABC] = {0};
            //检测到尖刺电流直接输出上一次合法的
            if ((CUR[CH_A_CUR]>10||CUR[CH_A_CUR]<-10)||
                (CUR[CH_B_CUR]>10||CUR[CH_B_CUR]<-10)||
                (CUR[CH_C_CUR]>10||CUR[CH_C_CUR]<-10)) {
                adc_filter_buf[CH_A_CUR][Index]=last_filter_sum[CH_A_CUR];
                adc_filter_buf[CH_B_CUR][Index]=last_filter_sum[CH_B_CUR];
                adc_filter_buf[CH_C_CUR][Index]=last_filter_sum[CH_C_CUR];
                }else {
                    adc_filter_buf[CH_A_CUR][Index] = CUR[CH_A_CUR];
                    adc_filter_buf[CH_B_CUR][Index] = CUR[CH_B_CUR];
                    adc_filter_buf[CH_C_CUR][Index] = CUR[CH_C_CUR];
                }
            Index = (Index+1)%FILTER_BUFF;
            for(uint8_t i=0; i<CH_ABC; i++) {
                for (uint8_t j = 0; j<FILTER_BUFF; j++) {
                    filter_sum[i] += adc_filter_buf[i][j];
                }
            }
            for(uint8_t i=0; i<CH_ABC; i++) {
                Filter_Result[i] = filter_sum[i] / FILTER_BUFF;
            }
            last_filter_sum[CH_A_CUR] = Filter_Result[CH_A_CUR];
            last_filter_sum[CH_B_CUR] = Filter_Result[CH_B_CUR];
            last_filter_sum[CH_C_CUR] = Filter_Result[CH_C_CUR];
            return 1;
            break;
        case CALIBRA_MODE:
            static uint8_t Calc_Index = 0;
            static float filter_calc[CH_ABC][FILTER_CALC] = {0};
            static float filter_calc_sum[CH_ABC] = {0};
            filter_calc[CH_A_CUR][Calc_Index] = CUR[CH_A_CUR];
            filter_calc[CH_B_CUR][Calc_Index] = CUR[CH_B_CUR];
            filter_calc[CH_C_CUR][Calc_Index] = CUR[CH_C_CUR];
            Calc_Index++;
            if(Calc_Index >= FILTER_CALC) {
                for(uint8_t i=0; i<CH_ABC; i++) {
                    for (uint8_t j = 0; j<FILTER_CALC; j++) {
                        filter_calc_sum[i] += filter_calc[i][j];
                    }
                }
                for(uint8_t i=0; i<CH_ABC; i++) {
                    Filter_Result[i] = filter_calc_sum[i] / FILTER_CALC;
                }
                return 1;//最后得到偏移值，状态置1
            }
            return 0;
            break;
        default:
            return 0;
            break;
    }
}
FILTER_TOOL fit_Bus = {
    .raw_data = 0,
    .filtered_data = 0,
    .fit_last_data = 0,
    .filter_size = FILTER_BUFF,
    .fit_allow_max = 24.0f,
    .fit_index = 0
};
float Get_Bus_Voltage()
{
    // 读取DMA缓存中母线通道原始值
    uint16_t raw = adc_dma_buf[CH_VBUS];
    // 换算引脚电压 → 真实母线电压
    float v_pin = (float)raw * ADC_VREF / ADC_12BIT_MAX;
    float v_bus = v_pin * VOLT_SCALE;
    fit_Bus.raw_data = v_bus;
    FILTER_Sliding_Mean(&fit_Bus);           //滑动均值滤波
    return fit_Bus.filtered_data;
}
static float Get_CUR(uint8_t Index) {
    // 1.ADC原始值转SOx引脚电压
    uint16_t adc_raw = adc_dma_buf[Index];
    float v_soa = (float)adc_raw * ADC_VREF / ADC_12BIT_MAX;
    // 2.套用DRV8323双向电流公式
    float current = (v_soa - VREF_HALF) / (DRV_GAIN * R_SHUNT);
    return current;
}

/**
 * @brief 获取三相电流值
 * @param CUR 传入的电流结果数组
 */
void Get_CUR_ABC(float *CUR) {
    for (uint8_t i=0; i<CH_ABC; i++) {
        CUR[i]=Get_CUR(i)-adc_filter_result[i];//顺序为：丝印w、v、u；soa、sob、soc
    }
}

/**************************功能函数*************************/
/**
 *@brief 初始化获取三相电流的偏移值
 *@return 返回是否得到偏移值的结果
 */
void FOC_Calc_Cur()
{
    float Calc[CH_ABC] = {0};
    //静止电机下，进行读取adc以校准
    do {
        Get_CUR_ABC(Calc);
    }while (CUR_filter(Calc,adc_filter_result,CALIBRA_MODE)!=1);
}

/**
 * @brief 依据相序舍弃短相
 */
void Get_Phase_Sequence(float *Ia, float *Ib, float *Ic,uint8_t N) {
    static float last_Ia=0,last_Ib=0,last_Ic=0;
    if (Ia==NULL||Ib==NULL||Ic==NULL) return;
    float RAW_ABC[CH_ABC] = {0};
    float ABC_Phase[CH_ABC] = {0};
    float ABC_Phase_More[CH_ABC] = {0};
    Get_CUR_ABC(RAW_ABC);
    CUR_filter(RAW_ABC,ABC_Phase,NOMINAL_MODE);
    CUR_filter(ABC_Phase,ABC_Phase_More,NOMINAL_MODE);
    switch (N) {
    //     case 0: //扇区1
    //     case 5: //扇区6
    //         *Ib = ABC_Phase[1];
    //         *Ic = ABC_Phase[2];
    //         *Ia = -(*Ib+*Ic);
    //         break;
    //     case 1:
    //     case 2:
    //         *Ia = ABC_Phase[0];
    //         *Ic = ABC_Phase[2];
    //         *Ia = -(*Ia+*Ic);
    //         break;
    //     case 3:
    //     case 4:
    //         *Ia = ABC_Phase[0];
    //         *Ib = ABC_Phase[1];
    //         *Ic = -(*Ia+*Ib);
    //         break;
        default:
            *Ia = ABC_Phase_More[0];
            *Ib = ABC_Phase_More[1];
            *Ic = ABC_Phase_More[2];
    }
    if (*Ia>10||*Ib>10||*Ic>10||*Ia<-10||*Ib<-10||*Ic<-10) {
        *Ia = last_Ia;
        *Ib = last_Ib;
        *Ic = last_Ic;
    }
    last_Ia = *Ia;
    last_Ib = *Ib;
    last_Ic = *Ic;
}
void FOC_FB_Update(FOC_FB *fb) {
    fb->Bus_Voltage=Get_Bus_Voltage();
    Get_Phase_Sequence(&fb->Ia,&fb->Ib,&fb->Ic,0);
}

