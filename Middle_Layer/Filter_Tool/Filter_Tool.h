#ifndef FILTER_TOOL_H
#define FILTER_TOOL_H
#include "stm32g4xx_hal.h"
#define FILTER_BUFF_SIZE 30
#define FILTER_RC_alpha  0.8   //一阶RC低通滤波系数，越大越接近当前值
typedef struct {
    float raw_data;             //原始数据输入
    float filtered_data;        //滤波结果输出(可用其他变量接受)
    float fit_allow_max;        //允许原始数据的输入的最大值（防止高频噪声污染）
    float fit_last_data;        //滤波上次有效值
    uint8_t fit_index;          //滤波缓冲索引
    uint8_t filter_size;        //滤波缓存大小（数值越大可靠性更高）
}FILTER_TOOL;
uint8_t FILTER_Sliding_Mean(FILTER_TOOL* fit);
uint8_t FILTER_RC(FILTER_TOOL* fit);
#endif //FILTER_TOOL_H
