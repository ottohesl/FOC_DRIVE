#include "Filter_Tool.h"
/**
 * @brief 常用滑动均值滤波
 * @param fit 滤波工具结构体
 * @note 使用该函数前必须定义raw_data、filter_size、fit_allow_max这三个值
 * @return 返回0和1，表示滤波失败或成功
 */
uint8_t FILTER_Sliding_Mean(FILTER_TOOL* fit) {
    if (fit->filter_size>FILTER_BUFF_SIZE||fit->fit_allow_max<0){return 0;}
    float fit_sum=0;
    if (fit->raw_data > fit->fit_allow_max||fit->raw_data < -fit->fit_allow_max) {
        fit->filtered_data = fit->fit_last_data;
        return 1;
    }
    fit->fit_buf[fit->fit_index]=fit->raw_data;
    fit->fit_index = (fit->fit_index+1) % fit->filter_size;
    //滑动滤波
    for (uint8_t i=0;i<fit->filter_size;i++) {
        fit_sum+=fit->fit_buf[i];
    }
    fit->filtered_data = fit_sum/(float)fit->filter_size;
    fit->fit_last_data = fit->filtered_data;
    return 1;
}
/**
 * @brief 一阶RC低通滤波
 * @param fit fit 滤波工具结构体
 * @return 返回0和1，表示滤波失败或成功
 */
uint8_t FILTER_RC(FILTER_TOOL* fit) {
    fit->filtered_data = fit->raw_data * FILTER_RC_alpha + fit->fit_last_data * (1-FILTER_RC_alpha);
    fit->fit_last_data = fit->filtered_data;
    return 1;
}

