#include "as5600.h"
#include<stdio.h>
#include <string.h>
#include <tgmath.h>
#include "i2c.h"
#include "Filter_Tool/Filter_Tool.h"
#define dirr_num 5//滤波大小
int full_ration=0;//圈数
float angle_points=0;//锚点
float speeds=0;
AS5600_Enc_DATA *g_Enc_ptr = NULL;
void FOC_ENC_DATA_Init(AS5600_Enc_DATA *enc) {
	g_Enc_ptr =  enc;
	memset(g_Enc_ptr, 0, sizeof(AS5600_Enc_DATA));
	g_Enc_ptr->Enc_calc_speed.tick = 0.001f;
}
/**
  * @brief  读取原始角度值（0-4095）
  * @param  hi2c: I2C句柄指针
  * @retval 角度值（0-4095）或AS5600_ERROR
  */
uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t angle_data[2];
    HAL_StatusTypeDef status;

    // 从0x0C寄存器开始连续读取2个字节
    status = HAL_I2C_Mem_Read(hi2c,
                             AS5600_I2C_ADDR << 1,
                             AS5600_RAW_ANG_H,
                             I2C_MEMADD_SIZE_8BIT,
                             angle_data,
                             2,100);  // 超时100ms

    if(status != HAL_OK) {
        return AS5600_ERROR;
    }

    // 组合高低字节数据
    return ((uint16_t)angle_data[0] << 8) | angle_data[1];
}
/**
  * @brief  读取原始角度值（0-4095）
  * @param  hi2c: I2C句柄指针
  * @retval 角度值（0-4095）或AS5600_ERROR
  */
void AS5600_StartReadRawAngle(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->State == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Read_DMA(hi2c,AS5600_I2C_ADDR << 1,AS5600_RAW_ANG_H,I2C_MEMADD_SIZE_8BIT,g_Enc_ptr->Enc_Raw_buf,2);
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &i2c_AS5600) {
		//dma传输完成触发回调，将数据存入缓冲区
		float enc_raw_angle = ((uint16_t)g_Enc_ptr->Enc_Raw_buf[0] << 8) | g_Enc_ptr->Enc_Raw_buf[1];
		g_Enc_ptr->Enc_angle = (enc_raw_angle * 360.0f) / 4096.0f;
		g_Enc_ptr->Enc_angle_buf[g_Enc_ptr->Enc_Index] = g_Enc_ptr->Enc_angle;	//将当前角度传入（0-360）
		g_Enc_ptr->Enc_Index = (g_Enc_ptr->Enc_Index + 1) % AS5600_ANGLE_BUFF;	//定义环形缓冲区
	}
}
/**
  * @brief  获取角度（0-360度）
  * @param  hi2c: I2C句柄指针
  * @retval 角度值（浮点型）或NAN
  */
float AS5600_GetAngleDegrees(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw_angle = AS5600_ReadRawAngle(hi2c);

    if(raw_angle == AS5600_ERROR) {
        return AS5600_DEG_ERROR;
    }

    // 转换为角度（4096对应360度）
    return (raw_angle * 360.0f) / 4096.0f;
}
//获取顺时针还是逆时针
int8_t AS5600_Get_LR(I2C_HandleTypeDef *hi2c){
	   static int8_t result = 0;
	    static int8_t  dirr[dirr_num] = {0};       // 滤波数组（初始化为0）
	    static float last_angle = NAN;         // 初始化为无效值
	    static int filter_idx = 0;             // 循环缓冲区索引
	    static int valid_samples = 0;          // 有效样本计数

	    float angle_point = AS5600_GetAngleDegrees(hi2c);

	    // 处理传感器读取错误
	    if (angle_point == AS5600_DEG_ERROR) return 0;

	    // 首次调用初始化
	    if (isnan(last_angle)) {
	        last_angle = angle_point;
	        return 0; // 首次无法判断方向
	    }

	    // 计算跨边界角度差（±180°范围内）
	    float d_angle = angle_point - last_angle;
	    if (d_angle > 180.0f) d_angle -= 360.0f;   // 顺时针跨边界（如350°→10°）
	    if (d_angle < -180.0f) d_angle += 360.0f;  // 逆时针跨边界（如10°→350°）

	    // 添加角度变化阈值（过滤噪声）
	    #define MIN_ANGLE_CHANGE 0.5f  // 最小有效角度变化（0.5°）
	    int flag = 0;
	    if (fabs(d_angle) > MIN_ANGLE_CHANGE) {
	        flag = (d_angle > 0) ? 1 : -1; // 顺时针/逆时针
	    }

	    // 更新滤波缓冲区（仅当有效方向变化时更新）
	    if (flag != 0) {
	        dirr[filter_idx] = flag;
	        filter_idx = (filter_idx + 1) % dirr_num; // 循环索引
	        valid_samples = (valid_samples < dirr_num) ? valid_samples + 1 : dirr_num;
	    }

	    // 当样本填满缓冲区时进行方向判断（多数表决）
	    if (valid_samples >= dirr_num) {
	        int8_t sum = 0;
	        for (int i = 0; i < dirr_num; i++) {
	            sum += dirr[i];
	        }
	        result = (sum > 0) ? 1 : (sum < 0 ? -1 : 0); // 正数=顺时针，负数=逆时针，0=无一致方向
	    } else {
	        result = 0; // 样本不足，不输出方向
	    }

	    last_angle = angle_point; // 更新上一次角度
	    return result;
}
//获取圈数与绝对累计角度
float AS5600_Get_Turns(I2C_HandleTypeDef *hi2c){
	float d_val=0.5f;//瞬间从360-》0的变化的倍数
	//int turns=AS5600_Get_LR(hi2c);//顺时针还是逆时针
	float angle=AS5600_GetAngleDegrees(hi2c);
	static int val=0;//圈数初始化
	static float point_angle=0;//锚点
	static float last_angle=0;
	//记录当前角度并将当前的角度作为零点
	static int loop=0;
	if(loop==0){
		point_angle=angle;//第一次获得的作为锚点
		loop=1;
	}
	angle_points=point_angle;//用于检测
	float actual_angle=((angle-point_angle)>0)? (angle-point_angle):(angle-point_angle+360);
	float d_angle=actual_angle-last_angle;
	//顺时针为正圈数计算，逆时针为负圈数计算
	if(fabs(d_angle)>d_val*360)val+=(d_angle>0)?-1:1;
	full_ration=val;
	last_angle=actual_angle;

	return val*360+actual_angle;
}
/**
 * @brief  AS5600机械转速计算（RPS），滑动平均滤波+过零+限幅
 * @param  mech_angle_deg  当前机械角度（度，0~360）
 * @param  dt              控制周期（秒）
 * @return float           机械转速 RPS
 */
float AS5600_CalcSpeed_MovAvg(float mech_angle_deg)
{
	// 静态持久化状态变量
	static float    last_angle     = 0.0f;    // 上一次机械角度（度）
	static uint32_t last_tick      = 0;       // 上一次调用的系统Tick（ms）
	static float    filtered_rps   = 0.0f;    // 滤波后输出转速
	static uint8_t  first_run      = 1;       // 首次调用标志

	// ========== 1. 首次调用：初始化状态，避免上电跳变冲击 ==========
	if (first_run)
	{
		last_angle = mech_angle_deg;
		last_tick  = HAL_GetTick();
		first_run  = 0;
		return 0.0f;
	}

	// ========== 2. 计算真实时间差dt，处理HAL_GetTick溢出回绕 ==========
	uint32_t now_tick = HAL_GetTick();
	float dt;

	if (now_tick >= last_tick)
	{
		// 正常无溢出：Tick差 / 1000 转换为秒
		dt = (float)(now_tick - last_tick) * 1e-3f;
	}
	else
	{
		// Tick溢出回绕：补全32位最大值的差值
		dt = (float)(UINT32_MAX - last_tick + now_tick + 1) * 1e-3f;
	}

	// ========== 3. dt合理性钳位（防止异常值炸转速、冲坏PID） ==========
	// 最小周期0.5ms（防止中断异常、重复调用），最大周期50ms（防止任务卡死）
	const float dt_min = 0.0005f;
	const float dt_max = 0.05f;
	if (dt < dt_min)  dt = dt_min;
	if (dt > dt_max)  dt = dt_max;

	// ========== 4. 角度差计算 + 0°/360°过零处理 ==========
	float delta_deg = mech_angle_deg - last_angle;

	// 角度差超过半圈（180°），判定为过零翻转，反向补偿360°
	if (delta_deg >  180.0f)  delta_deg -= 360.0f;
	if (delta_deg < -180.0f)  delta_deg += 360.0f;

	// ========== 5. 计算瞬时转速 RPS ==========
	float instant_rps = (delta_deg / 360.0f) / dt;

	// ========== 6. 一阶低通滤波（平滑转速，抑制编码器磁钢噪声） ==========
	const float filter_coeff = 0.1f;  // 0.05~0.2可调，越小越平滑、响应越慢
	filtered_rps = filtered_rps * (1.0f - filter_coeff) + instant_rps * filter_coeff;

	// ========== 7. 转速硬限幅（适配电机物理极限） ==========
	const float max_rps = 20.0f;  // 2804云台电机建议15~20，根据实际调整
	if (filtered_rps >  max_rps)  filtered_rps =  max_rps;
	if (filtered_rps < -max_rps)  filtered_rps = -max_rps;

	// ========== 8. 更新状态变量 ==========
	last_angle = mech_angle_deg;
	last_tick  = now_tick;

	return filtered_rps;
}
FILTER_TOOL Enc_speed={
	.fit_allow_max = 1000,		//最大转速超过无效
	.fit_index = 0,
	.raw_data = 0,
	.filter_size = 10,			//均值滤波大小
	.filtered_data = 0,
	.fit_last_data = 0,
	.fit_buf = {0}
};

/**
 * @brief 获取电机的速度
 * @param calc 编码器数据结构体
 * @return 当前速度（°/s）
 */
float AS5600_Get_Speed(AS5600_Enc_DATA *calc) {
	//获取最新缓冲区索引
	uint8_t calc_Index = g_Enc_ptr->tim_enc_data.tim_index;
	//将最新数据取出
	calc->Enc_calc_speed.cal_now_angle = calc->Enc_angle_buf[calc_Index];
	//与上一次得到的角度作差
	float calc_angle_deg = calc->Enc_calc_speed.cal_now_angle - calc->Enc_calc_speed.cal_last_angle;
	//角度归一化
	if (calc_angle_deg>180.0f)  calc_angle_deg -= 360.0f;
	if (calc_angle_deg<-180.0f)  calc_angle_deg += 360.0f;
	//依据单位时间得到速度（rps、rpm）
	calc->Enc_speed = calc_angle_deg / calc->Enc_calc_speed.tick / 360.0f;
	//这次角度赋给上一次角度
	calc->Enc_calc_speed.cal_last_angle = calc->Enc_calc_speed.cal_now_angle;
	//滤波
	Enc_speed.raw_data = calc->Enc_speed;
	FILTER_Sliding_Mean(&Enc_speed);
	Enc_speed.raw_data = Enc_speed.filtered_data;
	FILTER_RC(&Enc_speed);
	calc->Enc_speed = Enc_speed.filtered_data;
	return calc->Enc_speed;
}
void FOC_ENC_Update(AS5600_Enc_DATA *calc)
{
	AS5600_StartReadRawAngle(&i2c_AS5600);
	*calc = *g_Enc_ptr;
	if (calc->tim_enc_data.calc_flag) {
		AS5600_Get_Speed(calc);
		calc->tim_enc_data.calc_flag = 0;
	}
}




