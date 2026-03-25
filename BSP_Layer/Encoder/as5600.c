#include "as5600.h"
#include<stdio.h>
#define dirr_num 5//滤波大小
int full_ration=0;//圈数
float angle_points=0;//锚点
float speeds=0;
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
                             2,
                             100);  // 超时100ms

    if(status != HAL_OK) {
        return AS5600_ERROR;
    }

    // 组合高低字节数据
    return ((uint16_t)angle_data[0] << 8) | angle_data[1];
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
//获取角速度
float AS5600_Get_Speed(I2C_HandleTypeDef *hi2c){
	    // 获取当前角度（含圈数）
	    float now_angle = AS5600_Get_Turns(hi2c);
		float Ts = 0.01f;
	    // 静态变量保持用户指定的变量名
	    static float last_angle = 0;       // 上一次角度（含圈数）
	    static uint32_t last_now = 0;         // 上一次时间（ms）
	    static uint8_t first_call = 1;     // 首次调用标志

	    // 处理首次调用：初始化并返回0
	    if (first_call) {
	        last_angle = now_angle;
	        last_now = HAL_GetTick();
	        first_call = 0;
	        return 0; // 首次无速度
	    }

	    // 计算时间间隔（处理HAL_GetTick()溢出）
	    uint32_t now = HAL_GetTick();
	    float dt = 0;
	    if (now >= last_now) {
	        dt = (now - last_now) * 1e-3f; // 正常时间差
	    } else {
	        dt = (UINT32_MAX - last_now + now + 1) * 1e-3f; // 溢出处理
	    }
	    // 限制时间间隔在合理范围
	    if (dt < 0.001f || dt > Ts * 10) dt = 8*Ts; // 最小1ms.dt大概为0.088s

	    // 计算速度（角度/秒）
	    float speed = (now_angle - last_angle) / dt;
	    float RPS = speed / 360.0f; // 转换为RPS

	    // 更新状态变量（用户指定的变量名）
	    last_angle = now_angle;
	    last_now = now;
	    speeds = RPS; // 保存全局速度变量（若有需要）

	    return RPS;
}

