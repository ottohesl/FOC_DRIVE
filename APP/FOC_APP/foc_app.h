#ifndef FOC_APP_H
#define FOC_APP_H
#include "Main_Freertos.h"
#include "stm32g4xx_hal.h"
#define APP_RAW_DATA_BUF  128
#define FRAME_LEN 8   //缓存帧长
#define PARA_HEAD 0xAA
#define PARA_TAIL 0x6B
typedef enum FOC_APP_PARAM_STATE {
    SEEK_HEAD = 0,
    SEEK_DATA = 1,
    SEEK_TAIL = 2,
}param_state;
typedef enum MOTOR_STATE {
    MOTOR_DISABLE = 0,      //失能电机
    MOTOR_ENABLE = 1,       //使能电机
}motor_state;
// CMD1 大类
typedef enum
{
    CMD_MOTOR_CTRL     = 0x01, //电机使能控制
    CMD_PID_TUNE       = 0x02, //PID调参
    CMD_FOC_TARGET     = 0x03, //FOC目标指令（力矩/速度/位置）
}CMD_MAIN_TYPE;
// 子指令第一层：环路选择
typedef enum{
    PID_CUR_Q    = 0x01, //电流环Q轴
    PID_CUR_D    = 0x02, //电流环D轴
    PID_SPEED    = 0x03, //速度环
    PID_POSIT    = 0x04, //位置环
}PID_LOOP_TYPE;
// 子指令第二层：参数类型
typedef enum {
    PID_KP = 0X01,
    PID_KI = 0x02,
    PID_KD = 0x03,
}PID_PARAM_TYPE;
typedef enum {
    TARGET_CUR  = 0x04,  //目标力矩(Iq)
    TARGET_SPE  = 0x05,  //目标速度
    TARGET_POS  = 0x06,  //目标位置
}FOC_TARGET_CMD;
typedef struct{
    float kp;
    float ki;
    float kd;
}Foc_pid_control_param;
typedef struct {
    float speed;
    float position;
    float iq;
    float id;
    uint8_t motor_state;     //0表示关闭、1表示使能
}Foc_Target_Control;        //foc参数输入（使用串口、按键）
typedef struct {
    Foc_pid_control_param pid_spe;      //速度环
    Foc_pid_control_param pid_cur_q;    //q轴电流环
    Foc_pid_control_param pid_cur_d;    //d轴电流环
    Foc_pid_control_param pid_pos;      //位置环
    Foc_Target_Control foc_param;       //电机控制输入
}Foc_App_Param;
typedef struct {
    volatile uint8_t USB_DATAS[APP_RAW_DATA_BUF];
    volatile uint8_t data_len;
}USB_DATA;
void foc_app_find_data();
void FOC_APP_Update(FOC_DRIVE *MOTOR);
extern USB_DATA FOC_USB_DATA;
#endif //FOC_APP_H
