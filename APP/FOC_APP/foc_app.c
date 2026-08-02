#include "foc_app.h"
#include <string.h>
#include "ottohesl.h"
uint8_t APP_RAW_DATA[APP_RAW_DATA_BUF]={0};
static uint8_t APP_RAW_DATA_8[8]={0};
Foc_App_Param APP_PARAM = {
    .pid_cur_q.kp = cur_kp,
    .pid_cur_q.ki = cur_ki,
    .pid_cur_d.kp = cur_kp,
    .pid_cur_d.ki = cur_ki,
    .pid_pos.kp = pos_kp,
    .pid_pos.ki = pos_ki,
    .pid_pos.kd = pos_kd,
    .pid_spe.kp = spe_kp,
    .pid_spe.ki = spe_ki,
    .pid_spe.kd = spe_kd,
};
USB_DATA FOC_USB_DATA;
typedef union{
    uint8_t bytes[4];
    float fval;
}float_union_t;

void protocol_frame_handle(uint8_t *frame)
{
    uint8_t cmd1 = frame[1];
    uint8_t cmd2 = frame[2];
    float_union_t data_union;
    //拷贝4字节小端浮点
    memcpy(data_union.bytes, &frame[3], 4);
    float val = data_union.fval;

    switch(cmd1)
    {
        case CMD_MOTOR_CTRL: //0x01 电机使能/失能
        {
            if(cmd2 == MOTOR_ENABLE)
            {
                APP_PARAM.foc_param.motor_state = 1;
            }
            else if(cmd2 == MOTOR_DISABLE)
            {
                APP_PARAM.foc_param.motor_state = 0;
            }
            break;
        }
        case CMD_PID_TUNE: //0x02 PID调参
        {
            uint8_t loop  = (cmd2 >> 4) & 0x0F;
            uint8_t param = cmd2 & 0x0F;
            switch(loop)
            {
                case PID_CUR_Q: //CUR_Q
                    if(param == PID_KP) APP_PARAM.pid_cur_q.kp =val;
                    if(param == PID_KI) APP_PARAM.pid_cur_q.ki =val;
                    break;
                case PID_CUR_D: //CUR_D
                    if(param == PID_KP) APP_PARAM.pid_cur_d.kp =val;
                    if(param == PID_KI) APP_PARAM.pid_cur_d.ki =val;
                    break;
                case PID_SPEED: //SPEED环
                    if(param == PID_KP) APP_PARAM.pid_spe.kp = val;
                    if(param == PID_KI) APP_PARAM.pid_spe.ki = val;
                    if(param == PID_KD) APP_PARAM.pid_spe.kd = val;
                    break;
                case PID_POSIT: //位置环
                    if(param == PID_KP) APP_PARAM.pid_pos.kp = val;
                    if(param == PID_KI) APP_PARAM.pid_pos.ki = val;
                    if(param == PID_KD) APP_PARAM.pid_pos.kd = val;
                    break;
            }
            break;
        }
        case CMD_FOC_TARGET: //0x03 目标指令
        {
            switch(cmd2)
            {
                case TARGET_CUR: //目标力矩Iq
                    APP_PARAM.foc_param.iq = val;
                    break;
                case TARGET_SPE: //目标速度
                    APP_PARAM.foc_param.speed = val;
                    break;
                case TARGET_POS: //目标位置
                    APP_PARAM.foc_param.position = val;
                    break;
            }
            break;
        }
        default: break;
    }
}

volatile static param_state rx_state = SEEK_HEAD;
volatile uint8_t rx_frame_buf[8];
volatile uint8_t rx_idx = 0;

void protocol_byte_input(uint8_t byte)
{
   switch(rx_state)
   {
      case SEEK_HEAD:
         if(byte == PARA_HEAD)
         {
            rx_frame_buf[0] = byte;
            rx_idx = 1;
            rx_state = SEEK_DATA;
         }
         break;
      case SEEK_DATA:
         rx_frame_buf[rx_idx++] = byte;
         if(rx_idx >= FRAME_LEN - 1) //收到第7字节，等待最后一个帧尾
         {
            rx_state = SEEK_TAIL;
         }
         break;
      case SEEK_TAIL:
         if(byte == PARA_TAIL)
         {
            rx_frame_buf[7] = byte;
            protocol_frame_handle(rx_frame_buf);
         }
         //无论校验成功失败，立刻重新搜索帧头
         rx_state = SEEK_HEAD;
         rx_idx = 0;
         break;
      default:
         rx_state = SEEK_HEAD;
         rx_idx = 0;
         break;
   }
}
float DATA[14] = {0};
//app任务链:
//获取usb、串口数据----使用定时器/data_task轮询获取
void foc_app_find_data() {
   //从缓冲区里面寻找帧头帧尾
   for(uint8_t i=0;i<FOC_USB_DATA.data_len;i++) {
       protocol_byte_input(FOC_USB_DATA.USB_DATAS[i]);
   }
    // DATA[0]=APP_PARAM.pid_cur_q.kp;
    // DATA[1]=APP_PARAM.pid_cur_q.ki;
    // DATA[2]=APP_PARAM.pid_cur_d.kp;
    // DATA[3]=APP_PARAM.pid_cur_d.ki;
    // DATA[4]=APP_PARAM.pid_spe.kp;
    // DATA[5]=APP_PARAM.pid_spe.ki;
    // DATA[6]=APP_PARAM.pid_spe.kd;
    // DATA[7]=APP_PARAM.pid_pos.kp;
    // DATA[8]=APP_PARAM.pid_pos.ki;
    // DATA[9]=APP_PARAM.pid_pos.kd;
    // DATA[10]=APP_PARAM.foc_param.iq;
    // DATA[11]=APP_PARAM.foc_param.speed;
    // DATA[12]=APP_PARAM.foc_param.position;
    // DATA[13]=APP_PARAM.foc_param.motor_state;
    //OTTO_USB_CDC_JustFloat(DATA,14);
    FOC_USB_DATA.data_len = 0;
}

/**
 * @brief 将app层的要求参数写入底层motor类
 * @param MOTOR FOC驱动底层结构体
 */
void FOC_APP_Update(FOC_DRIVE *MOTOR) {
    foc_app_find_data();
    MOTOR->FOC_PID.iq_pid.kp = APP_PARAM.pid_cur_q.kp;
    MOTOR->FOC_PID.iq_pid.ki = APP_PARAM.pid_cur_q.ki;
    MOTOR->FOC_PID.id_pid.kp = APP_PARAM.pid_cur_d.kp;
    MOTOR->FOC_PID.id_pid.ki = APP_PARAM.pid_cur_d.ki;
    MOTOR->FOC_PID.spe_pid.kp = APP_PARAM.pid_spe.kp;
    MOTOR->FOC_PID.spe_pid.ki = APP_PARAM.pid_spe.ki;
    MOTOR->FOC_PID.spe_pid.kd = APP_PARAM.pid_spe.kd;
    MOTOR->FOC_PID.pos_pid.kp = APP_PARAM.pid_pos.kp;
    MOTOR->FOC_PID.pos_pid.ki = APP_PARAM.pid_pos.ki;
    MOTOR->FOC_PID.pos_pid.kd = APP_PARAM.pid_pos.kd;
    //MOTOR->FOC_PARAM.iq = APP_PARAM.foc_param.iq;
    MOTOR->FOC_PARAM.speed = APP_PARAM.foc_param.speed;
    MOTOR->FOC_PARAM.pos = APP_PARAM.foc_param.position;
    MOTOR->FOC_PARAM.ON_OFF = APP_PARAM.foc_param.motor_state;
}
//根据帧头帧尾定义，分类命令再将命令分发-------数据帧状态机解析帧类型
//解析数据帧数据------------从缓冲区读取数据并解析
//数据输入给foc_task
