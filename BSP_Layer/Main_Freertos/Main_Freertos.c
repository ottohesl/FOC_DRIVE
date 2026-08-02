#include "Main_Freertos.h"
#include "arm_math.h"
#include "foc_app.h"
#include "FOC_RUN.h"
#include "LCD_1.14.h"
#include "usb_device.h"
FOC_DRIVE MOTOR;       //定义电机，基于foc控制
TaskHandle_t FocTask_Control = NULL;
BaseType_t FocTask_Busy = pdFALSE;
/**
 * @brief FOC任务
 * @note 优先级：osPriorityHigh
 * @param argument 无
 */
void FOC_Task(void *argument)
{
    FocTask_Control = xTaskGetCurrentTaskHandle();
    /* init code for USB_Device */
    MX_USB_Device_Init();
    taskENTER_CRITICAL();
    FOC_Calc_Cur();
    taskEXIT_CRITICAL();
    /* End init code for USB_Device */
    FOC_SPWM_Init(&MOTOR.FOC_SPWM);         //初始化spwm
    FOC_SVPWM_Init(&MOTOR.FOC_SVPWM);       //初始化svpwm
    FOC_PID_Init(&MOTOR.FOC_PID);           //初始化pid参数
    FOC_ENC_DATA_Init(&MOTOR.FOC_ENC_DATA); //初始化编码器
    //Calibrate_Zero_Eangle(&Calc_Zero); //校准零偏纠正角
    /* USER CODE BEGIN FOC_Task */
    /* Infinite loop */
    float FOC_SVPWM_DATA[20] = {0};
    static uint8_t last_state = 0;
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        FocTask_Busy = pdTRUE;
        FOC_FB_Update(&MOTOR.FOC_CUR_PARAM);         //ADC获取电流电压
        FOC_ENC_Update(&MOTOR.FOC_ENC_DATA);         //获取编码器的速度与角度值
        FOC_MOTOR_RUN(&MOTOR,FOC_SPEED_MODE);
        if (MOTOR.FOC_PARAM.ON_OFF !=last_state) {
            if (MOTOR.FOC_PARAM.ON_OFF==1) {
                FOC_MOTOR_OPEN();
                last_state = MOTOR.FOC_PARAM.ON_OFF;
            }else {
                FOC_MOTOR_STOP();
                last_state = MOTOR.FOC_PARAM.ON_OFF;
            }
        }
        FOC_APP_Update(&MOTOR);
        LED_RUNNING(MOTOR.FOC_ENC_DATA.Enc_speed);
        FOC_SVPWM_DATA[0] = MOTOR.FOC_SVPWM.out_abc.ua;
        FOC_SVPWM_DATA[1] = MOTOR.FOC_SVPWM.out_abc.ub;
        FOC_SVPWM_DATA[2] = MOTOR.FOC_SVPWM.out_abc.uc;
        FOC_SVPWM_DATA[3] = MOTOR.FOC_SVPWM.elect_angle;
        FOC_SVPWM_DATA[4] = MOTOR.FOC_SVPWM.qd.uq;
        FOC_SVPWM_DATA[5] = MOTOR.FOC_SVPWM.qd.ud;
        FOC_SVPWM_DATA[6] = MOTOR.FOC_SVPWM.iqd.iq;
        FOC_SVPWM_DATA[7] = MOTOR.FOC_SVPWM.iqd.id;
        FOC_SVPWM_DATA[8] = MOTOR.FOC_ENC_DATA.Enc_speed;//实际速度
        FOC_SVPWM_DATA[9] = MOTOR.FOC_PARAM.iq;          //目标力矩
        FOC_SVPWM_DATA[10]= MOTOR.FOC_PARAM.speed;       //目标速度
        FOC_SVPWM_DATA[11]= MOTOR.FOC_PARAM.pos;         //目标位置
        FOC_SVPWM_DATA[12] = MOTOR.FOC_CUR_PARAM.Ia;
        FOC_SVPWM_DATA[13] = MOTOR.FOC_CUR_PARAM.Ib;
        FOC_SVPWM_DATA[14] = MOTOR.FOC_CUR_PARAM.Ic;
        FOC_SVPWM_DATA[15] = MOTOR.FOC_PID.spe_pid.increment;   //单次增量
        FOC_SVPWM_DATA[16] = MOTOR.FOC_PID.iq_pid.integral;     //位置式积分
        OTTO_USB_CDC_JustFloat(FOC_SVPWM_DATA,17);
        FocTask_Busy = pdFALSE;

    }
    /* USER CODE END FOC_Task */
}

/**
 * @brief 数据传输任务（串口、USB）
 * @note 优先级：osPriorityBelowNormal
 * @param argument 无
 */
void DATA_Task(void *argument) {
    float FOC_SPWM_DATA[10] = {0};
    float FOC_SVPWM_DATA[15] = {0};
    float FOC_ENC_DATA[10] = {0};
    float FOC_PID_DATA[10] = {0};
    float FOC_ADC_DATA[10] = {0};
    static uint8_t last_state = 0;
    /*****************************数据发送*********************************/
    for (;;) {

        osDelay(1);
    }
}
