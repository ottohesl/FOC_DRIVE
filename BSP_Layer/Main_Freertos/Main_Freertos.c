#include "Main_Freertos.h"

#include "arm_math.h"
#include "FOC_RUN.h"
#include "LCD_1.14.h"
#include "usb_device.h"
FOC_DRIVE MOTOR;       //定义电机，基于foc控制
TaskHandle_t FocTask_Control = NULL;
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
    /* End init code for USB_Device */
    FOC_Calc_Cur();                         //初始化校准电流
    FOC_SPWM_Init(&MOTOR.FOC_SPWM);         //初始化spwm
    FOC_SVPWM_Init(&MOTOR.FOC_SVPWM);       //初始化svpwm
    FOC_PID_Init(&MOTOR.FOC_PID);           //初始化pid参数
    //Calibrate_Zero_Eangle(&Calc_Zero); //校准零偏纠正角
    /* USER CODE BEGIN FOC_Task */
    /* Infinite loop */
    for(;;)
    {
        if (foc_control_time) {
            FOC_FB_Update(&MOTOR.FOC_CUR_PARAM);     //ADC获取电流电压
            FOC_ENC_Update(&MOTOR.FOC_ENC_DATA);     //获取编码器的速度与角度值
            FOC_MOTOR_RUN(&MOTOR,FOC_SPEED_MODE);
            foc_control_time = 0;
        }
        //MOTOR.notify_overfull=ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  //等待TIM2的foc中断任务启用通知，未收到跳转其他任务
        osDelay(1);
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
    float FOC_SVPWM_DATA[10] = {0};
    float FOC_ENC_DATA[10] = {0};
    float FOC_PID_DATA[10] = {0};
    float FOC_ADC_DATA[10] = {0};
    /*****************************数据发送*********************************/
    for (;;) {
        FOC_SVPWM_DATA[0] = MOTOR.FOC_SVPWM.elect_angle;
        FOC_SVPWM_DATA[1] = MOTOR.FOC_SVPWM.qd.uq;
        FOC_SVPWM_DATA[2] = MOTOR.FOC_SVPWM.qd.ud;
        FOC_SVPWM_DATA[3] = MOTOR.FOC_SVPWM.out_abc.ua;
        FOC_SVPWM_DATA[4] = MOTOR.FOC_SVPWM.out_abc.ub;
        FOC_SVPWM_DATA[5] = MOTOR.FOC_SVPWM.out_abc.uc;
        FOC_SVPWM_DATA[6] = MOTOR.FOC_SVPWM.iqd.iq;
        FOC_SVPWM_DATA[7] = MOTOR.FOC_SVPWM.iqd.id;
        OTTO_USB_CDC_JustFloat(FOC_SVPWM_DATA,8);



        // OTTO_usb_cdc("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        //              vofa_data_run.Udata.A,vofa_data_run.Udata.B,vofa_data_run.Udata.C,vofa_data_run.elect_angle,
        //              vofa_data_run.now_speed,vofa_data_run.targe_speed,
        //              vofa_data_run.uq,vofa_data_run.ud,vofa_data_run.iq,vofa_data_run.id,
        //              vofa_data_run.Idata.A,vofa_data_run.Idata.B,vofa_data_run.Idata.C);
        osDelay(1);
    }
}
