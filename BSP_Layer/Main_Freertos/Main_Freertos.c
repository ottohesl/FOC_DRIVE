#include "Main_Freertos.h"
#include "FOC_RUN.h"
#include "LCD_1.14.h"
#include "usb_device.h"
void FOC_Task(void *argument)
{
    /* init code for USB_Device */
    Calc_Cur();                        //初始化校准电流
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);//开启LED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);//开启LED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//开启LED
    MX_USB_Device_Init();

    SPWM Calc_Zero;
    SVPWM run1;
    Init_SVPWM(&run1);
    //Calibrate_Zero_Eangle(&Calc_Zero); //校准零偏纠正角
    /* USER CODE BEGIN FOC_Task */
    /* Infinite loop */
    for(;;)
    {
        if (foc_control_time) {
            foc_control_time = 0;
        SVPWM_RUN(&run1,FOC_SVPWM_SPEED_MODE);
        }
        //SVPWM_RUN(&run1,FOC_SVPWM_OPEN_MODE);
         osDelay(1);
    }
    /* USER CODE END FOC_Task */
}
void DATA_Task(void *argument) {
    SVPWM sv_uart;
    /*****************************数据发送*********************************/
    for (;;) {
        // if (osMessageQueueGet(VOFAHandle,&sv_uart,0,100)==osOK) {
        //     OTTO_usb_cdc("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        //         sv_uart._output.ua+LIN_V/2,sv_uart._output.ub+LIN_V/2,sv_uart._output.uc+LIN_V/2,
        //         sv_uart.qd.uq,sv_uart.qd.ud,sv_uart.elect_angle);
        // }

        //
        //
         //Get_CUR_ABC(RAW_ABC);
        //CUR_filter(RAW_ABC,ABC_Phase,NOMINAL_MODE);
        OTTO_usb_cdc("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                     vofa_data_run.Udata.A,vofa_data_run.Udata.B,vofa_data_run.Udata.C,vofa_data_run.elect_angle,
                     vofa_data_run.now_speed,vofa_data_run.targe_speed,
                     vofa_data_run.uq,vofa_data_run.ud,vofa_data_run.iq,vofa_data_run.id,
                     vofa_data_run.Idata.A,vofa_data_run.Idata.B,vofa_data_run.Idata.C);
        osDelay(1);
    }
}
