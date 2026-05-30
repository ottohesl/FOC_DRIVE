#include "Main_Freertos.h"
#include "FOC_RUN.h"
#include "LCD_1.14.h"
void FOC_Task(void *argument)
{
    /* init code for USB_Device */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);//开启LED
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);//开启LED
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//开启LED
    //MX_USB_Device_Init();
    SPWM run;
    /* USER CODE BEGIN FOC_Task */
    /* Infinite loop */
    for(;;)
    {
        SPWM_RUN(&run,FOC_SPWM_OPEN_MODE);
        osDelay(10);
    }
    /* USER CODE END FOC_Task */
}
