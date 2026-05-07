#include "Main_Freertos.h"
#include "FOC_RUN.h"
#include "LCD_1.14.h"
#include "OLED_Menu.h"
#include "ottohesl.h"


void FOC_Task(void *argument)
{
    /* init code for USB_Device */
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

void OLED_Task(void *argument)
{
    /* USER CODE BEGIN OLED_Task */
    LCD_Clear(RED);
    LCD_Draw_Circle(120,60,60);
    /* Infinite loop */
    for(;;)
    {

        // double angle=AS5600_GetAngleDegrees(&i2c_AS5600 );
        // int lr=AS5600_Get_LR(&i2c_AS5600);
        // float turns=AS5600_Get_Turns(&i2c_AS5600);
        // float speed=AS5600_Get_Speed(&i2c_AS5600);
        // float speed_angle=speed * 360.0f;
        // OLED_Printf(0, 0*FONT_16, FONT_16,OLED_NORMAL, "RPS:%.2f",speed);
        // OLED_Printf(0, 1*FONT_16, FONT_16,OLED_NORMAL, "角度/s:%.2f",speed_angle);
        // OLED_Printf(0, 2*FONT_16, FONT_16,OLED_NORMAL, "角度:%.2f", angle);
        // OLED_Printf(0, 3*FONT_16, FONT_16,OLED_NORMAL, "圈数:%.2f",turns);
        // // OLED_Printf(0, 3*FONT_16, FONT_16,OLED_NORMAL, "方向:%d",lr);
        // OLED_ShowFrame();
        // osDelay(50);
    }
    /* USER CODE END OLED_Task */
}