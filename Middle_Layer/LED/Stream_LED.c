#include "Stream_LED.h"

/**
 * @brief 运动流水灯：转速越快，流水速度越快
 * @param rps 电机转速 (转/秒 RPS)
 * @note LED_NUM=3，LED1/LED2/LED3依次点亮
 */
void LED_RUNNING(float rps)
{
    uint32_t now_tick = HAL_GetTick();
    static uint32_t last_tick = 0;
    static uint8_t LED_index = 0;

    float HZ = rps * 0.5f;
    if(HZ == 0.0f)
    {
        LED1(1);
        LED2(1);
        LED3(1);
        return;
    }
    float total_period_ms = 1000.0f / HZ;    //完整一轮流水总时长
    float seg_time = total_period_ms / LED_NUM; //单颗LED持续时长

    uint32_t tick_diff = now_tick - last_tick;

    if(tick_diff < seg_time)
    {
        // 当前区段未超时：维持当前LED
        switch (LED_index)
        {
            case 0: LED1(1); LED2(0); LED3(0); break;
            case 1: LED1(0); LED2(1); LED3(0); break;
            case 2: LED1(0); LED2(0); LED3(1); break;
        }
    }
    else
    {
        // 区段超时：切换下一个LED
        last_tick += (uint32_t)seg_time;
        LED_index = (LED_index + 1) % LED_NUM;
    }
}

/**
 * @brief 故障灯：三颗LED同步500ms闪烁
 */
void LED_ERROR(void)
{
    uint32_t now_tick = HAL_GetTick();
    static uint32_t last_tick = 0;
    static uint8_t led_state = 0;

    if(now_tick - last_tick >= 500)
    {
        last_tick = now_tick;
        led_state = !led_state;
    }

    if(led_state)
    {
        LED1(1); LED2(1); LED3(1);
    }
    else
    {
        LED1(0); LED2(0); LED3(0);
    }
}
