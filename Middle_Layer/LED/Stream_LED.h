#ifndef STREAM_LED_H
#define STREAM_LED_H
#include "stm32g4xx_hal.h"
#define LED1_PIN GPIO_PIN_5
#define LED1_PORT GPIOC
#define LED2_PIN GPIO_PIN_3
#define LED2_PORT GPIOA
#define LED3_PIN GPIO_PIN_4
#define LED3_PORT GPIOA
#define LED1(n) HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, n==0?GPIO_PIN_RESET:GPIO_PIN_SET)
#define LED2(n) HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, n==0?GPIO_PIN_RESET:GPIO_PIN_SET)
#define LED3(n) HAL_GPIO_WritePin(LED3_PORT, LED3_PIN, n==0?GPIO_PIN_RESET:GPIO_PIN_SET)

#define LED_NUM 3       //可进行流水灯的总灯数
void LED_RUNNING(float rps);
void LED_ERROR(void);
#endif //STREAM_LED_H
