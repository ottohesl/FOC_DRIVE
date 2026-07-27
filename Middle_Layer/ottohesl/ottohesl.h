#ifndef OTTOHESL_H
#define OTTOHESL_H
#define OTTOHESL_H_Vision 4
#include "stdint.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if (OTTOHESL_H_Vision==1)
#include "stm32f1xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==4)
#include "stm32G4xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==7)
#include "stm32h7xx_hal.h"
#endif



#define OTTOHESL_UART_BUFFER 256

void OTTO_Uart_FireWater(UART_HandleTypeDef *huart, const char *fmt, ...);
void OTTO_Uart_FireWater_DMA(UART_HandleTypeDef *huart, const char *fmt, ...);
void OTTO_Uart_JustFloat(UART_HandleTypeDef *huart, float *data, uint8_t ch_num);
void OTTO_Uart_JustFloat_DMA(UART_HandleTypeDef *huart, float *data, uint8_t ch_num);
void OTTO_USB_CDC_FireWater(const char *fmt, ...);
void OTTO_USB_CDC_JustFloat(float *data, uint8_t ch_num);
#endif //OTTOHESL_H
