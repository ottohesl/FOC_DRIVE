#ifndef __LCD_1_14_H
#define __LCD_1_14_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32G4xx_hal.h"
#include <stdint.h>

    /************************** LCD 硬件参数 **************************/
#define LCD_Width       240
#define LCD_Height      135

#define Width_Offset    40
#define Height_Offset   52

#define SPI            hspi2
    /************************** 引脚定义(自行修改) **************************/
#define LCD_RST_PIN     GPIO_PIN_6
#define LCD_RST_GPIO_PORT   GPIOB

#define LCD_DC_PIN      GPIO_PIN_14
#define LCD_DC_GPIO_PORT    GPIOB

#define LCD_BL_PIN     GPIO_PIN_5
#define LCD_BL_GPIO_PORT   GPIOA

#define LCD_RST(n)  HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, n?GPIO_PIN_SET:GPIO_PIN_RESET)
#define LCD_DC(n)   HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, n?GPIO_PIN_SET:GPIO_PIN_RESET)
#define LCD_BL(n)  HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_PIN, n?GPIO_PIN_SET:GPIO_PIN_RESET)

    /************************** 颜色定义 **************************/
#define WHITE           0xFFFF
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define MAGENTA         0xF81F
#define GREEN           0x07E0
#define CYAN            0x7FFF
#define YELLOW          0xFFE0
#define GRAY            0X8430

    /************************** 外部变量 **************************/
    extern uint16_t POINT_COLOR;
    extern uint16_t BACK_COLOR;

    /************************** 驱动函数 **************************/
    void LCD_Init(void);
    void LCD_Clear(uint16_t color);
    void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
    void LCD_Draw_Point(uint16_t x, uint16_t y);
    void LCD_Draw_ColorPoint(uint16_t x, uint16_t y, uint16_t color);
    void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void LCD_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r);
    void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void LCD_Write_HalfWord(const uint16_t da);
    void LCD_Show_Image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p);
    void LCD_TouchGFX_Flush(const uint16_t *frameBuffer);

#ifdef __cplusplus
}
#endif

#endif