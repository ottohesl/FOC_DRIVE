#include "lcd_1.14.h"
#include "spi.h"

#define LCD_TOTAL_BUF_SIZE   (240*135*2)
#define LCD_Buf_Size         648

static uint8_t lcd_buf[LCD_Buf_Size];
uint16_t POINT_COLOR = WHITE;
uint16_t BACK_COLOR  = BLACK;

// ==============================
// 核心 SPI2 发送
// ==============================
static void LCD_SPI_Send(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(&SPI, data, size, 100);
}

static void LCD_Write_Cmd(uint8_t cmd)
{
    LCD_DC(0);
    LCD_SPI_Send(&cmd, 1);
}

static void LCD_Write_Data(uint8_t data)
{
    LCD_DC(1);
    LCD_SPI_Send(&data, 1);
}


void LCD_Write_HalfWord(const uint16_t da)
{
    uint8_t data[2];
    data[0] = da >> 8;
    data[1] = da;
    LCD_DC(1);
    LCD_SPI_Send(data, 2);
}

// ==============================
// 窗口设置
// ==============================
void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Write_Cmd(0x2A);
    LCD_Write_Data((Width_Offset + x1) >> 8);
    LCD_Write_Data((Width_Offset + x1));
    LCD_Write_Data((Width_Offset + x2) >> 8);
    LCD_Write_Data((Width_Offset + x2));

    LCD_Write_Cmd(0x2B);
    LCD_Write_Data((Height_Offset + y1) >> 8);
    LCD_Write_Data((Height_Offset + y1));
    LCD_Write_Data((Height_Offset + y2) >> 8);
    LCD_Write_Data((Height_Offset + y2));

    LCD_Write_Cmd(0x2C);
}

// ==============================
// 清屏
// ==============================
void LCD_Clear(uint16_t color)
{
    uint16_t i, j;
    uint8_t data[2] = {color >> 8, color & 0xFF};

    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);

    for (j = 0; j < LCD_Buf_Size / 2; j++)
    {
        lcd_buf[j * 2]     = data[0];
        lcd_buf[j * 2 + 1] = data[1];
    }

    LCD_DC(1);
    for (i = 0; i < LCD_TOTAL_BUF_SIZE / LCD_Buf_Size; i++)
    {
        LCD_SPI_Send(lcd_buf, LCD_Buf_Size);
    }
}

// ==============================
// 区域填充
// ==============================
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;
    uint32_t remain = 0;

    if (size > LCD_Buf_Size)
    {
        remain = size - LCD_Buf_Size;
        size = LCD_Buf_Size;
    }

    LCD_Address_Set(x1, y1, x2, y2);

    while (1)
    {
        for (int i = 0; i < size / 2; i++)
        {
            lcd_buf[2*i]   = color >> 8;
            lcd_buf[2*i+1] = color;
        }

        LCD_DC(1);
        LCD_SPI_Send(lcd_buf, size);

        if (remain == 0) break;
        if (remain > LCD_Buf_Size)
        {
            remain -= LCD_Buf_Size;
        }
        else
        {
            size = remain;
            remain = 0;
        }
    }
}

// ==============================
// 画点
// ==============================
void LCD_Draw_Point(uint16_t x, uint16_t y)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(POINT_COLOR);
}

void LCD_Draw_ColorPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_Write_HalfWord(color);
}

// ==============================
// 画线
// ==============================
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int ux = dx > 0 ? 1 : -1;
    int uy = dy > 0 ? 1 : -1;
    int x = x1, y = y1;
    int eps = 0;

    dx = dx < 0 ? -dx : dx;
    dy = dy < 0 ? -dy : dy;

    if (dx > dy)
    {
        for (x = x1; x != x2 + ux; x += ux)
        {
            LCD_Draw_Point(x, y);
            eps += dy;
            if ((eps << 1) >= dx)
            {
                y += uy;
                eps -= dx;
            }
        }
    }
    else
    {
        for (y = y1; y != y2 + uy; y += uy)
        {
            LCD_Draw_Point(x, y);
            eps += dx;
            if ((eps << 1) >= dy)
            {
                x += ux;
                eps -= dy;
            }
        }
    }
}

// ==============================
// 画矩形
// ==============================
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}

// ==============================
// 画圆
// ==============================
void LCD_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r)
{
    int a = 0, b = r;
    int di = 3 - 2 * r;

    while (a <= b)
    {
        LCD_Draw_Point(x0 + a, y0 + b);
        LCD_Draw_Point(x0 - a, y0 + b);
        LCD_Draw_Point(x0 + a, y0 - b);
        LCD_Draw_Point(x0 - a, y0 - b);
        LCD_Draw_Point(x0 + b, y0 + a);
        LCD_Draw_Point(x0 - b, y0 + a);
        LCD_Draw_Point(x0 + b, y0 - a);
        LCD_Draw_Point(x0 - b, y0 - a);
        a++;
        if (di > 0)
        {
            b--;
            di += 4 * (a - b) + 10;
        }
        else
        {
            di += 4 * a + 6;
        }
    }
}

// ==============================
// 显示图片
// ==============================
void LCD_Show_Image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *p)
{
    if (x + w > LCD_Width || y + h > LCD_Height)
        return;

    LCD_Address_Set(x, y, x + w - 1, y + h - 1);
    LCD_DC(1);
    LCD_SPI_Send((uint8_t *)p, w * h * 2);
}

void LCD_TouchGFX_Flush(const uint16_t *frameBuffer)
{
    // 1. 窗口
    LCD_Address_Set(0, 0, LCD_Width - 1, LCD_Height - 1);
    LCD_DC(1);

    uint32_t total = LCD_Width * LCD_Height;
    const uint16_t *src = frameBuffer;

    const uint32_t block = LCD_Buf_Size / 2;

    while (total > 0)
    {
        uint32_t cnt = total > block ? block : total;

        // 翻转字节序 → 写入你现有的小缓存
        for (uint32_t i = 0; i < cnt; i++)
        {
            uint16_t c = src[i];
            lcd_buf[i*2]   = (c >> 8) & 0xFF;
            lcd_buf[i*2+1] = c & 0xFF;
        }

        LCD_SPI_Send(lcd_buf, cnt * 2);

        src += cnt;
        total -= cnt;
    }
}
// ==============================
// 初始化
// ==============================
void LCD_Init(void)
{
    LCD_RST(0);
    HAL_Delay(15);
    LCD_RST(1);
    HAL_Delay(15);

    LCD_Write_Cmd(0x11); HAL_Delay(12);
    LCD_Write_Cmd(0x36); LCD_Write_Data(0x60);
    LCD_Write_Cmd(0x3A); LCD_Write_Data(0x65);
    LCD_Write_Cmd(0xB2); LCD_Write_Data(0x0C); LCD_Write_Data(0x0C); LCD_Write_Data(0x00); LCD_Write_Data(0x33); LCD_Write_Data(0x33);
    LCD_Write_Cmd(0xB7); LCD_Write_Data(0x72);
    LCD_Write_Cmd(0xBB); LCD_Write_Data(0x3D);
    LCD_Write_Cmd(0xC0); LCD_Write_Data(0x2C);
    LCD_Write_Cmd(0xC2); LCD_Write_Data(0x01);
    LCD_Write_Cmd(0xC3); LCD_Write_Data(0x19);
    LCD_Write_Cmd(0xC4); LCD_Write_Data(0x20);
    LCD_Write_Cmd(0xC6); LCD_Write_Data(0x0F);
    LCD_Write_Cmd(0xD0); LCD_Write_Data(0xA4); LCD_Write_Data(0xA1);
    LCD_Write_Cmd(0xE0);
    LCD_Write_Data(0xD0);LCD_Write_Data(0x04);LCD_Write_Data(0x0D);LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);LCD_Write_Data(0x2B);LCD_Write_Data(0x3F);LCD_Write_Data(0x54);
    LCD_Write_Data(0x4C);LCD_Write_Data(0x18);LCD_Write_Data(0x0D);LCD_Write_Data(0x0B);
    LCD_Write_Data(0x1F);LCD_Write_Data(0x23);
    LCD_Write_Cmd(0xE1);
    LCD_Write_Data(0xD0);LCD_Write_Data(0x04);LCD_Write_Data(0x0C);LCD_Write_Data(0x11);
    LCD_Write_Data(0x13);LCD_Write_Data(0x2C);LCD_Write_Data(0x3F);LCD_Write_Data(0x44);
    LCD_Write_Data(0x51);LCD_Write_Data(0x2F);LCD_Write_Data(0x1F);LCD_Write_Data(0x1F);
    LCD_Write_Data(0x20);LCD_Write_Data(0x23);
    LCD_Write_Cmd(0x21);
    LCD_Write_Cmd(0x29);
    LCD_BL(1);
    LCD_Clear(BLACK);
}