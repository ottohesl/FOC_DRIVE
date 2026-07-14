#include <gui/screen_screen/screenView.hpp>
// 必须加：TouchGFX 字符串/浮点数转换头文件
#include <touchgfx/Unicode.hpp>
#ifdef __cplusplus
extern "C" {
#endif
#include "../../../../Middle_Layer/UI/UI_Component.h"
#ifdef __cplusplus
}
#endif

static screenView* g_screenView_ptr = NULL;

screenView::screenView()
{

}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();
    g_screenView_ptr = this;
}

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();
    g_screenView_ptr = NULL;
}

/**
 * @brief 所有UI数据更新处
 */
void screenView::handleTickEvent()
{
    UI_Update();
    invalidate();
}

/************************* 更新电压 **************************/
void screenView::updateVoltage(float value)
{
    touchgfx::Unicode::snprintfFloat(VoltageBuffer, VOLTAGE_SIZE, "%.1f", value);

}

extern "C" void Updata_Voltage(float value)
{
    if(g_screenView_ptr != NULL)
    {
        g_screenView_ptr->updateVoltage(value);
    }
}

/************************* 更新角度 **************************/
void screenView::updateAngle1(float value)
{
    touchgfx::Unicode::snprintfFloat(ANGLE1Buffer, ANGLE1_SIZE, "%.1f", value);

}

void screenView::updateAngle2(float value)
{
    touchgfx::Unicode::snprintfFloat(ANGLE2Buffer, ANGLE2_SIZE, "%.1f", value);

}

extern "C" void Updata_Angle(float value1,float value2)
{
    if(g_screenView_ptr != NULL)
    {
        g_screenView_ptr->updateAngle1(value1);
        g_screenView_ptr->updateAngle2(value2);
    }
}

/************************* 更新电流 **************************/
void screenView::updateCur1(float value)
{
    touchgfx::Unicode::snprintfFloat(CUR1Buffer, CUR1_SIZE, "%.1f", value);

}

void screenView::updateCur2(float value)
{
    touchgfx::Unicode::snprintfFloat(CUR2Buffer, CUR2_SIZE, "%.1f", value);

}

extern "C" void Updata_Cur(float value1,float value2)
{
    if(g_screenView_ptr != NULL)
    {
        g_screenView_ptr->updateCur1(value1);
        g_screenView_ptr->updateCur2(value2);
    }
}
/************************* 更新速度 **************************/
void screenView::updateRps1(float value)
{
    touchgfx::Unicode::snprintfFloat(RPS1Buffer, RPS1_SIZE, "%.2f", value);
}
void screenView::updateRps2(float value)
{
    touchgfx::Unicode::snprintfFloat(RPS2Buffer, RPS2_SIZE, "%.2f", value);
}
extern "C" void Updata_RPS(float value1,float value2)
{
    if(g_screenView_ptr != NULL)
    {
        g_screenView_ptr->updateRps1(value1);
        g_screenView_ptr->updateRps2(value2);
    }
}

/************************* 更新模式 **************************/
void screenView::updateMode1(const char* ch)
{
    touchgfx::Unicode::strncpy(MODE1Buffer, ch, MODE1_SIZE);
}

void screenView::updateMode2(const char* ch)
{
    touchgfx::Unicode::strncpy(MODE2Buffer, ch, MODE2_SIZE);
}

extern "C" void Updata_Mode(const char* ch1,const char* ch2)
{
    if(g_screenView_ptr != NULL)
    {
        g_screenView_ptr->updateMode1(ch1);
        g_screenView_ptr->updateMode2(ch2);
    }
}