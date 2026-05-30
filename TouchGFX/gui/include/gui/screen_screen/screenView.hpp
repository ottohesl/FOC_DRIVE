#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>

class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    //更新电机相关数据
    void updateVoltage(float value);
    void updateAngle1(float value);
    void updateAngle2(float value);
    void updateCur1(float value);
    void updateCur2(float value);
    void updateMode1(const char* ch);
    void updateMode2(const char* ch);
    void updateRps1(float value);
    void updateRps2(float value);
protected:
    virtual void handleTickEvent();
};

#endif // SCREENVIEW_HPP
