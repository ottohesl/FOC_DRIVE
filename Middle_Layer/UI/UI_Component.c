#include "UI_Component.h"
MUY motor1;
MUY motor2;
/**
 * @param muy1 电机1ui的结构体指针变量
 * @param muy2 电机2ui的结构体指针变量
 * @note 后续将每个电机的电压、电流、速度、模式直接进行赋值
 * @note 里面的调用函数来自TOUCHgfx里面的screeView.cpp
 */
void UI_RUN(MUY *muy1, MUY *muy2) {
    Updata_Voltage(muy1->Voltage);
    Updata_Cur(muy1->ui.real_Cur,muy2->ui.real_Cur);
    Updata_Angle(muy1->ui.real_Angle,muy2->ui.real_Angle);
    Updata_Mode(muy1->ui.real_Mode, muy2->ui.real_Mode);
    Updata_RPS(muy1->ui.real_Speed, muy2->ui.real_Speed);
}
/**
 * @brief motor1/motor2为各电机ui名称结构体
 * @note 后续将每个电机的电压、电流、速度、模式直接进行赋值
 */
void UI_Update(void) {
    motor1.Voltage = MOTOR.FOC_CUR_PARAM.Bus_Voltage;
    motor1.ui.real_Angle=MOTOR.FOC_ENC_DATA.Enc_angle;
    motor1.ui.real_Speed=MOTOR.FOC_ENC_DATA.Enc_speed;
    motor1.ui.real_Cur= MOTOR.FOC_SVPWM.iqd.iq;
    strcpy(motor1.ui.real_Mode, "pos");
    motor2.Voltage = 24.2f;
    motor2.ui.real_Angle=MOTOR.FOC_ENC_DATA.Enc_angle;
    motor2.ui.real_Speed=MOTOR.FOC_ENC_DATA.Enc_speed;
    motor2.ui.real_Cur= MOTOR.FOC_SVPWM.iqd.iq;
    strcpy(motor2.ui.real_Mode, "spe");
    UI_RUN(&motor1,&motor2);
}