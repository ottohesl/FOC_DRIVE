# FOC_DRIVER
--- 
# 硬件
<img width="1820" height="1024" alt="9cab4b58dcbb5f75e80285b25ca24353" src="https://github.com/user-attachments/assets/d429619d-92be-4323-8fe5-f71d9d0d7f7a" />

## 一、上层主控板硬件设计

<img width="1234" height="795" alt="image" src="https://github.com/user-attachments/assets/73410c57-878a-4375-94a4-25fecececb91" />
上层板为系统控制核心板，以STM32G473为主控芯片，主要负责系统逻辑控制、信号运算、外设通信及PWM控制信号输出等核心功能。  

电源输入方面，上层板支持Type-C接口5V电压输入，适配常规调试供电场景。板载LDO降压电路，可将输入的5V电压稳压降压至3.3V，为STM32G473主控、LCD显示模块及板载各类弱电外设提供稳定工作电源，保障控制电路正常运行。  

为满足项目后续外设拓展与功能迭代需求，主控板集成了标准化I2C、USART板载通信接口，预留充足的外设拓展资源，可便捷外接传感器、调试模块、拓展模组等外部设备，极大提升了硬件平台的通用性与二次开发空间。工作过程中，主控板通过FPC软排线与下层驱动板建立连接，将生成的PWM驱动信号、控制指令等信号传输至下层驱动板，实现上下层电路的信号交互与系统联动。

## 二、下层驱动板硬件设计

<img width="1314" height="824" alt="image" src="https://github.com/user-attachments/assets/a6a2322a-9d76-4a10-944d-07ea9cfba04e" />
<img width="1332" height="822" alt="image" src="https://github.com/user-attachments/assets/b25470cb-bd2f-482a-a588-6c4a1acd75b2" />
使用kicad设计PCB
<img width="1917" height="1018" alt="image" src="https://github.com/user-attachments/assets/3e245f9a-5a89-41f8-ab8e-edec25f3e5d9" />

下层板为功率驱动核心板，是整个系统的主电源输入与电机驱动执行单元，主要接收上层主控板通过FPC传输的PWM控制信号，完成电机的驱动控制，本项目采用六步换相控制方式实现电机运转驱动。  

电源系统方面，下层驱动板为整机主供电端，外部接入24V工业供电电压。板载TPS5450 DC-DC降压芯片，先将24V输入电压降压至12V，为电机驱动功率电路、栅极驱动芯片等功率器件供电；再通过二级DC-DC降压电路将12V电压转换为5V，该路5V电源可通过FPC软排线反向为上层主控板供电，实现整机单电源供电适配，兼顾独立供电与联动供电两种使用场景。  

电机驱动电路方面，当前硬件采用DRV8323H最为电机驱动芯片，搭配MOS管搭建H桥驱动电路，依托芯片自举电容升压原理，实现H桥电路上下桥臂的有序导通与关断，配合六步换相控制逻辑，完成直流电机的稳定驱动，满足基础电机调速、正反转控制的验证需求。

---

# 软件层面

## 一、代码层级

代码严格按照BSP、Middle、App层级开发，当前BSP、Middle的基本框架已经形成，app待开发。
  
<img width="353" height="503" alt="image" src="https://github.com/user-attachments/assets/49a82cc1-f185-4e0c-a0ec-19ae602d2f39" />
     
### **BSP** 
包含**as5600**的i2c通讯的编码器文件、基础**FOC**算法、**FreeRTOS**应用文件、**LCD|OLED**的**Menu**显示

<img width="342" height="550" alt="image" src="https://github.com/user-attachments/assets/c104777b-cf82-42fe-be19-cb60b106cacd" />
     
###  **Middle**
-**ottohesl**文件是自写的**UART和USB CDC**的辅助调试工具，仅需串口或者typec连接电脑利用VOFA+查看各种数据。\
-**UI**文件是基于ST的**TouchGFX**插件为了将FOC里面更新的数据例如`当前角度`、`母线电压`、`RPM`等显示到LCD上面

<img width="349" height="170" alt="image" src="https://github.com/user-attachments/assets/d8ca2bfb-b971-481e-8e0c-47ea6c0da998" />

---
## 二、相关代码展示

**SPWM**算法就是基本的三相正弦波，使用帕克逆变换和克拉克逆变换求解三相电压

```
/**
 * @brief  求解SPWM的相关系数
 * @param  spwm  SPWM结构体的句柄，访问里面数据进行读写
 */
void FOC_Spwm_Solve(SPWM *spwm) {
    //帕克逆变换
    double ualp = -spwm->qd.uq * sin(spwm->elect_angle) + spwm->qd.ud * cos(spwm->elect_angle);
    double ubet = spwm->qd.uq * cos(spwm->elect_angle) + spwm->qd.ud * sin(spwm->elect_angle);
    //克拉克逆变换
    spwm->abc_v.ua=ualp+LIMIT_V/2;
    spwm->abc_v.ub=(SQRT_3*ubet-ualp)/2+LIMIT_V/2;
    spwm->abc_v.uc=(-ualp-SQRT_3*ubet)/2+LIMIT_V/2;
}
```
**SVPWM** 基于αβ两相坐标系，利用矢量合成、扇区判断、作用时间生成每相 PWM 导通时间，最终生成三相马鞍波形
```
/**
 * @brief 扇区判断
 * @param svpwm  SVPWM控制结构体
 * @param sector 输出扇区号
 * @return N值
 */
uint8_t Sector_Judgment(SVPWM *svpwm, uint8_t *sector)
{
    svpwm->svpwm_val1 = svpwm->ab.alpha * SQRT_3_DIV_2;
    svpwm->svpwm_val2 = svpwm->ab.beta / 2.0f;

    float A = svpwm->ab.beta;
    float B = svpwm->svpwm_val1 - svpwm->svpwm_val2;
    float C = -svpwm->svpwm_val1 - svpwm->svpwm_val2;

    uint8_t N = 0;
    if(A > 0)      N += 1;
    if(B > 0)      N += 2;
    if(C > 0)      N += 4;

    // 将N转换为扇区号(1-6)
    switch(N){
        case 3: *sector = 1; break;
        case 1: *sector = 2; break;
        case 5: *sector = 3; break;
        case 4: *sector = 4; break;
        case 6: *sector = 5; break;
        case 2: *sector = 6; break;
        default: *sector = 1; break; // 默认扇区1
    }
    return N;
}
```
**电流环、速度环**
```
/**
 * @brief 速度环
 * @param motor 电机结构体句柄
 * @param target_speed 目标每秒转速
 * @return 输出给电流环
 */
float FOC_SPEED_DRIVE(FOC_DRIVE *motor,float target_speed) {
    //求解电角度
    motor->FOC_SVPWM.elect_angle = Solve_Electrical_Angle(motor->FOC_ENC_DATA.Enc_angle);
    //pid求解
    FOC_INC_PID(&motor->FOC_PID.spe_pid,target_speed,motor->FOC_ENC_DATA.Enc_speed);
    return -motor->FOC_PID.spe_pid._output;
}

/**
 * @brief 电流环
 * @param motor 电机结构体句柄
 * @param Iq 目标电流
 */
void FOC_CUR_DRIVE(FOC_DRIVE *motor,float Iq){
    //克拉克变换
    float Ialpha = motor->FOC_CUR_PARAM.Ia;
    float Ibeta = (motor->FOC_CUR_PARAM.Ib-motor->FOC_CUR_PARAM.Ic) * SQRT_3_DIV_2;
    //帕克变换
    float elect_angle =motor->FOC_SVPWM.elect_angle;
    motor->FOC_SVPWM.iqd.id = Ialpha * cosf(elect_angle) + Ibeta * sinf(elect_angle);
    motor->FOC_SVPWM.iqd.iq = -Ialpha * sinf(elect_angle) + Ibeta * cosf(elect_angle);
    //将给定需要的值进行pi调节
    FOC_POS_PID(&motor->FOC_PID.iq_pid,Iq,motor->FOC_SVPWM.iqd.iq);
    FOC_POS_PID(&motor->FOC_PID.id_pid,0,motor->FOC_SVPWM.iqd.id);
    //输出Uq，作用于电机
    motor->FOC_SVPWM.qd.uq = motor->FOC_PID.iq_pid._output;
    motor->FOC_SVPWM.qd.ud = motor->FOC_PID.id_pid._output;
    //反park变化获取alpha和beta
    FOC_Svpwm_Solve(&motor->FOC_SVPWM);
    //给svpwm让电机旋转
    // 矢量时间计算
    uint8_t N = Sector_Judgment(&motor->FOC_SVPWM, &motor->FOC_SVPWM.sector);
    VectorActionTime(&motor->FOC_SVPWM, N, motor->FOC_CUR_PARAM.Bus_Voltage);
    FOC_Set_Svpwm(&motor->FOC_SVPWM,motor->FOC_CUR_PARAM.Bus_Voltage);
}
```
