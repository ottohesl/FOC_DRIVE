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

下层板为功率驱动核心板，是整个系统的主电源输入与电机驱动执行单元，主要接收上层主控板通过FPC传输的PWM控制信号，完成电机的驱动控制，本项目采用六步换相控制方式实现电机运转驱动。  

电源系统方面，下层驱动板为整机主供电端，外部接入24V工业供电电压。板载TPS5450 DC-DC降压芯片，先将24V输入电压降压至12V，为电机驱动功率电路、栅极驱动芯片等功率器件供电；再通过二级DC-DC降压电路将12V电压转换为5V，该路5V电源可通过FPC软排线反向为上层主控板供电，实现整机单电源供电适配，兼顾独立供电与联动供电两种使用场景。  

电机驱动电路方面，当前硬件采用FD6288T栅极驱动芯片搭配MOS管搭建H桥驱动电路，依托芯片自举电容升压原理，实现H桥电路上下桥臂的有序导通与关断，配合六步换相控制逻辑，完成直流电机的稳定驱动，满足基础电机调速、正反转控制的验证需求。

## 三、验证版测出的问题

### 1. 栅极驱动芯片性能问题

现有FD6288T栅极驱动芯片存在自举升压不足的问题，电机高速、满载工况下易出现MOS管导通不完全的情况，造成电机抖动、扭矩不足、驱动效率偏低，无法充分发挥FOC高精度控制性能。

### 2. 整机电源保护电路缺失，上电浪涌问题严重

由于本身foc板子是用于验证的，电路未有防浪涌、防过流、防过压的保护，驱动板后端大容量滤波电容，在6S锂电池上电时会产生明显浪涌电压与电流，易出现电容打火、电源芯片损坏、PCB铜皮撕裂等问题，硬件上电安全性与可靠性较差。

### 3. FPC连接电路设计简单

下层板FPC接口无二极管、限流稳压等防护电路，存在设计缺陷，不仅限制上下两层板无法同时上电，还会导致上层外设短路、电压波动等异常情况串扰至下层驱动板，较易引发信号干扰、电路短路故障，系统抗干扰能力差。

--- 
*ps：后续会将栅极驱动芯片替换为TI品牌DRV8323系列栅极驱动芯片。这个芯片不仅兼容原有电机驱动、PWM信号解析、自举升压等全部基础功能，还内置高精度ADC采样模块，可配合外接采样电阻实现电机工作电流的实时采集，既能够解决原有芯片升压不足的问题，还可直接实现电流闭环控制。（现在做了个N合一电路也就是防反接&缓启动&防浪涌&防过压&滤波一体的输入保护网络，已经打板准备测试了）*

---

# 软件层面

## 一、代码层级

代码严格按照BSP、Middle、App层级开发，当前还在完善BSP、Middle的内容。
  
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
/**
 * @brief 矢量作用时间计算
 * @param foc_pwm SVPWM控制结构体
 * @param sector 扇区
 * @param alphabeta αβ轴电压
 * @param Tpwm PWM周期(计数值)
 * @param Udc 母线电压
 */
void VectorActionTime(SVPWM *foc_pwm, uint8_t sector, uint32_t Tpwm, float Udc)
{
    Udc = Udc * 1.5f;
    float K = (float)Tpwm * SQRT_3 / Udc;

    float X = K * foc_pwm->ab.beta;
    float Y = K * (foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);
    float Z = K * (-foc_pwm->svpwm_val1 + foc_pwm->svpwm_val2);

    uint32_t T4 = 0, T6 = 0;
    uint32_t Ta = 0, Tb = 0, Tc = 0;
    uint32_t T1 = 0, T2 = 0, T3 = 0;

    // 根据扇区计算矢量时间
    switch(sector){
        case 1: T4 = Z;  T6 = Y;  break;
        case 2: T4 = Y;  T6 = -X; break;
        case 3: T4 = -Z; T6 = X;  break;
        case 4: T4 = -X; T6 = Z;  break;
        case 5: T4 = X;  T6 = -Y; break;
        case 6: T4 = -Y; T6 = -Z; break;
    }

    // 过调制处理
    if(T4 + T6 > (Tpwm * foc_pwm->K)){
        float ratio = (Tpwm * foc_pwm->K) / (T4 + T6);
        T4 *= ratio;
        T6 *= ratio;
    }

    // 计算各相作用时间
    Ta = (Tpwm - T4 - T6) / 4;
    Tb = Ta + T4 / 2;
    Tc = Tb + T6 / 2;

    // 根据扇区分配时间到各相
    switch(sector){
        case 1: T1 = Tb; T2 = Ta; T3 = Tc; break;
        case 2: T1 = Ta; T2 = Tc; T3 = Tb; break;
        case 3: T1 = Ta; T2 = Tb; T3 = Tc; break;
        case 4: T1 = Tc; T2 = Tb; T3 = Ta; break;
        case 5: T1 = Tc; T2 = Ta; T3 = Tb; break;
        case 6: T1 = Tb; T2 = Tc; T3 = Ta; break;
    }

    // 保存各相时间
    foc_pwm->T_abc.ua = T1;
    foc_pwm->T_abc.ub = T2;
    foc_pwm->T_abc.uc = T3;

    // 死区补偿(根据实际硬件调整)
    float deadtime_comp = 0;
    T1 += deadtime_comp;
    T2 += deadtime_comp;
    T3 += deadtime_comp;

    // 计算占空比
    foc_pwm->_output.ua = (float)T1 / Tpwm * LIN_V;
    foc_pwm->_output.ub = (float)T2 / Tpwm * LIN_V;
    foc_pwm->_output.uc = (float)T3 / Tpwm * LIN_V;
}
/**
 * @brief 帕克逆变换
 * @param svpwm 计算结果记录于该结构体内
 */
void FOC_Svpwm_Solve(SVPWM *svpwm) {
    //帕克逆变换
    svpwm->ab.alpha = -svpwm->qd.uq * sin(svpwm->elect_angle) + svpwm->qd.ud * cos(svpwm->elect_angle);
    svpwm->ab.beta = svpwm->qd.uq * cos(svpwm->elect_angle) + svpwm->qd.ud * sin(svpwm->elect_angle);
}
void Set_Svpwm(SVPWM *svpwm){
    double ua=constrain((svpwm->_output.ua+LIN_V/2),0.0f,LIN_V);
    double ub=constrain((svpwm->_output.ub+LIN_V/2),0.0f,LIN_V);
    double uc=constrain((svpwm->_output.uc+LIN_V/2),0.0f,LIN_V);

    // 电压转化为占空比
    float dc_a=constrain(ua/LIN_V,0.0f,1.0f);
    float dc_b=constrain(ub/LIN_V,0.0f,1.0f);
    float dc_c=constrain(uc/LIN_V,0.0f,1.0f);
    uint32_t PSC,ARR;
    Get_PSC_ARR(FOC_TIM,&PSC,&ARR);
    // 设置PWM比较值
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_1, dc_a*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_2, dc_b*ARR);
    __HAL_TIM_SET_COMPARE(FOC_TIM, TIM_CHANNEL_3, dc_c*ARR);
}
```
