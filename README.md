<img width="1234" height="795" alt="image" src="https://github.com/user-attachments/assets/73410c57-878a-4375-94a4-25fecececb91" />
1.0 采用STM32G473的主控芯片，完成上层控制板的绘制。上层板子仅typec输入5v电源，利用ldo降压3.3v给LCD和主控mcu等供电;其中板子集成了基础I2C、USART的板载接口，方便外部其他外设开发。
<img width="1314" height="824" alt="image" src="https://github.com/user-attachments/assets/a6a2322a-9d76-4a10-944d-07ea9cfba04e" />
    下层板子为驱动板，主要是根据中间FPC连接的上层控制板的pwm等信号来驱动电机。驱动板目前使用fd6288t的栅极驱动芯片，利用自举电容升压完成一系列的由mos管组成的H桥的开闭达到驱动电机，也就是常用的6步换相法。
    驱动板是主要的电源输入端，使用tps5450用于将24v电压降压为12v，使用dcdc再降压12v转5v，该5v电源则经过FPC软排线给上层控制板供电。
···还存在的问题：fd6288t有明显的自举升压不够问题，后续改进需要跟换驱动芯片，目前想到使用TI的drv8323系列栅极驱动芯片，除应有的功能外，内置了adc可以直接通过外置电阻进行测电流。
                由于本身foc板子是用于验证的，电路未有防浪涌、防过流、防过压的保护，由于输入电源后继电路的大量电容，容值较高导致我6s锂电池上电形成浪涌电压明显，及其容易造成刚上电电容短路的打火和dcdc芯片浪涌电压导致的铜皮撕裂问题。
                FPC的下层板子的连接座较为简单，没有二极管等保护措施，那么上下两层板不能同时供电；且由于外设部分的线路问题，造成的短路问题及其容易波动到下次板子，也导致下层板子的串扰甚至短路
     <img width="1820" height="1024" alt="9cab4b58dcbb5f75e80285b25ca24353" src="https://github.com/user-attachments/assets/d429619d-92be-4323-8fe5-f71d9d0d7f7a" />

