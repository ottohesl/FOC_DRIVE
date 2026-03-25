#include "OLED_Menu.h"
#include <stdlib.h>
#include "cmsis_os2.h"
#include "ottohesl.h"
#include "usart.h"

// 全局菜单句柄（扩展后）
Menu_Handle current_menu={
    .current_level=MENU_LEVEL_1,
    .selected_idx=1,
    .idx_max=0,
    .level2_idx=0,  // 初始化二级菜单历史索引
    .level3_idx=0   // 初始化三级菜单历史索引
};

// 按键读取函数（保持你的原有逻辑，这里仅补全返回）
Key_Event get_key() {
    // 你的按键扫描逻辑
    if (HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 消抖
        if (HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET) {
            while(HAL_GPIO_ReadPin(KEY_EN_GPIO_Port, KEY_EN_Pin) == GPIO_PIN_RESET);
            return KEY_EVENT_ENTER;
        }
    }
    if (HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); // 消抖
        if (HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET) {
            while(HAL_GPIO_ReadPin(KEY_NE_GPIO_Port, KEY_NE_Pin) == GPIO_PIN_RESET);
            return KEY_EVENT_NEXT;
        }
    }
    return KEY_EVENT_NONE;
}

void level_1(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_1) {
        OLED_Clear();
        Key_Event key_event;
        do {
            key_event = get_key();
            OLED_ShowPicture(14,0,100,64,Robotic_picture_Data,OLED_NORMAL);
            OLED_Printf(85,39 , FONT_8, OLED_NORMAL, "MOTOR FOC");
            OLED_ShowFrame();
            osDelay(20);
        }while (key_event==KEY_EVENT_NONE);
        //检查到任意按键按下后，跳转到第二菜单
        current_menu->current_level = MENU_LEVEL_2;
    }
}

void OLED_Serial_Printf(uint8_t Ser, OLED_DisplayMode mode,const char *fmt, ...) {
    if(fmt == NULL) return;
    char buf[128] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Ser = (Ser - 1) % 4 + 1;
    OLED_Printf(0, (Ser-1)*FONT_16, FONT_16, mode, buf); // 修复：原代码传fmt，应该传buf
}

void level_2(Menu_Handle* current_menu) {
    current_menu->idx_max = 8;  //表示二级菜单总共菜单数量
    if (current_menu->current_level == MENU_LEVEL_2) {
        OLED_Clear();
        Key_Event key_event;
        while (1){
            // 显示二级菜单
            if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "实时监视            ");
                OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "参数设置            ");
                OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "快捷操作            ");
                OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "系统信息            ");
            }
            else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂回零位        ");
                OLED_Serial_Printf(6, (current_menu->selected_idx == 6) ? OLED_HIGHLIGHT : OLED_NORMAL, "数据              ");
                OLED_Serial_Printf(7, (current_menu->selected_idx == 7) ? OLED_HIGHLIGHT : OLED_NORMAL, "type5            ");
                OLED_Serial_Printf(8, (current_menu->selected_idx == 8) ? OLED_HIGHLIGHT : OLED_NORMAL, "return           ");
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                //让索引在规定范围循环
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }
            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==8) {
                    current_menu->current_level = MENU_LEVEL_1;
                    current_menu->selected_idx = 1; // 重置选中索引
                } else {
                    // 保存二级菜单选中的索引
                    current_menu->level2_idx = current_menu->selected_idx;
                    current_menu->current_level = MENU_LEVEL_3;
                    current_menu->selected_idx = 1; // 重置三级菜单起始索引
                }
                break;
            }
            osDelay(20);
        }
    }
}

void level_3(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_3) {
        OLED_Clear();
        Key_Event key_event;
        // 根据二级菜单选中的索引设置三级菜单最大索引
        switch (current_menu->level2_idx) {
            case 1: // 实时监视
                current_menu->idx_max = 5;
                break;
            case 2: // 参数设置
                current_menu->idx_max = 6;
                break;
            case 3: // 快捷操作
                current_menu->idx_max = 5;
                break;
            default: // 其他
                current_menu->idx_max = 4;
                break;
        }

        while (1) {
            // 根据二级菜单选中的索引显示对应三级菜单
            switch (current_menu->level2_idx) {
                case 1: //实时监视页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看电机状态    ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看末端位姿    ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "查看运动状态    ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, OLED_HIGHLIGHT, "查看电机状态    ");
                    }
                    break;
                case 2: //参数设置页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return          ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节角度      ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节长度      ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置关节速度      ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "设置工作空间限制  ");
                        OLED_Serial_Printf(6, (current_menu->selected_idx == 6) ? OLED_HIGHLIGHT : OLED_NORMAL, "end         ");
                    }
                    break;
                case 3: //快捷操作页面
                    OLED_Clear();
                    if(current_menu->selected_idx>0&&current_menu->selected_idx<5) {
                        OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return          ");
                        OLED_Serial_Printf(2, (current_menu->selected_idx == 2) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂回零位     ");
                        OLED_Serial_Printf(3, (current_menu->selected_idx == 3) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂校准      ");
                        OLED_Serial_Printf(4, (current_menu->selected_idx == 4) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂画正方形      ");
                    }
                    else if (current_menu->selected_idx>4&&current_menu->selected_idx<9) {
                        OLED_Serial_Printf(5, (current_menu->selected_idx == 5) ? OLED_HIGHLIGHT : OLED_NORMAL, "机械臂示教      ");
                    }
                    break;
                default:
                    OLED_Serial_Printf(1, OLED_NORMAL, "未知菜单        ");
                    break;
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                //让索引在规定范围循环
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }
            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==1) {
                    // 返回二级菜单
                    current_menu->current_level = MENU_LEVEL_2;
                    current_menu->selected_idx = current_menu->level2_idx; // 恢复二级菜单选中状态
                } else {
                    // 保存三级菜单选中的索引，进入四级菜单
                    current_menu->level3_idx = current_menu->selected_idx;
                    current_menu->current_level = MENU_LEVEL_4;
                    current_menu->selected_idx = 1; // 重置四级菜单起始索引
                }
                break;
            }
            osDelay(20);
        }
    }
}

void level_4(Menu_Handle* current_menu) {
    if (current_menu->current_level==MENU_LEVEL_4) {
        OLED_Clear();
        Key_Event key_event;
        // 根据二级+三级菜单索引设置四级菜单最大索引
        switch (current_menu->level2_idx) {
            case 1: // 实时监视下的四级菜单
                current_menu->idx_max = 2;
                break;
            case 2: // 参数设置下的四级菜单
                current_menu->idx_max = 6;
                break;
            case 3: // 快捷操作下的四级菜单
                current_menu->idx_max = 5;
                break;
            default:
                current_menu->idx_max = 1;
                break;
        }

        while (1) {
            // 根据二级+三级菜单索引显示对应四级菜单
            switch (current_menu->level2_idx) {
                case 1: // 实时监视页面的四级菜单
                    switch (current_menu->level3_idx) {
                        case 2: // 查看关节角度
                            OLED_Clear();
                            int rawangle=AS5600_ReadRawAngle(&i2c_AS5600 );
                            double angle=AS5600_GetAngleDegrees(&i2c_AS5600 );
                            int lr=AS5600_Get_LR(&i2c_AS5600 );
                            float turns=AS5600_Get_Turns(&i2c_AS5600 );

                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL,  "    电机状态    ");
                            OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_NORMAL, "原始值：%.2f",rawangle);
                            OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "角度：%.2f", angle);
                            OLED_Printf(64, 0*FONT_16+16, FONT_16,OLED_NORMAL, "圈数：%.2f",turns);
                            OLED_Printf(64, 1*FONT_16+16, FONT_16,OLED_NORMAL, "方向：%.2f",lr);
                            break;
                        case 3: // 查看末端位姿
                            OLED_Clear();
                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "位置 mm|姿态(度)");
                            OLED_Printf(0, 0*FONT_16+16, FONT_16,OLED_NORMAL, "X：%.1f");
                            OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "Y：%.1f");
                            OLED_Printf(0, 2*FONT_16+16, FONT_16,OLED_NORMAL, "Z：%.1f");
                            OLED_Printf(64, 0*FONT_16+16, FONT_16,OLED_NORMAL, "A：%.1f");
                            OLED_Printf(64, 1*FONT_16+16, FONT_16,OLED_NORMAL, "B：%.1f");
                            OLED_Printf(64, 2*FONT_16+16, FONT_16,OLED_NORMAL, "C：%.1f");
                            break;
                        case 4: // 查看运动状态
                            OLED_Clear();
                            OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "    运动状态    ");
                                OLED_Printf(0, 1*FONT_16+16, FONT_16,OLED_NORMAL, "机械臂运动中>>>");
                                OLED_Printf(0, 2*FONT_16+16, FONT_16,OLED_NORMAL, "误差:  mm");
                            break;
                    }
                    break;
                case 2: // 参数设置页面的四级菜单
                    OLED_Clear();
                    OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                    OLED_Serial_Printf(2, OLED_NORMAL, "设置值：%d     ", current_menu->selected_idx);
                    break;
                case 3: // 快捷操作页面的四级菜单
                    OLED_Clear();
                    OLED_Serial_Printf(1, (current_menu->selected_idx == 1) ? OLED_HIGHLIGHT : OLED_NORMAL, "return        ");
                    OLED_Serial_Printf(2, OLED_NORMAL, "执行：%s      ",
                        current_menu->level3_idx==2?"回零位":
                        current_menu->level3_idx==3?"校准":
                        current_menu->level3_idx==4?"画正方形":"示教");
                    break;
                default:
                    OLED_Serial_Printf(1, OLED_NORMAL, "无数据        ");
                    break;
            }
            OLED_ShowFrame();

            //按键扫描
            key_event = get_key();
            if (key_event==KEY_EVENT_NEXT) {
                current_menu->selected_idx++;
                if (current_menu->selected_idx > current_menu->idx_max) {
                    current_menu->selected_idx = 1;
                }

            }
            if (key_event==KEY_EVENT_ENTER) {
                if (current_menu->selected_idx==1) {
                    // 返回三级菜单
                    current_menu->current_level = MENU_LEVEL_3;
                    current_menu->selected_idx = current_menu->level3_idx; // 恢复三级菜单选中状态
                } else {
                    // 可扩展：执行具体操作
                }
                break;
            }
        }
        osDelay(20);
    }
}

void oled_loop() {
        // 根据当前层级执行对应菜单逻辑（原代码顺序执行会导致层级切换异常）
        switch (current_menu.current_level) {
            case MENU_LEVEL_1:
                level_1(&current_menu);
                break;
            case MENU_LEVEL_2:
                level_2(&current_menu);
                break;
            case MENU_LEVEL_3:
                level_3(&current_menu);
                break;
            case MENU_LEVEL_4:
                level_4(&current_menu);
                break;
            default:
                current_menu.current_level = MENU_LEVEL_1; // 异常层级重置
                break;
        }
}