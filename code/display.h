/*
 * display.h
 *
 *  Created on: 2025年11月20日
 *      Author: 18905
 */

#ifndef CODE_DISPLAY_H_
#define CODE_DISPLAY_H_

//外部变量
typedef enum{
    Mode_IDLE,
    Guandao_Recode_Mode,
    Guandao_portion_1,
    Guandao_Voice,
    Guandao_portion_3,
    Rack_Test_Mode
}Mode_Choice;

extern Mode_Choice main_mode;

extern uint8 key_mode1;
extern uint8 key_mode2;
extern uint8 CarGo_Flag;
extern float *p;
//宏定义
#define IPS200_TYPE     (IPS200_TYPE_SPI)    //屏幕初始化宏定义
#define X(x)                    8*(x)
#define Y(y)                    16*(y)
//函数定义

void Display_Init(void);
uint8 Key_Get(void);                                                    //界面切换按键扫描
void prompt(void);                                                     //提示信息巡回
void Menu_Contral(void);                                            //主循环控制
int16 Menu_key_Operation_int16(int16 *param_t );//参数设置模式 通过指针直接访问变量地址 函数数据类型与参数类型一一对应
float Menu_key_Operation_float(float *param_t );//参数设置模式 通过指针直接访问变量地址 函数数据类型与参数类型一一对应
void Menu_Main(void);
void Menu_1(void);
void Menu_Parameter(void);
void Menu_2Value(void);
void Menu_Control_Value(void);
void Menu_Mode_Choice(void);
void Menu_Recode_Points(void);
void GPS_prompt(void);
void Menu_Show_Route(void);
void Show_Route(void);
void Menu_PID_P(void);
void Menu_Control_P(void);
#endif /* CODE_DISPLAY_H_ */
