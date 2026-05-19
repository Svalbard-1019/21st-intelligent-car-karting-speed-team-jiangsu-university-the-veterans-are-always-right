/*
 * RemteControl.c
 *
 *  Created on: 2025年2月6日
 *      Author: FELMLY
 */

#include "zf_common_headfile.h"     //声明一下所有的头文件

////各通道引脚
int16 x6f_pin_map[6] = {X6F_CH1, X6F_CH2, X6F_CH3, X6F_CH4, X6F_CH5, X6F_CH6};
////各通道高电平计数变量
int16 x6f_count[6];
////各通道高电平计数输出
int16 x6f_out[6];
//
int   HotRC_GO_FLAG=0;//遥控发车标志位
////===================================================逐飞遥控===================================================
//

 float hot_rc_speed = 0;
 float hot_rc_steer = 0;
 float hot_rc_delta = 0;
void hotRc_Control_init(void)//遥控器引脚初始化
{

//    pit_ms_init(CCU60_CH1, 1);
   //初始化接收机引脚
   gpio_init(X6F_CH1, GPI, GPIO_LOW, GPI_PULL_UP);
   gpio_init(X6F_CH2, GPI, GPIO_LOW, GPI_PULL_UP);
   gpio_init(X6F_CH3, GPI, GPIO_LOW, GPI_PULL_UP);
   gpio_init(X6F_CH4, GPI, GPIO_LOW, GPI_PULL_UP);
//   gpio_init(X6F_CH5, GPI, GPIO_LOW, GPI_PULL_UP);
//   gpio_init(X6F_CH6, GPI, GPIO_LOW, GPI_PULL_UP);

   // 定时器中断10us，用于遥控器扫描
   pit_us_init(CCU60_CH1, 10);


}

void hotRc_Show(void)
{

    ips200_show_int(10*0,16*18,x6f_out[0],4);
    ips200_show_int(10*4,16*18,x6f_out[1],3);
    ips200_show_int(10*8,16*18,x6f_out[2],4);
    ips200_show_int(10*12,16*18,x6f_out[3],3);
    ips200_show_int(10*16,16*18,x6f_out[4],4);
//    ips200_show_int(10*20,16*18,x6f_out[5],4);
//    ips200_show_string(10*0,16*8,"Actual;");               ips200_show_float(10*12, 16*8, Actual, 4, 6);
//    ips200_show_string(10*0,16*9,"Out;");                  ips200_show_float(10*12, 16*9, Out, 4, 6);
//    ips200_show_string(10*0,16*10,"Target;");               ips200_show_int(10*15,16*10,Target,3);
//    ips200_show_string(10*0,16*11,"Z_360;");                ips200_show_float(10*12, 16*11, Z_360, 4, 6);
//    ips200_show_string(10*0,16*12,"SE_Out;");               ips200_show_float(10*12, 16*12, SE_Out, 4, 6);
//    ips200_show_string(10*0,16*13,"SE_Target;");            ips200_show_int(10*15,16*13,SE_Target,3);
//    ips200_show_int(10*0,16*14,MOTOR_CHOOSE,3);

}

void  x6f_scan(void)//遥控器通道扫描,需要放到10us的中断里
{
    for(int i = 0; i < 6; i ++)
    {
        if(gpio_get_level(x6f_pin_map[i]))
        {
            x6f_count[i]++;

        }
        else if(x6f_count[i] > 0)
        {
            x6f_out[i] = x6f_count[i];
            x6f_count[i] = 0;
        }
    }

}

//void HotRC_GO_Flag(void) //遥控标志位函数,选择时遥控或自动模式，放在主循环或者定时器中断里
//{
//    if(x6f_out[2]>=150)
//    {
//        HotRC_GO_FLAG = 1;
//    }
//    else
//    {
//        HotRC_GO_FLAG = 0;
//    }
//}
/*****************************************
函数功能；遥控滚轮控制舵机目标角度
函数名称；uint16_t map_x6f_to_SE_Target(int16_t x6f_out_0)
输入参数；
输出参数
备注；
*****************************************/
//void map_x6f_to_SE_Target(int16_t x6f_out_0)
//{
//    // 输入范围限制
//    if (x6f_out_0 < 100)
//    {
////        SE_Target = 150;
//    } else if (x6f_out_0 > 200)
//    {
//        x6f_out_0 = 150;
//    }
//    // 线性映射计算
////    SE_Target = 150 - x6f_out_0;
//}

/*****************************************
函数功能；遥控滚轮控制舵机PWM
函数名称；uint16_t map_x6f_to_pwm(int16_t x6f_out_0)
输入参数；
输出参数
备注；
*****************************************/
//uint16_t map_x6f_to_pwm(int16_t x6f_out_0)
//{
//    // 输入范围限制
//    if (x6f_out_0 < 100)
//    {
//        x6f_out_0 = 150;
//    } else if (x6f_out_0 > 200)
//    {
//        x6f_out_0 = 150;
//    }
//    // 线性映射计算
//    uint16_t pwm = (uint16_t)(3 * x6f_out_0 + 300);
//    return pwm;
//}
//
//int16_t motor_map_x6f_to_pwm(int16_t x6f_out_0)
//{
//    // 输入范围限制
//    if (x6f_out_0 < 100)
//    {
//        x6f_out_0 = 150;
//    } else if (x6f_out_0 > 200)
//    {
//        x6f_out_0 = 150;
//    }
//    // 线性映射计算
//    int16_t pwm = (uint16_t)(198 * (x6f_out_0 - 150));
//    return pwm;
//}

void hotRC_control(void)    //放入主循环，或者中断里
{

    hot_rc_steer = -2.125*x6f_out[0] +318.75;
    if(hot_rc_steer<2 &&hot_rc_steer>-2)hot_rc_steer = 0;

    if(x6f_out[3] != 100)
    {
        if(!gpio_get_level(SWITCH2))
        {
            hot_rc_speed = 1.5*x6f_out[1]-217.5;
            if(hot_rc_speed<0)hot_rc_speed = 0;
            if(hot_rc_speed >15)hot_rc_speed =  15;
        }
        else
        {
            hot_rc_speed = 4*x6f_out[1]-580;
            if(hot_rc_speed<0)hot_rc_speed = 0;
            if(hot_rc_speed >30)hot_rc_speed =  40;
        }

    }
    else
    {
        hot_rc_speed = -1.5*x6f_out[1]+217.5;
        if(hot_rc_speed>0)hot_rc_speed = 0;
        if(hot_rc_speed <-15)hot_rc_speed =  -15;
    }
    hot_rc_delta = (hot_rc_speed * tanf(hot_rc_steer/3.0/180.0f*M_PI)) / WHEEL_BASE;







}

