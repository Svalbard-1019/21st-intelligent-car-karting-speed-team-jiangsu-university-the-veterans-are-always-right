/*
 * UTF-8 详细注释说明：遥控器输入读取和通道映射模块。
 *
 * 当前科目一调试中遥控器扫描多处被禁用，主要是为了释放 P33_6/P33_7 给后轮编码器。
 * 如果重新启用遥控器，一定要确认 X6F 通道引脚没有和编码器、GPS、屏幕或转向编码器冲突。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 中 hotRc_Control_init() 已注释，CCU60_CH1 中断里的 x6f_scan() 也被注释，防止遥控占用 P33_6/P33_7 等关键引脚。
 * 2. YAOKONG 模式保留 hotRC_control()/Speed_Control() 旧链路，以后需要遥控接管时可以恢复。
 * 3. 科目一自动驾驶不依赖遥控器，速度来自 guandao 的 out_v_l/out_v_r，方向来自 out_servo。
 * 4. 如果重新启用遥控器，必须先确认通道引脚没有和后轮编码器、GPS 或下载串口冲突。
 */


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
/**
 * 函数说明：hotRc_Control_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：遥控器输入模块，当前科目一和 RackTest 基本禁用，避免和编码器/GPS 引脚冲突。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：hotRc_Show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：遥控器输入模块，当前科目一和 RackTest 基本禁用，避免和编码器/GPS 引脚冲突。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：x6f_scan()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：遥控器输入模块，当前科目一和 RackTest 基本禁用，避免和编码器/GPS 引脚冲突。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
/**
 * 函数说明：hotRC_control()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：遥控器输入模块，当前科目一和 RackTest 基本禁用，避免和编码器/GPS 引脚冲突。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

