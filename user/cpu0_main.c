/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "rear_motor/rear_motor.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

extern int num;

#define GUANDAO_SPEED_TO_MPS    (0.1f)

static void Guandao_Rear_Motor_Update(void)
{
    float target_mps = 0.0f;

    if(conrtol_mode == GUANDAO)
    {
        target_mps = (out_v_l + out_v_r) * 0.5f * GUANDAO_SPEED_TO_MPS;
    }
    else if(conrtol_mode == DAOCHE)
    {
        target_mps = daoche_speed * GUANDAO_SPEED_TO_MPS;
    }
    else if(main_mode != Rack_Test_Mode)
    {
        rear_motor_stop();
        return;
    }
    else
    {
        return;
    }

    if(target_mps == 0.0f)
    {
        rear_motor_stop();
    }
    else
    {
        rear_motor_set_target_mps(target_mps);
        rear_motor_pid_update_100ms();
    }
}
double gk_d = 0;
double gk_a = 0;
uint8 port2_flag = 0;




int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    Init_All();
    rear_motor_init();
    // 前轮转向改为 TIM4 磁编码器闭环，不再初始化 SPI 绝对值编码器
//    hotRc_Control_init();                                                              // RackTest禁用遥控器，避免占用P33_6/P33_7编码器
   pit_ms_init(CCU61_CH1, 1);
   pit_ms_init(CCU61_CH0, 1);



    cpu_wait_event_ready();                                                          // 等待所有核心初始化完毕

    Flash_Main_Read();                                                                  // 从Flash存储器中读取主要配置参数

    Menu_Contral();                                                                  // 调用菜单控制函数，显示和操作配置菜单

    Flash_Write_pid();                                                               //Flash写入

    if(GPS_WORK_FLAG){GPS_WorkMap_Copy(&INS);}       //如果GPS工作标志为真  将GPS路径点数据复制到导航数据结构中
//    Buzzer_check(500);
    Main_Key_Flag = 1;                                                            // 中断控制开始标志为1，主循环和中断控制同时启动
//    build_map_text(&INS);
//    while(Steer_Mid_Cheak());
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

        switch(main_mode)                                                    // 根据主模式选择执行不同功能
        {
            case Mode_IDLE:                                                   // 空闲模式

                break;

            case Guandao_Recode_Mode:                             // 惯导记录模式
//                hotRC_control();                                                // RackTest禁用遥控器
                guandao_recode(&INS);                                   // 记录管道路径点
                break;

            case Guandao_portion_1:                                     // 管道部分1模式
                portion_1();                                                        // 执行第一部分路径跟踪
                break;

            case Guandao_Voice:                                             // 管道语音模式    待完善
                if(key1_flag == 1)
                {
                    key1_flag = 0;
                    port2_flag = 1;
                }
                portion2_points_trace(0 , 0 ,port2_flag );
                break;

            case Guandao_portion_3:                                     // 管道部分1模式
                guandao_trace(&INS);                                      // 执行第三部分路径跟踪
                break;

            case Rack_Test_Mode:
                Rack_Test_Run();
                break;

            default : break;

        }
        Guandao_Rear_Motor_Update();
//        ips200_show_float(X(1),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lat, 3,6);
//        ips200_show_float(X(11),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lon, 3,6);
//        ips200_show_float(X(1),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].cheak_flag, 3,6);
//        ips200_show_float(X(11),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].theta, 3,6);
//        ips200_show_float(X(11),  Y(10) ,INS.gps_recode_length, 3,6);
//        ips200_show_float(X(1),  Y(10) ,gnss.satellite_used, 3,6);

            if(main_mode != Rack_Test_Mode)
            {
                ips200_show_int(X(1),  Y(8),Speed_ecd.delta_l ,5);
                ips200_show_int(X(10),  Y(8),Speed_ecd.delta_r ,5);
//                ips200_show_int(X(10),  Y(9),MoterPID_R.out ,5);
//            Moter_Set(5000 ,5000  );
//            Speed_Control(10 , 10);


//                    ips200_show_int(X(10),  Y(10),angle ,5);
                    ips200_show_float(X(10),  Y(11),Yaw_1 ,5 ,5);
//                    ips200_show_int(X(10),  Y(12),gpio_get_level(SWITCH2) ,5);
                    ips200_show_int(X(10),  Y(12),angle_speed ,5);
            }
//                    ips200_show_int(X(10),  Y(13),conrtol_mode ,5);
//                    ips200_show_float(X(10),  Y(12),angle_speed ,5 ,5);
//                    VeerMoter_Set(10000);

//        hotRc_Show();
        if(0 && main_mode != Rack_Test_Mode && x6f_out[4] ==200)                                          // 禁用遥控急停，避免未接收机时清零输出
        {
            conrtol_mode =IDLE;
            Moter_Set(0 , 0 );
            VeerMoter_Set(0);

        }
//        if(key1_flag ==1)
//        {
//            key1_flag=0;
//        }
//        if(key3_flag == 1)
//        {
//            key3_flag = 0;
//            GPS_Work_SHOW();
//            ips200_clear();
//        }


        // 此处编写需要循环执行的代码
    }
}




#pragma section all restore
// **************************** 代码区域 ****************************
