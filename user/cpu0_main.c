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
#include <stdio.h>
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

extern int num;

// guandao.c 输出的 out_v_l/out_v_r 仍沿用旧工程的速度单位。
// 后轮新模块使用 m/s，所以这里集中做比例换算，方便后续统一调速度标定。
#define GUANDAO_SPEED_TO_MPS    (0.1f)
#define SERIAL_DEBUG_PERIOD_MS  (200)

// 记录菜单可以选择 INS/passage/portion_3/portion_2。
// 屏幕调试页必须显示当前正在记录的那条链路，否则会误以为 Len/X/Y 没变化。
static guandao_state *Get_Record_Display_State(void)
{
    switch(route_setting_choice)
    {
        case 0: return &INS;
        case 1: return &passage;
        case 2: return &portion_3;
        case 3: return &portion_2;
        default: return &INS;
    }
}

// 科目一自动驾驶和倒车模式最终都通过 rear_motor 模块驱动后轮。
// 这个函数只做一件事：把惯导规划速度换成 m/s，并调用后轮闭环。
// 非自动驾驶/非倒车/非测试模式下主动停后轮，避免退出模式后残留 PWM。
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

// Convert float data to a scaled integer before printing.
// This avoids relying on floating-point printf support in the embedded C library.
static int32 Serial_Debug_Scale(float value, float scale)
{
    return (int32)(value * scale);
}

// Output one line through the downloader/debug UART.
// Hardware path: TC264 UART0, TX=P14_0, RX=P14_1, 115200 baud, initialized by debug_init().
static void Serial_Debug_Write(const char *line)
{
    uart_write_string(DEBUG_UART_INDEX, line);
}

// Periodic serial diagnostics for subject-one record and autonomous trace modes.
// REC lines are used while pushing the car to record points.
// AUTO lines are used while the car is tracking the saved INS route.
static void Serial_Debug_Update(void)
{
    static uint32 last_ms = 0;
    uint32 now_ms = system_getval_ms();
    char line[220];
    int len;

    if(now_ms - last_ms < SERIAL_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_ms = now_ms;

    if(main_mode == Guandao_Recode_Mode)
    {
        guandao_state *record_state = Get_Record_Display_State();

        len = sprintf(line,
                      "REC,t=%lu,route=%d,len=%d,full=%d,thr100=%ld,x100=%ld,y100=%ld,th10=%ld,encL=%d,encR=%d,key1=%d,gps=%d,sat=%d,gflag=%d\r\n",
                      (unsigned long)now_ms,
                      route_setting_choice,
                      record_state->length_index,
                      (record_state->length_index >= MAX_LENGTH_INDEX),
                      (long)Serial_Debug_Scale(recode_threshold, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.x, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.y, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.theta, 10.0f),
                      guandao_ecd.delta_l,
                      guandao_ecd.delta_r,
                      gpio_get_level(KEY1),
                      gnss.state,
                      gnss.satellite_used,
                      gnss_flag);
        if(len > 0)
        {
            Serial_Debug_Write(line);
        }
    }
    else if(main_mode == Guandao_portion_1)
    {
        len = sprintf(line,
                      "AUTO,t=%lu,idx=%d,len=%d,D100=%ld,A10=%ld,DF100=%ld,reason=%d,x100=%ld,y100=%ld,yaw10=%ld,base10=%ld,vc10=%ld,avg10=%ld,vl10=%ld,vr10=%ld,servo10=%ld,tgt100=%ld,act100=%ld,pwm=%d,enc10=%d,enc100=%ld\r\n",
                      (unsigned long)now_ms,
                      INS.current_point_index,
                      INS.length_index,
                      (long)Serial_Debug_Scale(guandao_debug_distance, 100.0f),
                      (long)Serial_Debug_Scale(guandao_debug_angle_diff, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_dist_final, 100.0f),
                      guandao_debug_stop_reason,
                      (long)Serial_Debug_Scale(INS.current_state.x, 100.0f),
                      (long)Serial_Debug_Scale(INS.current_state.y, 100.0f),
                      (long)Serial_Debug_Scale(Yaw_1, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_base_speed, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_v_center, 10.0f),
                      (long)Serial_Debug_Scale((out_v_l + out_v_r) * 0.5f, 10.0f),
                      (long)Serial_Debug_Scale(out_v_l, 10.0f),
                      (long)Serial_Debug_Scale(out_v_r, 10.0f),
                      (long)Serial_Debug_Scale(out_servo, 10.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_target_mps(), 100.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_speed_mps(), 100.0f),
                      rear_motor_get_pwm(),
                      rear_motor_get_encoder_10ms(),
                      (long)rear_motor_get_encoder_100ms());
        if(len > 0)
        {
            Serial_Debug_Write(line);
        }
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

    Init_All();                         // 初始化屏幕、按键、蜂鸣器、编码器、电机、IMU、GPS、路径结构等外设
    rear_motor_init();                  // 初始化后轮独立速度闭环模块，后续科目一直接使用这一套 PID/PWM 输出
    // 前轮转向改为 TIM4 磁编码器闭环，不再初始化 SPI 绝对值编码器
//    hotRc_Control_init();                                                              // RackTest禁用遥控器，避免占用P33_6/P33_7编码器
   pit_ms_init(CCU61_CH1, 1);           // 1ms 周期任务：按键扫描、IMU 解算、旧速度控制入口
   pit_ms_init(CCU61_CH0, 1);           // 1ms 周期任务：转向电机控制、GPS 解析节拍、后轮编码器采样



    cpu_wait_event_ready();                                                          // 等待所有核心初始化完毕

    Flash_Main_Read();                                                                  // 上电后读取 PID、路线点、GPS 辅助点等 Flash 数据

    Menu_Contral();                                                                      // 菜单结束后 main_mode/conrtol_mode 已确定，主循环按模式执行

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

            case Guandao_Recode_Mode:                             // 惯导记录模式：推车时按后轮编码器自动累计 X/Y/Theta 和路线点
//                hotRC_control();                                                // RackTest禁用遥控器
                guandao_recode(&INS);                                   // 记录管道路径点
                break;

            case Guandao_portion_1:                                     // 科目一自动驾驶：读取已保存 INS 路线并追踪到停车点/终点
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
        Serial_Debug_Update();
//        ips200_show_float(X(1),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lat, 3,6);
//        ips200_show_float(X(11),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lon, 3,6);
//        ips200_show_float(X(1),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].cheak_flag, 3,6);
//        ips200_show_float(X(11),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].theta, 3,6);
//        ips200_show_float(X(11),  Y(10) ,INS.gps_recode_length, 3,6);
//        ips200_show_float(X(1),  Y(10) ,gnss.satellite_used, 3,6);

            // 记录模式单独显示“记录诊断页”：
            // Enc 不变说明编码器没进来；Enc 变但 X/Y/Len 不变，才继续查里程积分和记录阈值。
            if(main_mode == Guandao_Recode_Mode)
            {
                guandao_state *record_state = Get_Record_Display_State();
                ips200_show_string(X(1),  Y(8), "REC");      ips200_show_int(X(6),  Y(8), route_setting_choice, 2);
                ips200_show_string(X(10), Y(8), "Len");      ips200_show_int(X(15), Y(8), record_state->length_index, 4);
                ips200_show_string(X(1),  Y(9), "X");        ips200_show_float(X(4),  Y(9), record_state->current_state.x, 4, 2);
                ips200_show_string(X(12), Y(9), "Y");        ips200_show_float(X(15), Y(9), record_state->current_state.y, 4, 2);
                ips200_show_string(X(1),  Y(10), "Theta");   ips200_show_float(X(8),  Y(10), record_state->current_state.theta, 4, 1);
                ips200_show_string(X(1),  Y(11), "Enc");     ips200_show_int(X(6),  Y(11), guandao_ecd.delta_l, 5); ips200_show_int(X(14), Y(11), guandao_ecd.delta_r, 5);
                ips200_show_string(X(1),  Y(12), "KEY1");    ips200_show_int(X(7),  Y(12), gpio_get_level(KEY1), 1);
                ips200_show_string(X(10), Y(12), "GPS");     ips200_show_int(X(15), Y(12), gnss.state, 1);
                ips200_show_string(X(1),  Y(13), "Sat");     ips200_show_int(X(6),  Y(13), gnss.satellite_used, 3);
                ips200_show_string(X(10), Y(13), "GFlag");   ips200_show_int(X(17), Y(13), gnss_flag, 1);
                if(record_state->length_index >= MAX_LENGTH_INDEX)
                {
                    ips200_show_string(X(1),  Y(14), "Route FULL");
                }
            }
            // 自动驾驶诊断页：用于判断停车原因。
            // Idx 接近 Len 表示路线追完；TgtAct/PWM 为 0 表示后轮目标已被上层清掉。
            else if(main_mode != Rack_Test_Mode)
            {
                ips200_show_string(X(1),  Y(8), "Idx");      ips200_show_int(X(6),  Y(8), INS.current_point_index, 4);
                ips200_show_string(X(12), Y(8), "Len");      ips200_show_int(X(17), Y(8), INS.length_index, 4);
                ips200_show_string(X(1),  Y(9), "D");        ips200_show_float(X(6),  Y(9), guandao_debug_distance, 3, 2);
                ips200_show_string(X(12), Y(9), "A");        ips200_show_float(X(16), Y(9), guandao_debug_angle_diff, 3, 1);
                ips200_show_string(X(1),  Y(10), "Reason");  ips200_show_int(X(10), Y(10), guandao_debug_stop_reason, 2);
                ips200_show_string(X(1),  Y(11), "VlVr");    ips200_show_float(X(7),  Y(11), out_v_l, 3, 1); ips200_show_float(X(15), Y(11), out_v_r, 3, 1);
                ips200_show_string(X(1),  Y(12), "TgtAct");  ips200_show_float(X(9),  Y(12), rear_motor_get_target_mps(), 2, 1); ips200_show_float(X(16), Y(12), rear_motor_get_speed_mps(), 2, 1);
                ips200_show_string(X(1),  Y(13), "PWM");     ips200_show_int(X(7),  Y(13), rear_motor_get_pwm(), 5);
                ips200_show_string(X(1),  Y(14), "Yaw");     ips200_show_float(X(7),  Y(14), Yaw_1, 4, 1);
                ips200_show_string(X(1),  Y(15), "XY");      ips200_show_float(X(5),  Y(15), INS.current_state.x, 3, 1); ips200_show_float(X(13), Y(15), INS.current_state.y, 3, 1);
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
