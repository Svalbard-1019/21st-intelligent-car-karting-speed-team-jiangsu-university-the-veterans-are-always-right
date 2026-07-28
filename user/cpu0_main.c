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
#define DISPLAY_DEBUG_PERIOD_MS (200)
#define SYSTEM_MS_WRAP          (42950u)

static uint32 main_loop_last_ms = 0;
static uint32 main_loop_last_dt_ms = 0;
static uint32 main_loop_max_dt_ms = 0;
static char serial_debug_tx_buffer[1024];
static uint16 serial_debug_tx_length = 0;
static uint16 serial_debug_tx_index = 0;
static uint32 serial_debug_tx_dropped = 0;

static uint32 Main_Elapsed_Ms(uint32 now_ms, uint32 start_ms)
{
    if(now_ms >= start_ms) return now_ms - start_ms;
    return (SYSTEM_MS_WRAP - start_ms) + now_ms;
}

static void Main_Loop_Timing_Update(void)
{
    uint32 now_ms = system_getval_ms();

    if(main_loop_last_ms != 0)
    {
        main_loop_last_dt_ms = Main_Elapsed_Ms(now_ms, main_loop_last_ms);
        if(main_loop_last_dt_ms > main_loop_max_dt_ms)
        {
            main_loop_max_dt_ms = main_loop_last_dt_ms;
        }
    }
    main_loop_last_ms = now_ms;
}

// GNSS UART interrupt only assembles received bytes and raises gnss_flag.
// Parsing runs here at low priority so it cannot block steering, odometry or
// the UART2 SBUS receiver interrupt.
static void GPS_Main_Loop_Update(void)
{
    static uint32 last_gps_ms = 0;
    uint32 now_ms;
    uint32 interrupt_state;
    uint8 parse_pending = 0;

    if(!GPS_WORK_FLAG) return;

    now_ms = system_getval_ms();
    if(last_gps_ms != 0 && Main_Elapsed_Ms(now_ms, last_gps_ms) < 100u) return;
    last_gps_ms = now_ms;

    interrupt_state = interrupt_global_disable();
    if(gnss_flag)
    {
        gnss_flag = 0;
        parse_pending = 1;
    }
    interrupt_global_enable(interrupt_state);

    if(!parse_pending) return;

    gnss_data_parse();
    if(Main_Key_Flag) update_gpsinformation();
}

static uint8 Main_Display_Update_Due(void)
{
    static uint32 last_display_ms = 0;
    uint32 now_ms = system_getval_ms();

    if(last_display_ms == 0)
    {
        last_display_ms = now_ms;
        return 1;
    }
    if(Main_Elapsed_Ms(now_ms, last_display_ms) < DISPLAY_DEBUG_PERIOD_MS)
    {
        return 0;
    }
    last_display_ms = now_ms;
    return 1;
}

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

    /* Keep remote recording on the proven KMY response.  Only the
     * autonomous subject-three return selects the KMS rear profile. */
    rear_motor_select_route((conrtol_mode == GUANDAO
            && route_setting_choice == 2u) ? 2u : 0u);

    /* An explicit route/save/parking stop owns the rear motor until
     * its nonblocking brake sequence finishes. Remote neutral never
     * starts this state, so manual control keeps its original behavior. */
    if(rear_motor_brake_active())
    {
        rear_motor_brake_update();
        return;
    }

    if(conrtol_mode == GUANDAO)
    {
        target_mps = (out_v_l + out_v_r) * 0.5f * GUANDAO_SPEED_TO_MPS;
    }
    else if(conrtol_mode == DAOCHE)
    {
        target_mps = daoche_speed * GUANDAO_SPEED_TO_MPS;
    }
    else if(conrtol_mode == YAOKONG)
    {
        target_mps = hot_rc_speed * GUANDAO_SPEED_TO_MPS;
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
    uint16 length = 0;

    if(serial_debug_tx_index < serial_debug_tx_length)
    {
        serial_debug_tx_dropped++;
        return;
    }
    while(line[length] != '\0' && length < (uint16)(sizeof(serial_debug_tx_buffer) - 1u))
    {
        serial_debug_tx_buffer[length] = line[length];
        length++;
    }
    serial_debug_tx_length = length;
    serial_debug_tx_index = 0;
}

/* 每轮只向UART FIFO提交一个字节，避免格式化诊断行阻塞遥控、追点和转向。
 * 直接使用固定ASCLIN模块地址，避开曾在KMS Start路径触发Bus Error的失效句柄。 */
static void Serial_Debug_Service(void)
{
    Ifx_ASCLIN *asclin;

    if(serial_debug_tx_index >= serial_debug_tx_length) return;
    asclin = IfxAsclin_getAddress((IfxAsclin_Index)DEBUG_UART_INDEX);
    if(asclin == NULL)
    {
        serial_debug_tx_index = 0;
        serial_debug_tx_length = 0;
        serial_debug_tx_dropped++;
        return;
    }
    if(IfxAsclin_getTxFifoFillLevel(asclin) != 0u) return;

    asclin->TXDATA.U = (uint32)(uint8)serial_debug_tx_buffer[serial_debug_tx_index];
    serial_debug_tx_index++;
    if(serial_debug_tx_index >= serial_debug_tx_length)
    {
        serial_debug_tx_index = 0;
        serial_debug_tx_length = 0;
    }
}

// Periodic serial diagnostics for subject-one record and autonomous trace modes.
// REC lines are used while pushing the car to record points.
// AUTO lines are used while the car is tracking the saved INS route.
static void Serial_Debug_Update(void)
{
    static uint32 last_ms = 0;
    uint32 now_ms = system_getval_ms();
    static char line[1024];
    static uint8 p3_diag_started = 0;
    static uint8 p3_gps_origin_valid = 0;
    static int32 p3_pulse_origin = 0;
    static double p3_lat_origin = 0.0;
    static double p3_lon_origin = 0.0;
    int len;

    if(last_ms != 0 && Main_Elapsed_Ms(now_ms, last_ms) < SERIAL_DEBUG_PERIOD_MS)
    {
        return;
    }
    last_ms = now_ms;

    if(main_mode != Guandao_portion_3)
    {
        p3_diag_started = 0;
        p3_gps_origin_valid = 0;
    }

    if(main_mode == Guandao_Recode_Mode)
    {
        guandao_state *record_state = Get_Record_Display_State();

        len = sprintf(line,
                      "REC,t=%lu,route=%d,len=%d,gpslen=%d,full=%d,thr100=%ld,x100=%ld,y100=%ld,th10=%ld,lastx100=%ld,lasty100=%ld,lastth10=%ld,encL=%d,encR=%d,key1=%d,key4=%d,ch1=%d,ch2=%d,ch3=%d,ch4=%d,gps=%d,sat=%d,gflag=%d,parkS=%d,parkT=%d,parkE=%d\r\n",
                      (unsigned long)now_ms,
                      route_setting_choice,
                      record_state->length_index,
                      record_state->gps_recode_length,
                      (record_state->length_index >= MAX_LENGTH_INDEX),
                      (long)Serial_Debug_Scale(recode_threshold, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.x, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.y, 100.0f),
                      (long)Serial_Debug_Scale(record_state->current_state.theta, 10.0f),
                      (long)Serial_Debug_Scale((record_state->length_index > 0) ? record_state->recode_map[record_state->length_index - 1].x : 0.0f, 100.0f),
                      (long)Serial_Debug_Scale((record_state->length_index > 0) ? record_state->recode_map[record_state->length_index - 1].y : 0.0f, 100.0f),
                      (long)Serial_Debug_Scale((record_state->length_index > 0) ? record_state->recode_map[record_state->length_index - 1].theta : 0.0f, 10.0f),
                      guandao_ecd.delta_l,
                      guandao_ecd.delta_r,
                      gpio_get_level(KEY1),
                      gpio_get_level(KEY4),
                      x6f_out[0],
                      x6f_out[1],
                      x6f_out[2],
                      x6f_out[3],
                      gnss.state,
                      gnss.satellite_used,
                      gnss_flag,
                      daoche_point_length,
                      daoche_target_flag,
                      daoche_target_length);
        if(len > 0)
        {
            Serial_Debug_Write(line);
        }
    }
    else if(main_mode == Guandao_portion_1)
    {
        len = sprintf(line,
                      "AUTO,cfg=p1spd8,t=%lu,dt=%lu,dtMax=%lu,base10=%ld,vl10=%ld,vr10=%ld,enc10=%d,pend=%u,merge=%lu,odom=%ld,enc100=%ld,idx=%d,len=%d,reason=%d,x100=%ld,y100=%ld,yaw10=%ld,rawS10=%ld,finS10=%ld,actS10=%ld,tgt100=%ld,act100=%ld,pwm=%d,pwmReq=%d,turn10=%ld,turnLv=%u,req100=%ld,cmd100=%ld,app=%d,elong100=%ld,elat100=%ld,eyaw10=%ld,revSt=%u,revPlan=%u,revIdx=%d,revLen=%d,revD100=%ld,revYaw10=%ld,revCmd10=%ld,revLd100=%ld,brk=%u,brkP=%d,brkR=%u\r\n",
                      (unsigned long)now_ms,
                      (unsigned long)main_loop_last_dt_ms,
                      (unsigned long)main_loop_max_dt_ms,
                      (long)Serial_Debug_Scale(base_speed, 10.0f),
                      (long)Serial_Debug_Scale(out_v_l, 10.0f),
                      (long)Serial_Debug_Scale(out_v_r, 10.0f),
                      rear_motor_get_encoder_10ms(),
                      (unsigned int)rear_motor_get_odometry_pending_samples(),
                      (unsigned long)rear_motor_get_odometry_merged_samples(),
                      (long)rear_motor_get_odometry_total_pulses(),
                      (long)rear_motor_get_encoder_100ms(),
                      INS.current_point_index,
                      INS.length_index,
                      guandao_debug_stop_reason,
                      (long)Serial_Debug_Scale(INS.current_state.x, 100.0f),
                      (long)Serial_Debug_Scale(INS.current_state.y, 100.0f),
                      (long)Serial_Debug_Scale(Yaw_1, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_steer_raw, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_steer_final, 10.0f),
                      (long)Serial_Debug_Scale((float)angle, 10.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_target_mps(), 100.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_speed_mps(), 100.0f),
                      rear_motor_get_pwm(),
                      rear_motor_get_requested_pwm(),
                      (long)Serial_Debug_Scale(guandao_debug_upcoming_turn, 10.0f),
                      (unsigned int)guandao_debug_turn_level,
                      (long)Serial_Debug_Scale(guandao_debug_speed_requested, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_speed_command, 10.0f),
                      guandao_debug_approach_active,
                      (long)Serial_Debug_Scale(guandao_debug_entry_long, 100.0f),
                      (long)Serial_Debug_Scale(guandao_debug_entry_lat, 100.0f),
                      (long)Serial_Debug_Scale(guandao_debug_entry_yaw, 10.0f),
                      (unsigned int)guandao_reverse_debug_state(),
                      (unsigned int)guandao_reverse_debug_plan_ready(),
                      guandao_reverse_debug_route_index(),
                      guandao_reverse_debug_route_count(),
                      (long)Serial_Debug_Scale(
                              guandao_reverse_debug_target_distance(), 100.0f),
                      (long)Serial_Debug_Scale(
                              guandao_reverse_debug_target_yaw_error(), 10.0f),
                      (long)Serial_Debug_Scale(
                              guandao_reverse_debug_steer_command(), 10.0f),
                      (long)Serial_Debug_Scale(
                              guandao_reverse_debug_lookahead(), 100.0f),
                      (unsigned int)rear_motor_brake_active(),
                      rear_motor_brake_pwm(),
                      (unsigned int)rear_motor_brake_reason());
        if(len > 0)
        {
            Serial_Debug_Write(line);
        }
    }
    else if(main_mode == Guandao_portion_3)
    {
        int target_index = portion_3.current_point_index;
        state_t target_point;
        float dx;
        float dy;
        float target_angle;
        float angle_error;
        float gps_east_m = 0.0f;
        float gps_north_m = 0.0f;
        int32 pulse_relative;

        if(!p3_diag_started)
        {
            p3_pulse_origin = rear_motor_get_odometry_total_pulses();
            p3_diag_started = 1;
        }
        if(!p3_gps_origin_valid && gnss.state == 1)
        {
            p3_lat_origin = gnss.latitude;
            p3_lon_origin = gnss.longitude;
            p3_gps_origin_valid = 1;
        }
        if(p3_gps_origin_valid && gnss.state == 1)
        {
            gps_north_m = (float)((gnss.latitude - p3_lat_origin) * 111319.5);
            gps_east_m = (float)((gnss.longitude - p3_lon_origin) * 111319.5
                    * cos(p3_lat_origin * M_PI / 180.0));
        }
        pulse_relative = rear_motor_get_odometry_total_pulses() - p3_pulse_origin;

        if(target_index < 0) target_index = 0;
        if(target_index >= portion_3.length_index && portion_3.length_index > 0)
        {
            target_index = portion_3.length_index - 1;
        }
        if(portion_3.length_index > 0)
        {
            target_point = portion_3.recode_map[target_index];
        }
        else
        {
            target_point.x = 0.0f;
            target_point.y = 0.0f;
            target_point.theta = 0.0f;
        }
        dx = target_point.x - portion_3.current_state.x;
        dy = target_point.y - portion_3.current_state.y;
        target_angle = atan2f(dx, dy) / M_PI * 180.0f;
        angle_error = target_angle - portion_3.current_state.theta;
        angle_plan(&angle_error);

        len = sprintf(line,
                      "P3AUTO,cfg=p3track3,t=%lu,dt=%lu,dtMax=%lu,base10=%ld,p3Init=%u,gzRaw=%d,gzOff=%ld,txDrop=%lu,odomMerge=%lu,pRel=%ld,pend=%u,gOrg=%u,gE100=%ld,gN100=%ld,gps=%u,sat=%u,idx=%d,len=%d,gpslen=%d,D100=%ld,A10=%ld,reason=%d,x100=%ld,y100=%ld,yaw10=%ld,tx100=%ld,ty100=%ld,tth10=%ld,dx100=%ld,dy100=%ld,tang10=%ld,err10=%ld,vl10=%ld,vr10=%ld,servo10=%ld,steerAct10=%ld,tgt100=%ld,act100=%ld,pwm=%d,turn10=%ld,turnLv=%u,req100=%ld,cmd100=%ld,enc10=%d,enc100=%ld,brk=%u,brkP=%d,brkR=%u\r\n",
                      (unsigned long)now_ms,
                      (unsigned long)main_loop_last_dt_ms,
                      (unsigned long)main_loop_max_dt_ms,
                      (long)Serial_Debug_Scale(base_speed, 10.0f),
                      (unsigned int)IMU_yaw_rezero_active(),
                      (int)IMU_yaw_rezero_raw_z(),
                      (long)Serial_Debug_Scale(IMU_yaw_rezero_offset_z(), 1.0f),
                      (unsigned long)serial_debug_tx_dropped,
                      (unsigned long)rear_motor_get_odometry_merged_samples(),
                      (long)pulse_relative,
                      (unsigned int)rear_motor_get_odometry_pending_samples(),
                      (unsigned int)p3_gps_origin_valid,
                      (long)Serial_Debug_Scale(gps_east_m, 100.0f),
                      (long)Serial_Debug_Scale(gps_north_m, 100.0f),
                      (unsigned int)gnss.state,
                      (unsigned int)gnss.satellite_used,
                      portion_3.current_point_index,
                      portion_3.length_index,
                      portion_3.gps_recode_length,
                      (long)Serial_Debug_Scale(guandao_debug_distance, 100.0f),
                      (long)Serial_Debug_Scale(guandao_debug_angle_diff, 10.0f),
                      guandao_debug_stop_reason,
                      (long)Serial_Debug_Scale(portion_3.current_state.x, 100.0f),
                      (long)Serial_Debug_Scale(portion_3.current_state.y, 100.0f),
                      (long)Serial_Debug_Scale(Yaw_1, 10.0f),
                      (long)Serial_Debug_Scale(target_point.x, 100.0f),
                      (long)Serial_Debug_Scale(target_point.y, 100.0f),
                      (long)Serial_Debug_Scale(target_point.theta, 10.0f),
                      (long)Serial_Debug_Scale(dx, 100.0f),
                      (long)Serial_Debug_Scale(dy, 100.0f),
                      (long)Serial_Debug_Scale(target_angle, 10.0f),
                      (long)Serial_Debug_Scale(angle_error, 10.0f),
                      (long)Serial_Debug_Scale(out_v_l, 10.0f),
                      (long)Serial_Debug_Scale(out_v_r, 10.0f),
                      (long)Serial_Debug_Scale(out_servo, 10.0f),
                      (long)Serial_Debug_Scale(angle_control_get_current_angle_float(), 10.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_target_mps(), 100.0f),
                      (long)Serial_Debug_Scale(rear_motor_get_speed_mps(), 100.0f),
                      rear_motor_get_pwm(),
                      (long)Serial_Debug_Scale(guandao_debug_upcoming_turn, 10.0f),
                      (unsigned int)guandao_debug_turn_level,
                      (long)Serial_Debug_Scale(guandao_debug_speed_requested, 10.0f),
                      (long)Serial_Debug_Scale(guandao_debug_speed_command, 10.0f),
                      rear_motor_get_encoder_10ms(),
                      (long)rear_motor_get_encoder_100ms(),
                      (unsigned int)rear_motor_brake_active(),
                      rear_motor_brake_pwm(),
                      (unsigned int)rear_motor_brake_reason());
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
    uart_receiver_init();               // SBUS receiver: UART2, TX placeholder P10_5, RX P10_6, 100000 baud
    // 前轮转向改为 TIM4 磁编码器闭环，不再初始化 SPI 绝对值编码器
//    hotRc_Control_init();                                                              // RackTest禁用遥控器，避免占用P33_6/P33_7编码器
   pit_ms_init(CCU61_CH1, 1);           // 1ms 周期任务：按键扫描、IMU 解算、旧速度控制入口
   pit_ms_init(CCU61_CH0, 1);           // 1ms 周期任务：转向电机控制、GPS 解析节拍、后轮编码器采样



    cpu_wait_event_ready();                                                          // 等待所有核心初始化完毕

    Flash_Main_Read();                                                                  // 上电后读取 PID、路线点、GPS 辅助点等 Flash 数据

    Menu_Contral();                                                                      // 菜单结束后 main_mode/conrtol_mode 已确定，主循环按模式执行

    /* Parameter editing persists explicitly; Start must not rewrite Flash. */
    if(main_mode != Guandao_Recode_Mode
            && main_mode != Guandao_portion_3
            && GPS_WORK_FLAG)
    {
        GPS_WorkMap_Copy(&INS);
    }
//    Buzzer_check(500);
    Main_Key_Flag = 1;                                                            // 中断控制开始标志为1，主循环和中断控制同时启动
//    build_map_text(&INS);
//    while(Steer_Mid_Cheak());
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

        Main_Loop_Timing_Update();
        sbus_rc_control();

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
        GPS_Main_Loop_Update();
        Serial_Debug_Update();
        Serial_Debug_Service();
        uint8 display_update_due = Main_Display_Update_Due();
//        ips200_show_float(X(1),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lat, 3,6);
//        ips200_show_float(X(11),  Y(8) ,INS.recode_gpsmap[INS.gps_recode_length -1].lon, 3,6);
//        ips200_show_float(X(1),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].cheak_flag, 3,6);
//        ips200_show_float(X(11),  Y(9) ,INS.recode_gpsmap[INS.gps_recode_length -1].theta, 3,6);
//        ips200_show_float(X(11),  Y(10) ,INS.gps_recode_length, 3,6);
//        ips200_show_float(X(1),  Y(10) ,gnss.satellite_used, 3,6);

            // 记录模式单独显示“记录诊断页”：
            // Enc 不变说明编码器没进来；Enc 变但 X/Y/Len 不变，才继续查里程积分和记录阈值。
            if(display_update_due && main_mode == Guandao_Recode_Mode)
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
                ips200_show_string(X(1),  Y(14), "ParkS");   ips200_show_int(X(8),  Y(14), daoche_point_length, 4);
                ips200_show_string(X(13), Y(14), "T");       ips200_show_int(X(16), Y(14), daoche_target_flag, 1);
                if(record_state->length_index >= MAX_LENGTH_INDEX)
                {
                    ips200_show_string(X(1),  Y(15), "Route FULL");
                }
            }
            // 自动驾驶诊断页：用于判断停车原因。
            // Idx 接近 Len 表示路线追完；TgtAct/PWM 为 0 表示后轮目标已被上层清掉。
            else if(display_update_due && main_mode == Guandao_portion_1)
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
