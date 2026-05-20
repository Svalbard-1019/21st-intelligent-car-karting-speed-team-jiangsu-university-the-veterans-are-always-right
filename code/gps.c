/*
 * UTF-8 详细注释说明：GPS 辅助记录和自动驾驶校验模块。
 *
 * 模块职责：
 * 1. KEY2 触发时把当前 GNSS 经纬度、Yaw 和路线索引记录到 recode_gpsmap。
 * 2. 自动驾驶时根据当前位置到 GPS 辅助点的距离/方位进行远距离校验。
 * 3. 将记录的 GPS 点复制到 gps_work，用于运行时追踪和显示。
 *
 * 重要区别：
 * - 普通路线点记录主要靠后轮编码器，不靠 GPS。
 * - GPS 只做辅助校验，所以 Enc 没数据时，GPS 正常也不能生成普通路线里程。
 *
 * 调试重点：
 * - gnss.state=1 表示定位有效，satellite_used 表示卫星数。
 * - gnss_flag 表示串口收到完整数据等待解析，不等同于定位成功。
 */

/*
 * 主函数/科目一调用链：
 * 1. Init_All() 中调用 GPS_Init() 初始化 GNSS 串口；UART3 接收中断调用 gnss_uart_callback() 收数据。
 * 2. CCU61_CH0 中断检测 gnss_flag 后调用 gnss_data_parse()，Main_Key_Flag 打开后继续调用 update_gpsinformation() 更新经纬度缓存。
 * 3. core0_main() 在 Flash_Main_Read() 后，如果 GPS_WORK_FLAG 打开，会调用 GPS_WorkMap_Copy(&INS) 复制 GPS 辅助点。
 * 4. 记录模式可通过 recode_gps() 保存 GPS 校验点，自动驾驶时 trace_gps()/swtich_gps() 用于辅助判断路线段或显示调试。
 */


/*
 * gps.c
 *
 *  Created on: 2026年4月23日
 *      Author: 18905
 */

#include "zf_common_headfile.h"

GPS_work gps_work;
Lost_Point lost_judge;
/**
 * 函数说明：angle_plan()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_plan(float * angle)
{
    if(* angle >= 180){* angle -= 360 ;}
    else if(* angle < -180){* angle += 360 ;}

}

/**
 * 函数说明：recode_gps()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void recode_gps(guandao_state * state)
{
    static uint8 rcd_gps_flag = 0 ;
    switch(rcd_gps_flag)
    {
        case 0 :
            state->recode_gpsmap[state->gps_recode_length].lat = gnss.latitude;
            state->recode_gpsmap[state->gps_recode_length].lon = gnss.longitude;
            state->recode_gpsmap[state->gps_recode_length].theta = Yaw_1;
            state->recode_gpsmap[state->gps_recode_length].cheak_flag = state->length_index;
            state->gps_recode_length ++;
            rcd_gps_flag =1;
            break;
        case 1 :
            state->recode_gpsmap[state->gps_recode_length].lat = gnss.latitude;
            state->recode_gpsmap[state->gps_recode_length].lon = gnss.longitude;
            state->recode_gpsmap[state->gps_recode_length].theta =fabs(state->recode_gpsmap[state->gps_recode_length - 1].theta - Yaw_1) ;
            state->recode_gpsmap[state->gps_recode_length].cheak_flag = state->length_index;
            state->gps_recode_length ++;
            rcd_gps_flag =0;
            break;
    }

}

/**
 * 函数说明：swtich_gps()。处理 GPS 辅助点、GPS 显示或 GPS 校验逻辑。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 swtich_gps(void)
{
    static float loop_theta = 0;
    static uint8 loop_theta_flag = 1;
    if(gps_work.distance_gps <= GPS_SWITCH_DISTANCE && gps_work.gps_current_point%2 ==0)
    {
        loop_theta = Yaw_1;
        loop_theta_flag = 0;
        gps_work.gps_current_point++;
        gps_work.azimuth_gps = get_two_points_azimuth(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);
        angle_plan(&gps_work.azimuth_gps);
        gps_work.angle_gps = gps_work.azimuth_gps - Yaw_1;
        angle_plan(&gps_work.angle_gps);
        gps_work.distance_gps =   get_two_points_distance(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);

    }
    if(gps_work.distance_gps <= GPS_SWITCH_DISTANCE && gps_work.gps_current_point%2 !=0 &&loop_theta_flag == 1)
    {
        gps_work.gps_current_point++;
        gps_work.azimuth_gps = get_two_points_azimuth(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);
        angle_plan(&gps_work.azimuth_gps);
        gps_work.angle_gps = gps_work.azimuth_gps - Yaw_1;
        angle_plan(&gps_work.angle_gps);
        gps_work.distance_gps =   get_two_points_distance(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);
        return 1;
    }
    if(fabs(Yaw_1 - loop_theta) == gps_work.points[gps_work.gps_current_point].theta/2.0f && loop_theta_flag == 0)
    {
        loop_theta_flag = 1;
    }
    return 0 ;

}

/**
 * 函数说明：trace_gps()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void trace_gps(guandao_state * e)
{
    if( gps_work.gps_update_flag == 1)
    {
        gps_work.gps_update_flag = 0;
        if(swtich_gps())
        {
            if(e->recode_gpsmap[gps_work.gps_current_point - 1].cheak_flag == e->current_point_index)  //e->recode_gpsmap[gps_work.gps_current_point - 1].cheak_flag == e->current_point_index + 1
            {
                lost_judge = FIND;
            }
            else
            {
                lost_judge = LOST;
            }
            if(fabs(gps_work.angle_gps) >=90)
            {
                if(gps_work.gps_current_point%2 !=0)
                {
                    lost_judge = LOST;
                }
                gps_work.gps_current_point++;
            }
        }
    }

}
/**
 * 函数说明：update_gpsinformation()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void update_gpsinformation(void)
{
    if(gps_work.gps_current_point >= gps_work.work_gps_length)return;
    gps_work.azimuth_gps = get_two_points_azimuth(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);
    angle_plan(&gps_work.azimuth_gps);
    gps_work.angle_gps = gps_work.azimuth_gps - Yaw_1;
    angle_plan(&gps_work.angle_gps);
    gps_work.distance_gps =   get_two_points_distance(gnss.latitude,   gnss.longitude,  gps_work.points[gps_work.gps_current_point  ].lat , gps_work.points[gps_work.gps_current_point  ].lon);
    gps_work.gps_update_flag = 1;
}

/**
 * 函数说明：double_to_int32()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - y：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int32 double_to_int32(double y)
{
    int32 x = 0;
    x = (int32)(y*10000000);
    return x;
}
/**
 * 函数说明：int32_to_double()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - y：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 double 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
double int32_to_double(int32 y)
{
    double x = 0;
    x = (double)y * 1.0f / 10000000;
    return x;
}

/**
 * 函数说明：GPS_WorkMap_Copy()。处理 GPS 辅助点、GPS 显示或 GPS 校验逻辑。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void GPS_WorkMap_Copy(guandao_state * e)
{
    int choice_flag = 0;
    guandao_state * p = e;
    while(choice_flag < route_setting_choice)
    {
        p = p->next;
        if(p == NULL)return;
        choice_flag++;
    }
    if(p->gps_recode_length%2 != 0)Buzzer_check(500);

    gps_work.work_gps_length = p->gps_recode_length;
    for( int i = 0 ; i <p->gps_recode_length ; i++)gps_work.points[i].theta= 0.0f;
    for( int i = 0 ; i <p->gps_recode_length/2 ; i+=2)gps_work.points[i].theta= p->recode_gpsmap[i].theta;

    for( int i = 0 ; i <p->gps_recode_length ; i++)
    {
        gps_work.points[i].lat = p->recode_gpsmap[i].lat;
        gps_work.points[i].lon = p->recode_gpsmap[i].lon;
        gps_work.points[i].gps_cheak_flag = p->recode_gpsmap[i].cheak_flag;
    }


}


/**
 * 函数说明：GPS_Points_Show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void GPS_Points_Show(guandao_state * e)
{
    double Max_R_Line = -10000.0f , Max_D_Line = -10000.0f , Min_L_Line = 10000.0f , Min_U_Line = 10000.0f;
    double Center_H = 0 ,  Center_W = 0,  INDEX_H = 0,  INDEX_W = 0 ;
    uint16 GD_Show [2][e->gps_recode_length] ;
    if(e->gps_recode_length ==0)return;
    for( int i = 0 ; i< e->gps_recode_length ;i ++)
    {
        if(e ->recode_gpsmap[i].lat < Min_L_Line)Min_L_Line =e ->recode_gpsmap[i].lat  ;
        if(e ->recode_gpsmap[i].lat   > Max_R_Line)Max_R_Line =e ->recode_gpsmap[i].lat  ;
        if(e->recode_gpsmap[i].lon < Min_U_Line)Min_U_Line =e->recode_gpsmap[i].lon;
        if(e->recode_gpsmap[i].lon > Max_D_Line)Max_D_Line =e->recode_gpsmap[i].lon;
    }
    Center_W = (Max_R_Line + Min_L_Line)/2.0f;
    Center_H = (Max_D_Line + Min_U_Line)/2.0f;
    INDEX_W = (Max_R_Line - Min_L_Line);
    INDEX_H = (Max_D_Line - Min_U_Line);

    for(  int i = 0 ; i< e->gps_recode_length ; i ++ )
    {
        GD_Show[0][i] = 110.0f + (e ->recode_gpsmap[i].lat  - Center_W)*(200.0f/INDEX_W);
        GD_Show[1][i] = 150.0f - (e->recode_gpsmap[i].lon - Center_H)*(280.0f/INDEX_H);

    }


    for(int i = 0 ; i< e->gps_recode_length - 1  ; i ++ )
    {
        ips200_draw_point((uint16)GD_Show[0][i],(uint16)GD_Show[1][i],RGB565_WHITE);
        ips200_draw_line((uint16)GD_Show[0][i] ,(uint16)GD_Show[1][i] ,(uint16)GD_Show[0][i+1] ,(uint16)GD_Show[1][i+1] , RGB565_WHITE);
        system_delay_ms(20);
    }

}

/**
 * 函数说明：GPS_Work_SHOW()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：GPS 辅助记录和显示模块，不是科目一后轮驱动主链路，但用于路线校验和定位调试。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void GPS_Work_SHOW(void)
{
    double Max_R_Line = -10000.0f , Max_D_Line = -10000.0f , Min_L_Line = 10000.0f , Min_U_Line = 10000.0f;
    double Center_H = 0 ,  Center_W = 0,  INDEX_H = 0,  INDEX_W = 0 ;
    uint16 GD_Show [2][gps_work.work_gps_length] ;
    if(gps_work.work_gps_length ==0)return;
    for( int i = 0 ; i< gps_work.work_gps_length ;i ++)
    {
        if( gps_work.points[i].lat< Min_L_Line)Min_L_Line =gps_work.points[i].lat  ;
        if(gps_work.points[i].lat   > Max_R_Line)Max_R_Line =gps_work.points[i].lat  ;
        if(gps_work.points[i].lon < Min_U_Line)Min_U_Line =gps_work.points[i].lon;
        if(gps_work.points[i].lon > Max_D_Line)Max_D_Line =gps_work.points[i].lon;
    }
    Center_W = (Max_R_Line + Min_L_Line)/2.0f;
    Center_H = (Max_D_Line + Min_U_Line)/2.0f;
    INDEX_W = (Max_R_Line - Min_L_Line);
    INDEX_H = (Max_D_Line - Min_U_Line);

    for(  int i = 0 ; i< gps_work.work_gps_length ; i ++ )
    {
        GD_Show[0][i] = 110.0f + (gps_work.points[i].lat  - Center_W)*(200.0f/INDEX_W);
        GD_Show[1][i] = 150.0f - (gps_work.points[i].lon - Center_H)*(280.0f/INDEX_H);

    }


    for(int i = 0 ; i<gps_work.work_gps_length - 1  ; i ++ )
    {
        ips200_draw_point((uint16)GD_Show[0][i],(uint16)GD_Show[1][i],RGB565_WHITE);
        ips200_draw_line((uint16)GD_Show[0][i] ,(uint16)GD_Show[1][i] ,(uint16)GD_Show[0][i+1] ,(uint16)GD_Show[1][i+1] , RGB565_WHITE);
        system_delay_ms(50);
    }


}
