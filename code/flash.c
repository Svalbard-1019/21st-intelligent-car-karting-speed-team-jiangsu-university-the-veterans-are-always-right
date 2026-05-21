/*
 * UTF-8 详细注释说明：Flash 参数与路线存取模块。
 *
 * 模块职责：
 * 1. 保存/读取 PID 参数、记录路线点、GPS 辅助点和停车点标记。
 * 2. 把 guandao_state 中的浮点/整数数据打包到 flash_union_buffer。
 * 3. 上电时通过 Flash_Main_Read() 恢复路线和参数。
 *
 * 科目一相关：
 * - INS 路线点存在 RECODE_MAP_POINTS_INDEX 页。
 * - daoche_point_length 用作科目一停车点，不设置时跑完整 INS 路线。
 * - GPS 辅助点只在 GPS_WORK_FLAG 打开时参与读写。
 *
 * 调试重点：
 * - 烧录程序通常不会清 Flash 路线，除非芯片擦除策略包含数据 Flash。
 * - 如果 Len 上电后为 0，优先查保存是否触发、Flash 页号是否冲突。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 上电初始化后调用 Flash_Main_Read()，恢复 PID 参数、INS 路线、passage 路线、portion_3 路线和 GPS 校验点。
 * 2. 记录模式 guandao_recode() 里长按 KEY1 会触发 Flash_Store_Mode(route_setting_choice)，把当前选择的路线写入对应 Flash 页。
 * 3. 科目一自动驾驶 portion_1() 使用的 INS.length_index 和 INS.map[] 来自 Flash_Read_INSpoints() 恢复的数据。
 * 4. 如果重新烧录但没有擦除对应 Flash 页，路线点通常还在；如果换工程或擦 Flash，则需要重新记录。
 */


/*
 * flash.c
 *
 *  Created on: 2025年11月23日
 *      Author: 18905
 */
#include "zf_common_headfile.h"

float speed_pid[6]={0.5f, 1.0f, 0.0f, 0.4f, 0.4f, 3.0f};
int16 control[5] = {10, -10, 2, 0, 0};
float kp;
float ki;
float kd;

#define FLASH_RECODE_THRESHOLD_DEFAULT   (0.4f)
#define FLASH_RECODE_THRESHOLD_MIN       (0.05f)
#define FLASH_RECODE_THRESHOLD_MAX       (2.0f)
#define FLASH_PURSUIT_THRESHOLD_DEFAULT  (0.4f)
#define FLASH_PURSUIT_THRESHOLD_MIN      (0.05f)
#define FLASH_PURSUIT_THRESHOLD_MAX      (3.0f)
#define FLASH_FINAL_DSTS_DEFAULT         (3.0f)
#define FLASH_FINAL_DSTS_MIN             (0.3f)
#define FLASH_FINAL_DSTS_MAX             (20.0f)

static int16 flash_clamp_route_length(int16 length)
{
    if(length < 0) return 0;
    if(length > MAX_LENGTH_INDEX) return MAX_LENGTH_INDEX;
    return length;
}

static float flash_sanitize_float(float value, float fallback, float min_value, float max_value)
{
    if(!(value >= min_value && value <= max_value))
    {
        return fallback;
    }
    return value;
}

static void flash_sanitize_runtime_params(void)
{
    speed_pid[3] = flash_sanitize_float(speed_pid[3],
                                        FLASH_RECODE_THRESHOLD_DEFAULT,
                                        FLASH_RECODE_THRESHOLD_MIN,
                                        FLASH_RECODE_THRESHOLD_MAX);
    speed_pid[4] = flash_sanitize_float(speed_pid[4],
                                        FLASH_PURSUIT_THRESHOLD_DEFAULT,
                                        FLASH_PURSUIT_THRESHOLD_MIN,
                                        FLASH_PURSUIT_THRESHOLD_MAX);
    speed_pid[5] = flash_sanitize_float(speed_pid[5],
                                        FLASH_FINAL_DSTS_DEFAULT,
                                        FLASH_FINAL_DSTS_MIN,
                                        FLASH_FINAL_DSTS_MAX);
}


/**
 * 函数说明：Flash_Read_pid()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_pid(void)
{
    if(flash_check(FLASH_SECTION_INDEX,SPEED_PID_PAGE_INDEX))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, SPEED_PID_PAGE_INDEX);
        for(uint8 i = 0  ; i<6 ; i++)
        {
            speed_pid[i] = flash_union_buffer[i].float_type;
        }
        flash_sanitize_runtime_params();
        MoterPID_L.Kp = speed_pid[0];
        MoterPID_R.Kp = speed_pid[0];
        MoterPID_L.Ki = speed_pid[1];
        MoterPID_R.Ki = speed_pid[1];
        MoterPID_L.Kd = speed_pid[2];
        MoterPID_R.Kd = speed_pid[2];
        recode_threshold = speed_pid[3];
        persuit_threshold = speed_pid[4];
        final_dsts = speed_pid[5];
        for(uint8 i = 12  ,j =0; i<16 ; i++ ,j++)
        {
            control[j] = flash_union_buffer[i].int16_type;
        }
        base_speed = (float)control[0];
        daoche_speed = (float)control[1];
        preview_spets = control[2];
    }


}

/**
 * 函数说明：Flash_Write_pid()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_pid(void)
{
    flash_buffer_clear();
    flash_sanitize_runtime_params();

    MoterPID_L.Kp = speed_pid[0];
    MoterPID_R.Kp = speed_pid[0];
    MoterPID_L.Ki = speed_pid[1];
    MoterPID_R.Ki = speed_pid[1];
    MoterPID_L.Kd = speed_pid[2];
    MoterPID_R.Kd = speed_pid[2];
    recode_threshold = speed_pid[3];
    persuit_threshold = speed_pid[4];
    final_dsts = speed_pid[5];
    base_speed = (float)control[0];
    daoche_speed = (float)control[1];
    preview_spets = control[2];

    for(uint8 i = 0  ; i<6 ; i++)
    {
        flash_union_buffer[i].float_type = speed_pid[i];
    }
    for(uint8 i = 12 ,j=0 ; i<16 ; i++ ,j++)
    {
        flash_union_buffer[i].int16_type =control[j] ;
    }
      if(flash_check(FLASH_SECTION_INDEX,SPEED_PID_PAGE_INDEX))
      {
          flash_erase_page(FLASH_SECTION_INDEX,SPEED_PID_PAGE_INDEX) ;
      }
      flash_write_page_from_buffer(FLASH_SECTION_INDEX,SPEED_PID_PAGE_INDEX);

}
/**
 * 函数说明：Flash_Store_Mode()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - route_choice：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Store_Mode(uint8 route_choice)
{
    switch(route_choice)
    {
        case 0:
            Key_Recode_Point(&INS); Flash_Write_INSpoints();
            break;
        case 1:
            if(portion2_points_build()){ Key_Recode_Point(&passage); Flash_Write_passage_points();}
            break;
        case 2:
            Key_Recode_Point(&portion_3);
            if(portion3_points_switch()){  Flash_Write_portion_3points();}
            break;
        default :break;
        if(GPS_WORK_FLAG)Flash_Write_gpscheak();
    }


}
/**
 * 函数说明：Flash_Main_Read()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Main_Read(void)
{
    Flash_Read_pid();
    Flash_Read_INSpoints();
    Flash_Read_passage_points();
    Flash_Read_portion_3points();
    Flash_Read_gpscheak();


}
/**
 * 函数说明：Flash_Write_passage_points()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_passage_points(void)
{
    int max_storage = 2 * passage.length_index +2 ;
    if (max_storage >=1020) max_storage = 1020;
    flash_buffer_clear();

    flash_union_buffer[0].int16_type = passage.length_index;
    for(int i = 2 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = passage.recode_map[j].x;
    }
    for(int i = 3 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = passage.recode_map[j].y;
    }


    if(flash_check(FLASH_SECTION_INDEX,RECODE_PASSAGE))
    {
        flash_erase_page(FLASH_SECTION_INDEX,RECODE_PASSAGE) ;
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX,RECODE_PASSAGE);
}
/**
 * 函数说明：Flash_Read_passage_points()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_passage_points(void)
{
    int get_max_storage = 0;
    if(flash_check(FLASH_SECTION_INDEX,RECODE_PASSAGE))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_PASSAGE);
        passage.length_index = flash_union_buffer[0].int16_type;
        get_max_storage =2 * passage.length_index +2;
        for(int i = 2 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            passage.recode_map[j].x = flash_union_buffer[i].float_type;
        }
        for(int i = 3 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            passage.recode_map[j].y = flash_union_buffer[i].float_type;
        }
    }
}

/**
 * 函数说明：Flash_Write_portion_3points()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_portion_3points(void)
{
    int max_storage = 2 * portion_3.length_index +2 ;
    if (max_storage >=1020) max_storage = 1020;
    flash_buffer_clear();

    flash_union_buffer[0].int16_type = portion_3.length_index;
    for(int i = 2 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = portion_3.recode_map[j].x;
    }
    for(int i = 3 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = portion_3.recode_map[j].y;
    }


    if(flash_check(FLASH_SECTION_INDEX,RECODE_PORTION_THREE))
    {
        flash_erase_page(FLASH_SECTION_INDEX,RECODE_PORTION_THREE) ;
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX,RECODE_PORTION_THREE);

}
/**
 * 函数说明：Flash_Read_portion_3points()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_portion_3points(void)
{
    int get_max_storage = 0;
    if(flash_check(FLASH_SECTION_INDEX,RECODE_PORTION_THREE))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_PORTION_THREE);
        portion_3.length_index = flash_union_buffer[0].int16_type -1 ;
        get_max_storage =2 * portion_3.length_index +2;
        for(int i = 2 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            portion_3.recode_map[j].x = flash_union_buffer[i].float_type;
        }
        for(int i = 3 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            portion_3.recode_map[j].y = flash_union_buffer[i].float_type;
        }
    }
}

/**
 * 函数说明：Flash_Write_INSpoints()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_INSpoints(void)
{

    int16 route_length = flash_clamp_route_length(INS.length_index);
    int16 stop_length = flash_clamp_route_length(daoche_point_length);
    if(stop_length > route_length) stop_length = route_length;
    int16 gps_length = INS.gps_recode_length;
    if(gps_length < 0) gps_length = 0;
    if(gps_length > MAX_GPS_RECODE) gps_length = MAX_GPS_RECODE;

    INS.length_index = route_length;
    daoche_point_length = stop_length;
    INS.gps_recode_length = gps_length;

    int max_storage = 2 * route_length +2 ;
    int gps_max_storage = max_storage + gps_length*2 + 2 ;
    flash_buffer_clear();

    flash_union_buffer[0].int16_type = route_length;
    if(daoche_flash_cheack)flash_union_buffer[1].int16_type = stop_length;
    else{flash_union_buffer[1].int16_type = route_length;}
    for(int i = 2 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = INS.recode_map[j].x;
    }
    for(int i = 3 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = INS.recode_map[j].y;
    }
    flash_union_buffer[max_storage].int16_type = gps_length;
    for(int i = max_storage+2 , j = 0; i < gps_max_storage ; i+=2 , j++)
    {
        flash_union_buffer[i].int32_type = double_to_int32(INS.recode_gpsmap[j].lat);
    }
    for(int i = max_storage+3 , j = 0; i < gps_max_storage ; i+=2 , j++)
    {
        flash_union_buffer[i].int32_type = double_to_int32(INS.recode_gpsmap[j].lon);
    }


    if(flash_check(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX))
    {
        flash_erase_page(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX) ;
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX);

}
/**
 * 函数说明：Flash_Read_INSpoints()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_INSpoints(void)
{
    int get_max_storage = 0;
    if(flash_check(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_MAP_POINTS_INDEX);
        INS.length_index = flash_clamp_route_length(flash_union_buffer[0].int16_type);
        daoche_point_length = flash_clamp_route_length(flash_union_buffer[1].int16_type);
        if(daoche_point_length > INS.length_index) daoche_point_length = INS.length_index;
        get_max_storage =2 * INS.length_index +2;
        for(int i = 2 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            INS.recode_map[j].x = flash_union_buffer[i].float_type;
        }
        for(int i = 3 , j = 0;i < get_max_storage ; i += 2 , j++)
        {
            INS.recode_map[j].y = flash_union_buffer[i].float_type;
        }
        INS.gps_recode_length = flash_union_buffer[get_max_storage].int16_type;
        if(INS.gps_recode_length >MAX_GPS_RECODE)INS.gps_recode_length = MAX_GPS_RECODE;
        int gps_max_storage = get_max_storage +INS.gps_recode_length*2 + 2 ;
        for(int i = get_max_storage+2 , j = 0; i < gps_max_storage ; i+=2 , j++)
        {
            INS.recode_gpsmap[j].lat = int32_to_double(flash_union_buffer[i].int32_type);
        }
        for(int i = get_max_storage+3 , j = 0; i < gps_max_storage ; i+=2 , j++)
        {
            INS.recode_gpsmap[j].lon = int32_to_double(flash_union_buffer[i].int32_type);
        }
    }

}

/**
 * 函数说明：Flash_Read_gpscheak()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_gpscheak(void)
{
    if(flash_check(FLASH_SECTION_INDEX,GPS_CHEAK_FLAG))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, GPS_CHEAK_FLAG);
        for(int i = 0 ; i < MAX_GPS_RECODE ; i++)
        {
            INS.recode_gpsmap[i].cheak_flag = flash_union_buffer[i].int16_type;
        }
        for(int i = MAX_GPS_RECODE , j = 1; i < MAX_GPS_RECODE*3/2 ; i++ , j+=2)
        {
            INS.recode_gpsmap[j].theta =flash_union_buffer[i].float_type ;
        }

    }

}
/**
 * 函数说明：Flash_Write_gpscheak()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_gpscheak(void)
{
    flash_buffer_clear();

    for(int i = 0 ; i < MAX_GPS_RECODE ; i++)
    {
        flash_union_buffer[i].int16_type = INS.recode_gpsmap[i].cheak_flag;
    }
    for(int i = MAX_GPS_RECODE , j = 1; i < MAX_GPS_RECODE*3/2 ; i++ , j+=2)
    {
        flash_union_buffer[i].float_type = INS.recode_gpsmap[j].theta;
    }


    if(flash_check(FLASH_SECTION_INDEX,GPS_CHEAK_FLAG))
    {
        flash_erase_page(FLASH_SECTION_INDEX,GPS_CHEAK_FLAG) ;
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX,GPS_CHEAK_FLAG);

}

