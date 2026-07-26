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

float speed_pid[6]={0.5f, 1.0f, 0.0f, 0.3f, 0.4f, 3.0f};
int16 control[5] = {15, -4, 2, 0, 0};
float kp;
float ki;
float kd;

#define FLASH_RECODE_THRESHOLD_DEFAULT   (0.3f)
#define FLASH_RECODE_THRESHOLD_MIN       (0.05f)
#define FLASH_RECODE_THRESHOLD_MAX       (2.0f)
#define FLASH_PURSUIT_THRESHOLD_DEFAULT  (0.4f)
#define FLASH_PURSUIT_THRESHOLD_MIN      (0.05f)
#define FLASH_PURSUIT_THRESHOLD_MAX      (3.0f)
#define FLASH_FINAL_DSTS_DEFAULT         (3.0f)
#define FLASH_FINAL_DSTS_MIN             (0.3f)
#define FLASH_FINAL_DSTS_MAX             (20.0f)
#define FLASH_PREVIEW_STEPS_DEFAULT      (2)
#define FLASH_PREVIEW_STEPS_MIN          (1)
#define FLASH_PREVIEW_STEPS_MAX          (20)
#define FLASH_REVERSE_SPEED_DEFAULT     (-4)
#define FLASH_REVERSE_SPEED_MIN         (-40)
#define FLASH_REVERSE_SPEED_MAX         (-2)
#define FLASH_ROUTE_FORMAT_MAGIC         (0x4B525432u)
#define FLASH_ROUTE_FIRST_PAGE_POINTS    (500)

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
    speed_pid[3] = FLASH_RECODE_THRESHOLD_DEFAULT;
    speed_pid[4] = flash_sanitize_float(speed_pid[4],
                                        FLASH_PURSUIT_THRESHOLD_DEFAULT,
                                        FLASH_PURSUIT_THRESHOLD_MIN,
                                        FLASH_PURSUIT_THRESHOLD_MAX);
    speed_pid[5] = flash_sanitize_float(speed_pid[5],
                                        FLASH_FINAL_DSTS_DEFAULT,
                                        FLASH_FINAL_DSTS_MIN,
                                        FLASH_FINAL_DSTS_MAX);
    if(control[2] < FLASH_PREVIEW_STEPS_MIN || control[2] > FLASH_PREVIEW_STEPS_MAX)
    {
        control[2] = FLASH_PREVIEW_STEPS_DEFAULT;
    }
    if(control[1] < FLASH_REVERSE_SPEED_MIN || control[1] > FLASH_REVERSE_SPEED_MAX)
    {
        control[1] = FLASH_REVERSE_SPEED_DEFAULT;
    }
}

static void flash_route_buffer_reset(void)
{
    for(int i = 0; i < EEPROM_PAGE_LENGTH; i++) flash_union_buffer[i].uint32_type = 0xFFFFFFFFu;
}

static void flash_route_page_commit(uint32 page_index)
{
    if(flash_check(FLASH_SECTION_INDEX, page_index)) flash_erase_page(FLASH_SECTION_INDEX, page_index);
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, page_index);
}

static int16 flash_route_first_count(int16 route_length)
{
    return (route_length > FLASH_ROUTE_FIRST_PAGE_POINTS) ? FLASH_ROUTE_FIRST_PAGE_POINTS : route_length;
}

static int flash_route_fill_page(guandao_state *route, int16 route_length, int16 start_point,
                                 int16 point_count, uint8 continuation, int16 primary_aux)
{
    flash_route_buffer_reset();
    if(continuation)
    {
        flash_union_buffer[0].uint32_type = FLASH_ROUTE_FORMAT_MAGIC;
        flash_union_buffer[1].uint32_type = ((uint32)(uint16)route_length << 16)
                                          | (uint16)point_count;
    }
    else
    {
        flash_union_buffer[0].int16_type = route_length;
        flash_union_buffer[1].int16_type = primary_aux;
    }
    for(int16 i = 0; i < point_count; i++)
    {
        flash_union_buffer[2 + i * 2].float_type = route->recode_map[start_point + i].x;
        flash_union_buffer[3 + i * 2].float_type = route->recode_map[start_point + i].y;
    }
    return 2 + point_count * 2;
}

static void flash_route_read_buffer(guandao_state *route, int16 start_point, int16 point_count)
{
    for(int16 i = 0; i < point_count; i++)
    {
        route->recode_map[start_point + i].x = flash_union_buffer[2 + i * 2].float_type;
        route->recode_map[start_point + i].y = flash_union_buffer[3 + i * 2].float_type;
        route->recode_map[start_point + i].theta = 0.0f;
    }
}

static uint8 flash_route_load_continuation(uint32 page_index,
                                           int16 expected_length,
                                           int16 expected_count)
{
    uint32 route_header;

    if(!flash_check(FLASH_SECTION_INDEX, page_index)) return 0;
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, page_index);
    if(flash_union_buffer[0].uint32_type != FLASH_ROUTE_FORMAT_MAGIC) return 0;
    route_header = flash_union_buffer[1].uint32_type;
    if((int16)(route_header >> 16) != expected_length) return 0;
    return ((int16)(route_header & 0xFFFFu) == expected_count);
}

static void flash_clear_ins_parking_data(void)
{
    daoche_start_flag = 0;
    daoche_start_state.x = daoche_start_state.y = daoche_start_state.theta = 0.0f;
    daoche_target_flag = 0;
    daoche_target_length = 0;
    daoche_target_state.x = daoche_target_state.y = daoche_target_state.theta = 0.0f;
}

static void flash_read_ins_metadata(int metadata_index)
{
    int gps_max_storage;
    INS.gps_recode_length = flash_union_buffer[metadata_index].int16_type;
    if(INS.gps_recode_length < 0 || INS.gps_recode_length > MAX_GPS_RECODE) INS.gps_recode_length = 0;
    gps_max_storage = metadata_index + INS.gps_recode_length * 2 + 2;
    if(gps_max_storage + 8 >= EEPROM_PAGE_LENGTH)
    {
        INS.gps_recode_length = 0;
        flash_clear_ins_parking_data();
        return;
    }
    for(int i = metadata_index + 2, j = 0; i < gps_max_storage; i += 2, j++)
        INS.recode_gpsmap[j].lat = int32_to_double(flash_union_buffer[i].int32_type);
    for(int i = metadata_index + 3, j = 0; i < gps_max_storage; i += 2, j++)
        INS.recode_gpsmap[j].lon = int32_to_double(flash_union_buffer[i].int32_type);
    daoche_target_flag = (flash_union_buffer[gps_max_storage].int16_type == 1);
    daoche_target_length = flash_clamp_route_length(flash_union_buffer[gps_max_storage + 1].int16_type);
    daoche_target_state.x = flash_union_buffer[gps_max_storage + 2].float_type;
    daoche_target_state.y = flash_union_buffer[gps_max_storage + 3].float_type;
    daoche_target_state.theta = flash_union_buffer[gps_max_storage + 4].float_type;
    daoche_start_flag = (flash_union_buffer[gps_max_storage + 5].int16_type == 1);
    daoche_start_state.x = flash_union_buffer[gps_max_storage + 6].float_type;
    daoche_start_state.y = flash_union_buffer[gps_max_storage + 7].float_type;
    daoche_start_state.theta = flash_union_buffer[gps_max_storage + 8].float_type;
}

static void flash_validate_ins_parking_data(void)
{
    if(!daoche_start_flag || daoche_point_length <= 0 || daoche_point_length > INS.length_index
            || fabsf(daoche_start_state.x) > 10000.0f || fabsf(daoche_start_state.y) > 10000.0f
            || fabsf(daoche_start_state.theta) > 360.0f)
    {
        daoche_start_flag = 0;
        daoche_start_state.x = daoche_start_state.y = daoche_start_state.theta = 0.0f;
    }
    if(!daoche_start_flag && daoche_point_length > 0 && daoche_point_length < INS.length_index)
    {
        daoche_start_state = INS.recode_map[daoche_point_length];
        daoche_start_flag = 1;
    }
    if(!daoche_target_flag || daoche_point_length <= 0 || daoche_target_length <= daoche_point_length
            || daoche_target_length > INS.length_index || fabsf(daoche_target_state.x) > 10000.0f
            || fabsf(daoche_target_state.y) > 10000.0f || fabsf(daoche_target_state.theta) > 360.0f)
    {
        daoche_target_flag = 0;
        daoche_target_length = 0;
        daoche_target_state.x = daoche_target_state.y = daoche_target_state.theta = 0.0f;
    }
    if(!daoche_target_flag)
    {
        daoche_start_flag = 0;
        daoche_start_state.x = daoche_start_state.y = daoche_start_state.theta = 0.0f;
    }
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
        flash_sanitize_runtime_params();
        recode_threshold = speed_pid[3];
        persuit_threshold = speed_pid[4];
        final_dsts = speed_pid[5];
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
    }
    if(GPS_WORK_FLAG)Flash_Write_gpscheak();


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
    int16 route_length = flash_clamp_route_length(passage.length_index);
    int16 first_count = flash_route_first_count(route_length);
    int16 continuation_count = route_length - first_count;

    passage.length_index = route_length;
    flash_route_fill_page(&passage, route_length, 0, first_count, 0, 0);
    flash_route_page_commit(RECODE_PASSAGE);

    flash_route_fill_page(&passage, route_length, first_count, continuation_count, 1, 0);
    flash_route_page_commit(RECODE_PASSAGE_CONTINUATION);
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
    if(flash_check(FLASH_SECTION_INDEX,RECODE_PASSAGE))
    {
        int16 stored_length;
        int16 first_count;
        int16 continuation_count;

        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_PASSAGE);
        stored_length = flash_union_buffer[0].int16_type;
        if(stored_length < 0 || stored_length > MAX_LENGTH_INDEX)
        {
            passage.length_index = 0;
            return;
        }

        passage.length_index = stored_length;
        first_count = flash_route_first_count(stored_length);
        continuation_count = stored_length - first_count;
        flash_route_read_buffer(&passage, 0, first_count);
        if(continuation_count > 0)
        {
            if(!flash_route_load_continuation(RECODE_PASSAGE_CONTINUATION,
                    stored_length, continuation_count))
            {
                passage.length_index = 0;
                return;
            }
            flash_route_read_buffer(&passage, first_count, continuation_count);
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
    int16 route_length = flash_clamp_route_length(portion_3.length_index);
    int16 first_count = flash_route_first_count(route_length);
    int16 continuation_count = route_length - first_count;

    portion_3.length_index = route_length;
    flash_route_fill_page(&portion_3, route_length, 0, first_count, 0, 0);
    flash_route_page_commit(RECODE_PORTION_THREE);

    flash_route_fill_page(&portion_3, route_length, first_count, continuation_count, 1, 0);
    flash_route_page_commit(RECODE_PORTION_THREE_CONTINUATION);
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
    if(flash_check(FLASH_SECTION_INDEX,RECODE_PORTION_THREE))
    {
        int16 stored_length;
        int16 first_count;
        int16 continuation_count;

        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_PORTION_THREE);
        stored_length = flash_union_buffer[0].int16_type;
        if(stored_length <= 1 || stored_length > MAX_LENGTH_INDEX)
        {
            portion_3.length_index = 0;
            portion_3.gps_recode_length = 0;
            return;
        }

        /* Preserve the legacy convention that excludes the final saved point. */
        portion_3.length_index = flash_clamp_route_length(stored_length - 1);
        first_count = flash_route_first_count(portion_3.length_index);
        continuation_count = portion_3.length_index - first_count;
        flash_route_read_buffer(&portion_3, 0, first_count);
        if(continuation_count > 0)
        {
            if(!flash_check(FLASH_SECTION_INDEX, RECODE_PORTION_THREE_CONTINUATION))
            {
                portion_3.length_index = 0;
                return;
            }
            flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_PORTION_THREE_CONTINUATION);
            if(flash_union_buffer[0].uint32_type != FLASH_ROUTE_FORMAT_MAGIC
                    || (int16)(flash_union_buffer[1].uint32_type >> 16) != stored_length
                    || (int16)(flash_union_buffer[1].uint32_type & 0xFFFFu) < continuation_count
                    || (int16)(flash_union_buffer[1].uint32_type & 0xFFFFu)
                            > MAX_LENGTH_INDEX - FLASH_ROUTE_FIRST_PAGE_POINTS)
            {
                portion_3.length_index = 0;
                return;
            }
            flash_route_read_buffer(&portion_3, first_count, continuation_count);
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
    int16 first_count;
    int16 continuation_count;
    int metadata_index;
    int gps_max_storage;

    if(stop_length > route_length) stop_length = route_length;
    int16 gps_length = INS.gps_recode_length;
    if(gps_length < 0) gps_length = 0;
    if(gps_length > MAX_GPS_RECODE) gps_length = MAX_GPS_RECODE;

    INS.length_index = route_length;
    daoche_point_length = stop_length;
    INS.gps_recode_length = gps_length;

    first_count = flash_route_first_count(route_length);
    continuation_count = route_length - first_count;

    flash_route_fill_page(&INS, route_length, 0, first_count, 0,
            daoche_flash_cheack ? stop_length : route_length);
    flash_route_page_commit(RECODE_MAP_POINTS_INDEX);

    metadata_index = flash_route_fill_page(&INS, route_length, first_count,
            continuation_count, 1, 0);
    gps_max_storage = metadata_index + gps_length * 2 + 2;
    flash_union_buffer[metadata_index].int16_type = gps_length;
    for(int i = metadata_index + 2, j = 0; i < gps_max_storage; i += 2, j++)
    {
        flash_union_buffer[i].int32_type = double_to_int32(INS.recode_gpsmap[j].lat);
    }
    for(int i = metadata_index + 3, j = 0; i < gps_max_storage; i += 2, j++)
    {
        flash_union_buffer[i].int32_type = double_to_int32(INS.recode_gpsmap[j].lon);
    }

    flash_union_buffer[gps_max_storage].int16_type = daoche_target_flag;
    flash_union_buffer[gps_max_storage + 1].int16_type = daoche_target_length;
    flash_union_buffer[gps_max_storage + 2].float_type = daoche_target_state.x;
    flash_union_buffer[gps_max_storage + 3].float_type = daoche_target_state.y;
    flash_union_buffer[gps_max_storage + 4].float_type = daoche_target_state.theta;
    flash_union_buffer[gps_max_storage + 5].int16_type = daoche_start_flag;
    flash_union_buffer[gps_max_storage + 6].float_type = daoche_start_state.x;
    flash_union_buffer[gps_max_storage + 7].float_type = daoche_start_state.y;
    flash_union_buffer[gps_max_storage + 8].float_type = daoche_start_state.theta;
    flash_route_page_commit(RECODE_MAP_POINTS_CONTINUATION);

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
    int16 stored_length;
    int16 first_count;
    int16 continuation_count;
    int metadata_index;
    uint8 new_format = 0;

    INS.planned_length = 0;
    INS.plan_ready = 0;
    INS.length_index = 0;
    INS.gps_recode_length = 0;
    daoche_point_length = 0;
    flash_clear_ins_parking_data();
    if(flash_check(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX))
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_MAP_POINTS_INDEX);
        stored_length = flash_union_buffer[0].int16_type;
        if(stored_length < 0 || stored_length > MAX_LENGTH_INDEX) return;

        INS.length_index = stored_length;
        daoche_point_length = flash_clamp_route_length(flash_union_buffer[1].int16_type);
        if(daoche_point_length > INS.length_index) daoche_point_length = INS.length_index;
        first_count = flash_route_first_count(stored_length);
        continuation_count = stored_length - first_count;
        flash_route_read_buffer(&INS, 0, first_count);

        if(flash_route_load_continuation(RECODE_MAP_POINTS_CONTINUATION,
                stored_length, continuation_count))
        {
            new_format = 1;
            flash_route_read_buffer(&INS, first_count, continuation_count);
            metadata_index = 2 + continuation_count * 2;
            flash_read_ins_metadata(metadata_index);
        }
        else if(continuation_count > 0)
        {
            INS.length_index = 0;
            daoche_point_length = 0;
            return;
        }

        if(!new_format)
        {
            /* Read routes saved by the original single-page 400-point format. */
            flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_MAP_POINTS_INDEX);
            metadata_index = 2 + stored_length * 2;
            if(metadata_index + 8 < EEPROM_PAGE_LENGTH)
            {
                flash_read_ins_metadata(metadata_index);
            }
        }
        flash_validate_ins_parking_data();
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

