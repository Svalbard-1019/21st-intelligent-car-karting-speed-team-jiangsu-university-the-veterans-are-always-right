/*
 * flash.c
 *
 *  Created on: 2025Äê11ÔÂ23ÈÕ
 *      Author: 18905
 */
#include "zf_common_headfile.h"

float speed_pid[6]={0};
int16 control[5];
float kp;
float ki;
float kd;



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
        base_speed = (float)control[0];
        daoche_speed = (float)control[1];
        preview_spets = control[2];
    }




}

void Flash_Write_pid(void)
{
    flash_buffer_clear();

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
void Flash_Main_Read(void)
{
    Flash_Read_pid();
    Flash_Read_INSpoints();
    Flash_Read_passage_points();
    Flash_Read_portion_3points();
    Flash_Read_gpscheak();


}
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

void Flash_Write_INSpoints(void)
{

    int max_storage = 2 * INS.length_index +2 ;
    if (max_storage >=MAX_LENGTH_INDEX*2) max_storage = MAX_LENGTH_INDEX*2;
    int gps_max_storage = max_storage +INS.gps_recode_length*2 + 2 ;
    if(gps_max_storage >= max_storage +MAX_GPS_RECODE*2 +2)gps_max_storage = max_storage +MAX_GPS_RECODE*2+2;
    flash_buffer_clear();

    flash_union_buffer[0].int16_type = INS.length_index;
    if(daoche_flash_cheack)flash_union_buffer[1].int16_type = daoche_point_length;
    else{flash_union_buffer[1].int16_type = INS.length_index;}
    for(int i = 2 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = INS.recode_map[j].x;
    }
    for(int i = 3 , j = 0;i < max_storage ; i += 2 , j++)
    {
        flash_union_buffer[i].float_type = INS.recode_map[j].y;
    }
    flash_union_buffer[max_storage].int16_type = INS.gps_recode_length;
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
void Flash_Read_INSpoints(void)
{
    int get_max_storage = 0;
    if(flash_check(FLASH_SECTION_INDEX,RECODE_MAP_POINTS_INDEX))
    {
        flash_buffer_clear();
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, RECODE_MAP_POINTS_INDEX);
        INS.length_index = flash_union_buffer[0].int16_type;
        daoche_point_length = flash_union_buffer[1].int16_type ;
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

