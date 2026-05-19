/*
 * gps.c
 *
 *  Created on: 2026Äê4ÔÂ23ÈÕ
 *      Author: 18905
 */

#include "zf_common_headfile.h"

GPS_work gps_work;
Lost_Point lost_judge;
void angle_plan(float * angle)
{
    if(* angle >= 180){* angle -= 360 ;}
    else if(* angle < -180){* angle += 360 ;}

}

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

int32 double_to_int32(double y)
{
    int32 x = 0;
    x = (int32)(y*10000000);
    return x;
}
double int32_to_double(int32 y)
{
    double x = 0;
    x = (double)y * 1.0f / 10000000;
    return x;
}

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
