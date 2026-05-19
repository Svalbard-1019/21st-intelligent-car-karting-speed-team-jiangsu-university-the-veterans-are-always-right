/*
 * gps.h
 *
 *  Created on: 2026年4月23日
 *      Author: 18905
 */

#ifndef CODE_GPS_H_
#define CODE_GPS_H_

// 宏定义

#define GPS_WORK_NUM                            30
#define GPS_WORK_FLAG                            1
#define GPS_SWITCH_DISTANCE                  2.0f
//外部变量

typedef struct
{
        double lat;
        double lon;
        float theta;
        int16 gps_cheak_flag;

}GPS_points;

typedef struct
{
        GPS_points points[GPS_WORK_NUM];
        float angle_gps;
        float azimuth_gps;
        double distance_gps ;

        uint8 gps_update_flag;
        int16 work_gps_length;
        uint16 gps_current_point;

}GPS_work;

typedef enum{
    FIND,
    LOST,
}Lost_Point;
//函数声明
void gps_work_init(void);
void angle_plan(float * angle);
void recode_gps(guandao_state * state);
uint8 swtich_gps(void);
void trace_gps(guandao_state * e);
void update_gpsinformation(void);
int32 double_to_int32(double y);
double int32_to_double(int32 y);
void GPS_WorkMap_Copy(guandao_state * e);
void GPS_Points_Show(guandao_state * e);
void GPS_Work_SHOW(void);
#endif /* CODE_GPS_H_ */
