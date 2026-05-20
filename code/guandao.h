/*
 * guandao.h
 *
 *  Created on: 2026年3月16日
 *      Author: 18905
 */

#ifndef CODE_GUANDAO_H_
#define CODE_GUANDAO_H_


//宏
#define ONE_TICK_DISTANCE                      0.000378f                                     //每每脉冲的实际距离单位：cm
#define MAX_LENGTH_INDEX                      400                                             //存储上线
#define MAX_GPS_RECODE                          30
#define M_PI                                                  3.14159265358979323846f
//#define RECORD_THRESHOLD                     0.4f                                   // 记录换点阈值 单位：m
//#define PURSUIT_THRESHOLD                     0.6f                                  //追踪换点阈值 单位：m
//#define PREVIEW_SPETS                               2                                    //前瞻步长
#define WHEEL_BASE                                     0.724f                               //轴距 单位：m
#define TRACK_WIDTH                                   0.594f                              //轮距 单位：m
//#define BASE_SPEED                                      10.0f                               // 基础速度控制
#define MIN_SPEED                                        10.0f                                //最小速度
#define MAX_STEERING_RAD                        90.0f                                //转向输出限幅
#define SLIP_CHEAK_INDEX                            4.0f                                 //打滑角速度经检测阈值
#define START_GPS_FLAG                              1                                      //启用GPS标志
#define PORTION_TWO_INDEX                      3
#define ANGLE_CORRECT_KP                         0.0f
#define CORRECT_ANGLE_1                           -90.0f
#define CORRECT_ANGLE_3                           180.0f
//全局变量


typedef struct {
        float x;
        float y;
        float theta;
}state_t;


typedef struct {
        double lat;
        double lon;
        float theta;
        int16 cheak_flag;
}GPS_state;

typedef struct guandao{
        state_t current_state;
        state_t recode_map[MAX_LENGTH_INDEX];
        GPS_state recode_gpsmap[MAX_GPS_RECODE];

        int16 length_index;
        int current_point_index;

        int16 gps_recode_length;

        struct guandao * next;
}guandao_state;

typedef enum {
    NONE,
    Left_Slip,
    Right_Slip,
}SLIP_Cheak;

extern SLIP_Cheak slip_state;
extern guandao_state INS;
extern guandao_state passage;                    //1 = route_setting_choice
extern guandao_state portion_3;                     //2 = route_setting_choice
extern guandao_state portion_2;                    //3 = route_setting_choice
extern float out_v_l ;
extern float out_v_r ;
extern float out_servo ;
extern uint8 route_setting_choice;
extern int16 daoche_point_length ;
extern float daoche_speed ;
extern float base_speed ;
extern uint16 portion3_foint_flag ;
extern uint8 daoche_flash_cheack ;
extern float persuit_threshold ;  //PURSUIT_THRESHOLD
extern float recode_threshold ;//RECORD_THRESHOLD
extern int16 preview_spets ;       //PREVIEW_SPETS
extern float final_dsts ;
//函数
void guandao_state_init(guandao_state * e);
void guandao_chain_init(void);
void update_state(guandao_state * state , Encoder_t * ecd);            //实时更新惯导状态
float get_distance(state_t p1, state_t p2);                                          //计算欧式距离
void recode_waypoint(guandao_state * state);                                 //点位记录函数
void pursuit_contral_mode(guandao_state * state,float * out_v_l,float * out_v_r,float *out_servo);            //追踪惯导点位
void guandao_show(guandao_state * p);                                                                //显示记录点位函数
void follow_points_show(guandao_state * p);                                                       //显示追踪惯导点位函数
void build_map_text(guandao_state * state);                                  //手动建图 测试用
float speed_calculate(Encoder_t * ecd , float time_tick);                  //速度检测 -》 打滑检测
void slip_cheak(Encoder_t * ecd,float steer_angle);                          //打滑检测
void pursuit_midhandle(guandao_state * state ,state_t * current_state , int index ,float * angle , float * distanse);
void guandao_recode(guandao_state * state);
void guandao_trace(guandao_state * state);
void portion_1(void);
void portion_1_reset(void);
uint8 portion3_points_switch(void);
void Guandao_Points_Show(guandao_state * e);
void Key_Recode_Point(guandao_state * e);
void portion2_points_recode(void);
uint8 portion2_points_build(void);
void portion2_points_trace(uint8 channal1 , uint8 channal2 ,uint8 state );
void azimuth_adjust(guandao_state * state , float start_d , float dist_to_final , float * target_steering , float target_yaw );
#endif /* CODE_GUANDAO_H_ */
