/*
 * UTF-8 详细注释说明：科目一惯导路线数据结构、参数和接口。
 *
 * 单位约定：
 * - x/y/距离：米。
 * - theta/Yaw/转角：度。
 * - out_v_l/out_v_r：旧工程速度单位，最终由 cpu0_main.c 按 GUANDAO_SPEED_TO_MPS 换算到 m/s。
 *
 * 接线依赖：
 * - 普通路线记录依赖后轮编码器。
 * - 航向角依赖 IMU963RA。
 * - GPS 只作为辅助校验点，不替代编码器记录路线。
 */

/*
 * guandao.h
 *
 *  Created on: 2026年3月16日
 *      Author: 18905
 */

#ifndef CODE_GUANDAO_H_
#define CODE_GUANDAO_H_


// ============================== 科目一惯导参数 ==============================
// ONE_TICK_DISTANCE：后轮编码器每个计数对应的车辆前进距离，单位 m。
// 当前只接左后轮编码器，左右轮里程都复用同一反馈，所以这个值直接影响记录距离和自动驾驶里程。
#define ONE_TICK_DISTANCE                      0.000378f
#define MAX_LENGTH_INDEX                       400        // 单条路线最多保存点数，Flash 写入也按这个上限组织
#define MAX_GPS_RECODE                         30         // GPS 辅助校验点数量上限
#define M_PI                                   3.14159265358979323846f
#define WHEEL_BASE                             0.724f     // 前后轴距，单位 m，用于纯追踪转角计算
#define TRACK_WIDTH                            0.594f     // 左右轮距，单位 m，用于差速速度分配
#define MIN_SPEED                              10.0f      // 惯导旧速度单位下的最低速度，最终会在 cpu0_main.c 换算为 m/s
#define MAX_STEERING_RAD                       90.0f      // 转向目标角限幅，单位 deg
#define SLIP_CHEAK_INDEX                       4.0f       // 打滑检测阈值，保留旧逻辑
#define START_GPS_FLAG                         1          // GPS 辅助开关标志
#define PORTION_TWO_INDEX                      3
#define ANGLE_CORRECT_KP                       0.0f
#define CORRECT_ANGLE_1                        -90.0f     // 科目一末段 GPS/航向修正目标角
#define CORRECT_ANGLE_3                        180.0f

// ============================== 数据结构 ==============================


// 车辆在惯导平面里的位姿。
// x/y 单位 m，theta 单位 deg；theta 由 IMU963RA 的 Yaw_1 提供。
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

// 一条可记录/可追踪的路线。
// INS 用于科目一主路线，passage/portion_3/portion_2 用于其他项目或辅助路线。
typedef struct guandao{
        state_t current_state;              // 当前实时位姿，由 update_state() 按编码器和 IMU 更新
        state_t recode_map[MAX_LENGTH_INDEX]; // 推车记录得到的路线点数组
        GPS_state recode_gpsmap[MAX_GPS_RECODE]; // GPS 辅助点，按 KEY2 记录，用于远距离校验/修正

        int16 length_index;              // 已记录路线点数量，也是自动驾驶的目标路线长度
        int current_point_index;        // 自动驾驶当前正在追踪的点索引

        int16 gps_recode_length;        // 已记录 GPS 辅助点数量

        struct guandao * next;
}guandao_state;

typedef enum {
    NONE,
    Left_Slip,
    Right_Slip,
}SLIP_Cheak;

extern SLIP_Cheak slip_state;
extern guandao_state INS;                         // 科目一主路线：记录模式保存，自动驾驶模式追踪
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
extern float guandao_debug_distance;             // 自动驾驶调试：当前位置到当前目标点距离
extern float guandao_debug_angle_diff;           // 自动驾驶调试：车头方向与目标点方向夹角
extern float guandao_debug_dist_final;           // 自动驾驶调试：当前位置到终点距离
extern uint8 guandao_debug_stop_reason;          // 自动驾驶调试：0正常，1空路线，2到点切换，4到终点
// ============================== 函数接口 ==============================
// 记录流程：guandao_recode() -> update_state() -> recode_waypoint()。
// 自动驾驶：portion_1()/guandao_trace() -> pursuit_contral_mode() -> out_v_l/out_v_r/out_servo。
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
void portion_1(void);                         // 科目一完整自动驾驶入口，按 INS 路线追踪到停车点/终点
void portion_1_reset(void);                   // 进入科目一前清零里程、输出、追点索引，避免沿用上一次状态
uint8 portion3_points_switch(void);
void Guandao_Points_Show(guandao_state * e);
void Key_Recode_Point(guandao_state * e);
void portion2_points_recode(void);
uint8 portion2_points_build(void);
void portion2_points_trace(uint8 channal1 , uint8 channal2 ,uint8 state );
void azimuth_adjust(guandao_state * state , float start_d , float dist_to_final , float * target_steering , float target_yaw );
#endif /* CODE_GUANDAO_H_ */
