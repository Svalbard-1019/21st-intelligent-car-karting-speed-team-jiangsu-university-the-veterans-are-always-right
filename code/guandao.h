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
 * 主函数/科目一调用链：
 * 1. core0_main() 主循环根据 main_mode 分流：Guandao_Recode_Mode 调 guandao_recode(&INS)，Guandao_portion_1 调 portion_1()。
 * 2. 记录模式 guandao_recode() 调 update_state() 用后轮编码器和 Yaw_1 积分当前位置，再由 recode_waypoint() 按距离阈值自动保存路线点。
 * 3. 自动驾驶 portion_1() 调 guandao_trace(&INS)，guandao_trace() 再调用 pursuit_contral_mode() 计算 out_v_l、out_v_r 和 out_servo。
 * 4. out_servo 在 CCU61_CH0 中断里送入 Steer_Moter_Contral() 控制前轮；out_v_l/out_v_r 在主循环末尾由 Guandao_Rear_Motor_Update() 转成后轮 m/s 目标。
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
// 左右后轮编码器的脉冲距离标定值，直接影响记录距离、自动驾驶里程和打滑检测。
#define ONE_TICK_DISTANCE                      0.000473f
#define MAX_LENGTH_INDEX                       800        // 整合固件路线容量；Flash 使用两个 EEPROM 页保存
#define MAX_GPS_RECODE                         100        // GPS 辅助校验点数量上限
#define M_PI                                   3.14159265358979323846f
#define WHEEL_BASE                             0.724f     // 前后轴距，单位 m，用于纯追踪转角计算
#define TRACK_WIDTH                            0.594f     // 左右轮距，单位 m，用于差速速度分配
#define MIN_SPEED                              4.0f       // 惯导旧速度单位下的最低爬行速度，最终会在 cpu0_main.c 换算为 m/s
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
/**
 * 结构体说明：用于集中保存本模块的一组状态量/参数，字段通常会被初始化函数清零，并在周期函数中持续更新。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
typedef struct {
        float x;
        float y;
        float theta;
}state_t;


/**
 * 结构体说明：用于集中保存本模块的一组状态量/参数，字段通常会被初始化函数清零，并在周期函数中持续更新。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
typedef struct {
        double lat;
        double lon;
        float theta;
        int16 cheak_flag;
}GPS_state;

// 一条可记录/可追踪的路线。
// INS 用于科目一主路线，passage/portion_3/portion_2 用于其他项目或辅助路线。
/**
 * 结构体说明：用于集中保存本模块的一组状态量/参数，字段通常会被初始化函数清零，并在周期函数中持续更新。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
typedef struct guandao{
        state_t current_state;              // 当前实时位姿，由 update_state() 按编码器和 IMU 更新
        state_t recode_map[MAX_LENGTH_INDEX]; // 推车记录得到的路线点数组
        GPS_state recode_gpsmap[MAX_GPS_RECODE]; // GPS 辅助点，按 KEY2 记录，用于远距离校验/修正

        int16 length_index;              // 已记录路线点数量，也是自动驾驶的目标路线长度
        int16 planned_length;            // 平滑路线点数量，plan_ready 为 1 时供追踪使用
        int current_point_index;        // 自动驾驶当前正在追踪的点索引
        uint8 plan_ready;                // 平滑路线可用标志，避免记录模式误用 planned_map

        int16 gps_recode_length;        // 已记录 GPS 辅助点数量

        struct guandao * next;
}guandao_state;

/**
 * 枚举说明：用于描述运行模式、阶段或状态，实际取值会被菜单、控制流程或调试显示读取。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
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
extern int16 daoche_target_length;
extern state_t daoche_start_state;
extern uint8 daoche_start_flag;
extern state_t daoche_target_state;
extern uint8 daoche_target_flag;
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
extern uint8 guandao_debug_stop_reason;          // 自动驾驶调试：0正常，1空路线，2到点切换，4到终点，9科目三末段越点
extern uint8 guandao_debug_approach_active;       // 第一停车点末段进场控制是否接管
extern uint8 guandao_debug_entry_gate;            // 是否已经越过第一停车点的入口线
extern float guandao_debug_entry_long;            // 相对入口线纵向位置，负值表示尚未到线
extern float guandao_debug_entry_lat;             // 相对入口路线横向误差
extern float guandao_debug_entry_yaw;             // 当前航向相对入口方向误差
// ============================== 函数接口 ==============================
// 记录流程：guandao_recode() -> update_state() -> recode_waypoint()。
// 自动驾驶：portion_1()/guandao_trace() -> pursuit_contral_mode() -> out_v_l/out_v_r/out_servo。
/**
 * 接口说明：guandao_state_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
extern int16 guandao_debug_steer_preview;
extern int16 guandao_debug_curve_preview;
extern float guandao_debug_upcoming_turn;
extern float guandao_debug_steer_raw;
extern float guandao_debug_steer_limited;
extern float guandao_debug_steer_final;
extern float guandao_debug_speed_requested;
extern float guandao_debug_speed_command;
extern uint8 guandao_debug_turn_level;
void guandao_state_init(guandao_state * e);
/**
 * 接口说明：guandao_chain_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_chain_init(void);
/**
 * 接口说明：update_state()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void update_state(guandao_state * state , Encoder_t * ecd);            //实时更新惯导状态
/**
 * 接口说明：get_distance()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p1：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - p2：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float get_distance(state_t p1, state_t p2);                                          //计算欧式距离
/**
 * 接口说明：recode_waypoint()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void recode_waypoint(guandao_state * state);                                 //点位记录函数
void guandao_build_smooth_plan(guandao_state * state);                       //由记录点生成平滑追踪路线
/**
 * 接口说明：pursuit_contral_mode()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - out_v_l：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - out_v_r：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - out_servo：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void pursuit_contral_mode(guandao_state * state,float * out_v_l,float * out_v_r,float *out_servo);            //追踪惯导点位
/**
 * 接口说明：guandao_show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_show(guandao_state * p);                                                                //显示记录点位函数
/**
 * 接口说明：follow_points_show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void follow_points_show(guandao_state * p);                                                       //显示追踪惯导点位函数
/**
 * 接口说明：build_map_text()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void build_map_text(guandao_state * state);                                  //手动建图 测试用
/**
 * 接口说明：speed_calculate()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * - time_tick：时间周期或采样间隔，速度计算时会参与单位换算。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float speed_calculate(Encoder_t * ecd , float time_tick);                  //速度检测 -》 打滑检测
/**
 * 接口说明：slip_cheak()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * - steer_angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void slip_cheak(Encoder_t * ecd,float steer_angle);                          //打滑检测
/**
 * 接口说明：pursuit_midhandle()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - current_state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - index：路线点索引或通道编号，用于选择数据来源/目标点。
 * - angle：角度或航向相关参数，除特别说明外单位为度。
 * - distanse：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void pursuit_midhandle(guandao_state * state ,state_t * current_state , int index ,float * angle , float * distanse);
/**
 * 接口说明：guandao_recode()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_recode(guandao_state * state);
void guandao_record_session_reset(void);
/**
 * 接口说明：guandao_trace()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_trace(guandao_state * state);
/**
 * 接口说明：portion_1()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion_1(void);                         // 科目一完整自动驾驶入口，按 INS 路线追踪到停车点/终点
uint8 guandao_reverse_debug_state(void);
uint8 guandao_reverse_debug_plan_ready(void);
int16 guandao_reverse_debug_route_index(void);
int16 guandao_reverse_debug_route_count(void);
float guandao_reverse_debug_target_distance(void);
float guandao_reverse_debug_target_yaw_error(void);
float guandao_reverse_debug_steer_command(void);
float guandao_reverse_debug_lookahead(void);
uint8 guandao_portion3_reverse_active(void);
int16 guandao_portion3_reverse_index(void);
float guandao_portion3_reverse_final_distance(void);
float guandao_portion3_reverse_steer_command(void);
uint8 guandao_park_gps_debug_ready(void);
uint8 guandao_park_gps_debug_count(void);
int16 guandao_park_gps_debug_reference(void);
float guandao_park_gps_debug_offset_x(void);
float guandao_park_gps_debug_offset_y(void);
/**
 * 接口说明：portion_1_reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion_1_reset(void);                   // 进入科目一前清零里程、输出、追点索引，避免沿用上一次状态
/**
 * 接口说明：portion3_points_switch()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 portion3_points_switch(void);
void portion3_return_reset(void);
/**
 * 接口说明：Guandao_Points_Show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Guandao_Points_Show(guandao_state * e);
/**
 * 接口说明：Key_Recode_Point()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Key_Recode_Point(guandao_state * e);
/**
 * 接口说明：portion2_points_recode()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion2_points_recode(void);
/**
 * 接口说明：portion2_points_build()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 portion2_points_build(void);
/**
 * 接口说明：portion2_points_trace()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - channal1：路线点索引或通道编号，用于选择数据来源/目标点。
 * - channal2：路线点索引或通道编号，用于选择数据来源/目标点。
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion2_points_trace(uint8 channal1 , uint8 channal2 ,uint8 state );
/**
 * 接口说明：azimuth_adjust()。处理 IMU/陀螺仪数据，用于更新车体姿态和航向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - start_d：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - dist_to_final：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - target_steering：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - target_yaw：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void azimuth_adjust(guandao_state * state , float start_d , float dist_to_final , float * target_steering , float target_yaw );
#endif /* CODE_GUANDAO_H_ */
