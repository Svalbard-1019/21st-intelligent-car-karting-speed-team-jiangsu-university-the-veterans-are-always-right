/*
 * UTF-8 详细注释说明：科目一惯导记录、路线追踪和纯追踪控制核心。
 *
 * 主数据：
 * - INS：科目一主路线，记录模式写入，自动驾驶模式读取。
 * - passage / portion_3 / portion_2：其他路线链表节点。
 * - out_v_l / out_v_r / out_servo：纯追踪输出，后轮速度由 cpu0_main.c 换算为 m/s。
 *
 * 运行流程：
 * 1. 记录模式 guandao_recode()：推车时 update_state() 积分位姿，recode_waypoint() 自动存点。
 * 2. 保存路线 Flash_Store_Mode()：KEY1 长按触发，保存路线和停车点。
 * 3. 科目一 portion_1()：读取 INS 路线，从 current_point_index 开始追踪。
 * 4. pursuit_contral_mode()：计算 D/A、切点、转向角、左右轮速度。
 *
 * 调试屏对应：
 * - Idx/Len：当前追踪点/路线总点数。
 * - D/A：到目标点距离/方向角误差。
 * - Reason：0正常，1空路线或到末尾，2距离到点切换，4到终点。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 主循环根据 main_mode 分流：Guandao_Recode_Mode 调 guandao_recode(&INS)，Guandao_portion_1 调 portion_1()。
 * 2. 记录模式 guandao_recode() 调 update_state() 用后轮编码器和 Yaw_1 积分当前位置，再由 recode_waypoint() 按距离阈值自动保存路线点。
 * 3. 自动驾驶 portion_1() 调 guandao_trace(&INS)，guandao_trace() 再调用 pursuit_contral_mode() 计算 out_v_l、out_v_r 和 out_servo。
 * 4. out_servo 在 CCU61_CH0 中断里送入 Steer_Moter_Contral() 控制前轮；out_v_l/out_v_r 在主循环末尾由 Guandao_Rear_Motor_Update() 转成后轮 m/s 目标。
 */


/*
 * guandao.c
 *
 *  Created on: 2026年3月16日
 *      Author: 18905
 */

#include "zf_common_headfile.h"
#include "rear_motor/rear_motor.h"

guandao_state INS;                               //0 = route_setting_choice
guandao_state passage;                    //1 = route_setting_choice
guandao_state portion_3;                     //2 = route_setting_choice
guandao_state portion_2;                    //3 = route_setting_choice

SLIP_Cheak slip_state = NONE;         // 打滑检测状态，初始为无打滑

uint8 route_setting_choice = 0;        // 路径选择标志（0-3）

float base_speed = 10.0f;
float persuit_threshold = 0.4f;         // 纯追踪阈值（到达目标点的距离容差）
float recode_threshold = 0.4f;         // 路径记录阈值
int16 preview_spets = 2;                  // 预瞄步数
float daoche_speed = -10.0;           //倒车速度
float final_dsts = 3.0f;                     // 终点距离减速阈值
float guandao_debug_distance = 0.0f;
float guandao_debug_angle_diff = 0.0f;
float guandao_debug_dist_final = 0.0f;
float guandao_debug_base_speed = 0.0f;
float guandao_debug_v_center = 0.0f;
float guandao_debug_yaw_offset = 0.0f;
uint8 guandao_debug_stop_reason = 0;

int16 daoche_point_length = 0;    // 倒车点长度
uint8 daoche_flag =0;                    // 倒车标志
uint8 daoche_flash_cheack =0;// 倒车Flash检查标志
static uint8 portion1_state_flag = 0;
static uint16 portion1_finally_length = 0;
static float guandao_heading_offset = 0.0f;
static float guandao_smoothed_steering = 0.0f;
static uint32 guandao_last_steer_ms = 0;

#define GUANDAO_START_SEARCH_POINTS    10
#define GUANDAO_TRACE_SEARCH_POINTS    8
#define GUANDAO_DEFAULT_PURSUIT_THRESHOLD 0.4f

static int16 guandao_clamp_length(int16 length)
{
    if(length < 0) return 0;
    if(length > MAX_LENGTH_INDEX) return MAX_LENGTH_INDEX;
    return length;
}

static int guandao_find_closest_index(guandao_state *state, int start_index, int end_index)
{
    int best_index = start_index;
    float best_distance = 0.0f;

    if(state->length_index <= 0) return 0;
    if(start_index < 0) start_index = 0;
    if(end_index >= state->length_index) end_index = state->length_index - 1;
    if(start_index > end_index) return start_index;

    best_distance = get_distance(state->current_state, state->recode_map[start_index]);
    for(int i = start_index + 1; i <= end_index; i++)
    {
        float distance = get_distance(state->current_state, state->recode_map[i]);
        if(distance + 0.05f < best_distance)
        {
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
}

static float guandao_get_route_start_heading(guandao_state *state)
{
    int end_index = 0;

    if(state->length_index < 2)
    {
        return Yaw_1;
    }

    end_index = state->length_index - 1;
    if(end_index > GUANDAO_START_SEARCH_POINTS) end_index = GUANDAO_START_SEARCH_POINTS;

    for(int i = 1; i <= end_index; i++)
    {
        float dx = state->recode_map[i].x - state->recode_map[0].x;
        float dy = state->recode_map[i].y - state->recode_map[0].y;

        if(hypotf(dx, dy) > 0.15f)
        {
            return atan2f(dx, dy) / M_PI * 180.0f;
        }
    }

    return state->recode_map[0].theta;
}

/*初始化管道状态数据结构*/
/**
 * 函数说明：guandao_state_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_state_init(guandao_state * e)
{
    e->current_point_index =0;
    e->current_state.theta =0.0f;
    e->current_state.x=0.0f;
    e->current_state.y=0.0f;
    e->length_index=0;

    e ->gps_recode_length =0;

}
/*初始化路径数据结构链*/
/**
 * 函数说明：guandao_chain_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_chain_init(void)
{
    INS.next = &passage;
    passage.next = &portion_3;
    portion_3.next = &portion_2;
    portion_2.next = NULL;
}
/*计算两点之间的欧氏距离*/
/**
 * 函数说明：get_distance()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p1：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - p2：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float get_distance(state_t p1, state_t p2)
{
    return hypotf(p2.x - p1.x, p2.y - p1.y);
}

/*基于编码器数据更新当前位姿（航迹推算）*/
/**
 * 函数说明：update_state()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void update_state(guandao_state * state , Encoder_t * ecd)
{
    float delta_real_center = 0;
    float delta_real_l = 0;
    float delta_real_r = 0;
    Encoder_Get(ecd);
    switch(slip_state)
    {
        case NONE:
            delta_real_l = (float)ecd->delta_l*ONE_TICK_DISTANCE;
            delta_real_r = (float)ecd->delta_r*ONE_TICK_DISTANCE;

            break;

        case Left_Slip:
            delta_real_r = (float)ecd->delta_r*ONE_TICK_DISTANCE;
            delta_real_l = delta_real_r;

            break;

        case Right_Slip:
            delta_real_l = (float)ecd->delta_l*ONE_TICK_DISTANCE;
            delta_real_r = delta_real_l;

            break;

        default : break;
    }

    if(!daoche_flag)
    {
        delta_real_center  = (delta_real_l+delta_real_r)/2.0f;
        state->current_state.theta =Yaw_1 + guandao_heading_offset;
        angle_plan(&state->current_state.theta);
    }
    else
    {
        delta_real_center  = -(delta_real_l+delta_real_r)/2.0f;
        state->current_state.theta =Yaw_1 + guandao_heading_offset + 180.0f;
        angle_plan(&state->current_state.theta);
    }


    state->current_state.x+=delta_real_center*sinf(state->current_state.theta/180.0f*M_PI);
    state->current_state.y+=delta_real_center*cosf(state->current_state.theta/180.0f*M_PI);

}
// 科目一自动驾驶入口前的状态复位。
// 注意：Flash 里的 INS.length_index 不能清零，它是已保存路线长度；这里只清运行态。
// current_point_index 从 1 开始，是为了避开记录路线时的第 0 个起点，防止起步时追起点。
/**
 * 函数说明：portion_1_reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion_1_reset(void)
{
    float route_start_heading = guandao_get_route_start_heading(&INS);

    portion1_state_flag = 0;
    portion1_finally_length = 0;
    INS.length_index = guandao_clamp_length(INS.length_index);
    INS.current_point_index = 0;
    daoche_flag = 0;
    persuit_threshold = GUANDAO_DEFAULT_PURSUIT_THRESHOLD;
    guandao_heading_offset = route_start_heading - Yaw_1;
    angle_plan(&guandao_heading_offset);
    guandao_debug_yaw_offset = guandao_heading_offset;
    guandao_smoothed_steering = 0.0f;
    guandao_last_steer_ms = 0;
    out_v_l = 0;
    out_v_r = 0;
    out_servo = 0;
    INS.current_state.x = 0.0f;
    INS.current_state.y = 0.0f;
    INS.current_state.theta = Yaw_1 + guandao_heading_offset;
    angle_plan(&INS.current_state.theta);
    Encoder_count_init(&guandao_ecd);
    Encoder_count_init(&Speed_ecd);
    encoder_clear_count(ENCODER_QUADDEC);
    rear_motor_stop();

    if(INS.length_index > 1)
    {
        int end_index = INS.length_index - 1;
        if(end_index > GUANDAO_START_SEARCH_POINTS) end_index = GUANDAO_START_SEARCH_POINTS;
        INS.current_point_index = guandao_find_closest_index(&INS, 1, end_index);
    }
}
/*第一部分路径跟踪（带倒车功能）*/
// 科目一主流程。
// 每次循环先用后轮编码器+Yaw 更新 INS.current_state，随后用纯追踪计算左右速度和转向目标。
// 如果打点时设置了停车点 daoche_point_length，则本次只追到停车点；否则追完整条路线。
/**
 * 函数说明：portion_1()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion_1(void)
{
    update_state(&INS,&guandao_ecd);                              // 更新当前车辆位姿（基于编码器航迹推算）
    if(portion1_state_flag == 0)                                   // 首次进入函数时确定本次停车点
    {
        portion1_finally_length = INS.length_index;
        if(daoche_point_length > 0 && daoche_point_length < portion1_finally_length)
        {
            INS.length_index = daoche_point_length;                // KEY1记录的点作为科目一停车点
        }
        else
        {
            INS.length_index = portion1_finally_length;            // 没有停车点时跑完整INS路线
        }
        portion1_state_flag = 1;
    }

    pursuit_contral_mode(&INS ,&out_v_l ,&out_v_r ,&out_servo);
    if(INS.current_point_index >= INS.length_index)
    {
        out_v_l = 0;
        out_v_r = 0;
        out_servo = 0;
        conrtol_mode = GUANDAO;
    }
    follow_points_show(&INS);
}
/*记录路径点

当移动距离超过记录阈值时记录新点

支持遥控器按键触发记录倒车点

自动过滤距离过近的点*/
/**
 * 函数说明：recode_waypoint()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void recode_waypoint(guandao_state * state)
{
    if(state ->length_index >=MAX_LENGTH_INDEX)return;

    if(state ->length_index ==0)
    {
        state->recode_map[state->length_index] =state->current_state;
        state->length_index++;
        return;
    }

    state_t last_recoded =  state->recode_map[state->length_index-1];
    float dist = get_distance(state->current_state, last_recoded );

    if(dist >=recode_threshold)
    {
        state->recode_map[state->length_index] =state->current_state;
        state->length_index++;
    }

    static uint8 dche_flag = 1;
    if(state->length_index >= MAX_LENGTH_INDEX)return;
    if(state == &INS && (key1_flag == 1|| x6f_out[3] ==200) && dche_flag ==1)  //遥控器控制
    {
        key1_flag =0;
        dche_flag =0;
        state->recode_map[state->length_index] =state->current_state;
        daoche_point_length = state->length_index;
        state->length_index++;
        daoche_flag =1;
        daoche_flash_cheack =1;
    }
}
/*第二部分路径点记录

按键1触发记录起始点

记录指定长度（PORTION_TWO_INDEX）的路径点*/
/**
 * 函数说明：portion2_points_recode()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion2_points_recode(void)
{
    static int16 p2p_r_flag1= 0 ;
//    static uint8 p2p_r_flag2= 0;
    if(key1_flag == 1)
    {
        key1_flag = 0;
        p2p_r_flag1 = passage.length_index;
        Key_Recode_Point(&passage);

    }
    if(passage.length_index - p2p_r_flag1<=PORTION_TWO_INDEX -1)
    {
        recode_waypoint(&passage);
    }


}
/*
 * 纯追踪（Pure Pursuit）控制算法核心。
 * 1. 读取当前追踪点，计算 D（距离）和 A（方向误差）。
 * 2. 只有 D 小于 persuit_threshold 才切换到下一个点，避免角度反向时直接跳到终点。
 * 3. 使用 preview_spets 预瞄点计算前轮目标角。
 * 4. 根据终点距离 final_dsts 做末段减速。
 * 5. 将中心速度拆成左右轮速度，交给 cpu0_main.c 换算成 m/s 后驱动后轮。
 */
/**
 * 函数说明：pursuit_contral_mode()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
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
void pursuit_contral_mode(guandao_state * state,float * out_v_l,float * out_v_r,float *out_servo)
{
    float actual_ld = 0 , preview_alpha =0;
    float actual_ld2 = 0 , preview_alpha2 =0;
    float target_steering = 0;

    guandao_debug_stop_reason = 0;
    if(state->length_index == 0 || state->current_point_index ==state->length_index)
    {
        guandao_debug_stop_reason = 1;
        * out_v_l = 0;
        * out_v_r = 0;
        *out_servo = 0;
//        Buzzer_check(50);
        return;
    }

    state_t current_point = state->current_state;
    if(state->current_point_index < 0) state->current_point_index = 0;
    if(state->current_point_index >= state->length_index) state->current_point_index = state->length_index - 1;

    int search_end_index = state->current_point_index + GUANDAO_TRACE_SEARCH_POINTS;
    if(search_end_index >= state->length_index) search_end_index = state->length_index - 1;
    int closest_index = guandao_find_closest_index(state, state->current_point_index, search_end_index);
    if(closest_index > state->current_point_index)
    {
        state->current_point_index = closest_index;
        guandao_debug_stop_reason = 5;
    }

    state_t target_point = state->recode_map[state->current_point_index];

    float dx = target_point.x - current_point.x;
    float dy = target_point.y - current_point.y;
    float distance_to_target = hypotf(dx, dy);
    guandao_debug_distance = distance_to_target;
    float angle_to_target = atan2f(dx,dy)/M_PI*180.0f;
    float angle_diff = angle_to_target - state->current_state.theta;

    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    guandao_debug_angle_diff = angle_diff;

    // 只允许“距离足够近”时切到下一个路线点。
    // 旧逻辑曾用 |angle_diff| > 90 直接跳点，车头方向一反就会瞬间跳到终点并停车。
    if(distance_to_target <= persuit_threshold)
    {
        if(guandao_debug_stop_reason == 0) guandao_debug_stop_reason = 2;
        state->current_point_index++;
        if(state->current_point_index >=state->length_index )
        {   state->current_point_index = state->length_index;
            guandao_debug_stop_reason = 4;
            * out_v_l = 0;
            * out_v_r = 0;
            *out_servo = 0;
//            Buzzer_check(50);
            return;
        }
    }

    // preview_spets 决定转向预瞄点，数值越大越平滑，但弯道响应会更慢。
    pursuit_midhandle(state , &current_point , preview_spets , &preview_alpha , &actual_ld);
    pursuit_midhandle(state , &current_point , 5 , &preview_alpha2 , &actual_ld2);

   float k = 0;
   k = -0.0038*fabs(preview_alpha2) + 1;
   Value_Limit_float(&k ,0.6 ,1);

   if(angle_diff >=90)
   {
       target_steering = 30.0f;

   }
   else if(angle_diff <= -90)
   {
       target_steering = -30.0f;
   }
   else if(fabsf(angle_diff) <=90)
   {
       target_steering = GUANDAO_STEERING_GAIN*atan2f(2.0f * WHEEL_BASE * sinf(preview_alpha/180.0f*M_PI), actual_ld)/M_PI*180.0f;
   }
   ips200_show_float(X(10),  Y(9),target_steering ,5 ,5);

   //限幅
   Value_Limit_float(&target_steering ,-MAX_STEERING_RAD,MAX_STEERING_RAD);

//   slip_cheak(&guandao_ecd,target_steering);

   float dist_to_final = get_distance(state->current_state, state->recode_map[state->length_index -1]);
   guandao_debug_dist_final = dist_to_final;
   float v_center = base_speed;
   guandao_debug_base_speed = base_speed;


   switch(route_setting_choice)
   {
       case 0:
           azimuth_adjust(state , 1.3 , dist_to_final , &target_steering ,CORRECT_ANGLE_1);
           break;
       case 2:
           azimuth_adjust(state , 5.5 , dist_to_final , &target_steering ,CORRECT_ANGLE_3);
           break;
       case 3:

           break;
       default : break;
   }
   Value_Limit_float(&target_steering ,-MAX_STEERING_RAD,MAX_STEERING_RAD);


   if (dist_to_final < final_dsts && state->current_point_index >= state->length_index - 30)
   {

       persuit_threshold = persuit_threshold*(dist_to_final / final_dsts);
       if(persuit_threshold < 0.3f){persuit_threshold = 0.3f;}
       v_center = base_speed * (dist_to_final / final_dsts);
       if (v_center < MIN_SPEED) v_center = MIN_SPEED; // 最低速度限制
   }

   // 差动驱动速度分配：这里仍是惯导旧速度单位，cpu0_main.c 会再换算为 m/s 给 rear_motor。
   uint32 now_steer_ms = system_getval_ms();
   uint32 steer_dt_ms = (guandao_last_steer_ms == 0) ? 10 : (uint32)(now_steer_ms - guandao_last_steer_ms);
   float max_steer_step = 0.0f;
   float steer_delta = 0.0f;

   if(steer_dt_ms > 50) steer_dt_ms = 50;
   max_steer_step = GUANDAO_STEERING_RATE_PER_10MS * ((float)steer_dt_ms / 10.0f);
   if(max_steer_step < 0.5f) max_steer_step = 0.5f;
   steer_delta = target_steering - guandao_smoothed_steering;
   Value_Limit_float(&steer_delta, -max_steer_step, max_steer_step);
   guandao_smoothed_steering += steer_delta;
   target_steering = guandao_smoothed_steering;
   guandao_last_steer_ms = now_steer_ms;

   guandao_debug_v_center = v_center;
   float w = (v_center * tanf(target_steering/3.0f/180.0f*M_PI)) / WHEEL_BASE;
   *out_v_l = v_center + (w * TRACK_WIDTH / 2.0f);
   *out_v_r = v_center - (w * TRACK_WIDTH / 2.0f);

   *out_servo = -target_steering;


}
/*终点航向角校正函数，
 * 用于在车辆接近终点时修正行驶方向，
 * 确保以特定角度到达目标点。
 */
/**
 * 函数说明：azimuth_adjust()。处理 IMU/陀螺仪数据，用于更新车体姿态和航向角。
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
void azimuth_adjust(guandao_state * state ,float start_d , float dist_to_final , float * target_steering , float target_yaw )
{
    float angle_delta = 0;
    if(dist_to_final < start_d && state->current_point_index >= state->length_index - 30)
    {
        if(!daoche_flag)angle_delta  = target_yaw - Yaw_1;
        else angle_delta  = -(target_yaw - Yaw_1);
        angle_plan(&angle_delta);

        float kp_d = (1 - ANGLE_CORRECT_KP)/start_d*dist_to_final +ANGLE_CORRECT_KP;
        float kp_y = (ANGLE_CORRECT_KP -1)*(dist_to_final/start_d - 1);

        * target_steering = kp_d*(* target_steering) + kp_y *angle_delta;
    }
}
/*中间计算函数

根据索引获取预瞄点（记录模式下取最后一点）

计算预看点相对于当前位置的夹角

计算到预看点的距离*/
// 计算预瞄点相对车辆的角度和距离。
// 自动驾驶时预瞄 current_point_index + index；记录模式下只用于显示最后一个记录点方向。
// 计算预瞄点相对车辆的角度和距离。
// 自动驾驶时预瞄 current_point_index + index；记录模式下只用于显示最后一个记录点方向。
// 计算预瞄点相对车辆的角度和距离。
// 自动驾驶时预瞄 current_point_index + index；记录模式下只用于显示最后一个记录点方向。
/**
 * 函数说明：pursuit_midhandle()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
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
void pursuit_midhandle(guandao_state * state ,state_t * current_state , int index ,float * angle , float * distanse)
{
    int preview_index = 0;
    if(main_mode == Guandao_Recode_Mode)
    {
        preview_index = state->length_index - 1;
    }
    else
    {
        preview_index = state->current_point_index+index;
    }

   if(preview_index >=state->length_index)preview_index = state->length_index - 1;

   state_t preview_point = state->recode_map[preview_index];

   float p_dx = preview_point.x - current_state->x;
   float p_dy = preview_point.y - current_state->y;
    * angle = atan2f(p_dx,p_dy)/M_PI*180.0f - state->current_state.theta;

   while (* angle > 180.0f) * angle -= 360.0f;
   while (* angle < -180.0f) * angle += 360.0f;

    * distanse = hypotf(p_dx, p_dy);
    if(*distanse < 0.1f) *distanse = 0.1f;

}
/* 手动建图*/
/**
 * 函数说明：build_map_text()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void build_map_text(guandao_state * state)
{
    float length = 0.176f *6.0;
    for(int i = 0 ; i<=5 ;i++)
    {
        state->recode_map[i].x = length * i;
        state->recode_map[i].y = length * i;
        state->recode_map[i].theta = 45.0f;
    }
    for(int i = 6 ; i<=10 ;i++)
    {
        state->recode_map[i].x = length* 10 - length * i;
        state->recode_map[i].y = length * i;
        state->recode_map[i].theta = 45.0f;
    }
    for(int i = 11 ; i<=15 ;i++)
    {
        state->recode_map[i].x =length* 10 - length* i;
        state->recode_map[i].y = length* 20 - length * i;
        state->recode_map[i].theta = 45.0f;
    }
    for(int i = 16 ; i<=20 ;i++)
    {
        state->recode_map[i].x =-length* 20 + length * i;


        state->recode_map[i].y = length* 20 - length* i;
        state->recode_map[i].theta = 45.0f;
    }
    state->length_index = 20;
    daoche_point_length =20;

//    for(int i = 0 ; i<20 ;i++)
//    {
//        state->recode_map[i].x = 0;
//        state->recode_map[i].y = length * i;
//        state->recode_map[i].theta = 0.0f;
//    }

}

float out_v_l = 0;
float out_v_r = 0;
float out_servo = 0;
/*
 * 推车记录路线入口。
 * 普通路线点不需要按键，车辆移动超过 recode_threshold 就会自动记录。
 * KEY1 短按：记录科目一停车点；KEY1 长按：保存当前路线到 Flash；KEY2：可选 GPS 辅助点。
 * route_setting_choice 决定当前写入 INS、passage、portion_3 还是 portion_2。
 */
/**
 * 函数说明：guandao_recode()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_recode(guandao_state * state)
{
    static uint8 flag0 = 1;                                                 // 首次调用标志（1=首次，0=已初始化），用于执行一次性初始化
    static uint8 flag1 = 1;                                                 // Flash存储标志（1=允许存储，0=已存储），防止重复保存
    static uint32 key1_save_start_ms = 0;
    static uint8 key1_save_wait_release = 0;
    uint32 now_ms = 0;
    int choice_flag = 0;                                                    // 路径选择计数器，用于遍历链表找到目标路径

    guandao_state * p = state;                                          // 工作指针，指向当前路径节点，用于链表遍历
    while(choice_flag < route_setting_choice)               // 根据route_setting_choice的值，遍历链表选择目标路径
                                                                                        // route_setting_choice: 0=INS, 1=passage, 2=portion_3, 3=portion_2
    {
        p = p->next;                                                        // 指针后移，指向下一个路径节点
        if(p == NULL)return;                                            // 空指针保护：若链表提前结束则退出函数
        choice_flag++;                                                  // 空指针保护：若链表提前结束则退出函数
    }

    if(flag0){  guandao_state_init(p); daoche_point_length = 0; daoche_flash_cheack = 0;  flag0 =0;}          // 清空路径点数组，重置索引和位姿
    guandao_heading_offset = 0.0f;
    guandao_debug_yaw_offset = 0.0f;
    update_state(p  , &guandao_ecd);                        // 基于编码器数据更新当前车辆位姿（x, y, theta）


    // KEY1 为上拉输入：未按=1，按下=0。
    // 按住超过 1.5s 保存路线；短按松开后才交给 recode_waypoint() 记录停车点。
    if(gpio_get_level(KEY1) == 0)
    {
        key1_flag = 0;
        now_ms = system_getval_ms();
        if(key1_save_start_ms == 0) key1_save_start_ms = now_ms;
        if((uint32)(now_ms - key1_save_start_ms) > 1500 && flag1)
        {
            Flash_Store_Mode(route_setting_choice);
            Buzzer_check(50);
            flag1 = 0;
            key1_save_wait_release = 1;
        }
        return;
    }
    else
    {
        if(key1_save_start_ms != 0 && key1_save_wait_release == 0)
        {
            key1_flag = 1;                 // 短按松开后才记录停车点
        }
        key1_save_start_ms = 0;
        if(key1_save_wait_release)
        {
            key1_save_wait_release = 0;
            return;
        }
    }
    if( p == &passage)portion2_points_recode();     // passage路径：按键手动记录（适合构建复杂赛道）
    else recode_waypoint(p);                                         // 其他路径：自动等距记录（移动超过阈值自动记录）

//    guandao_show(p);                                                // 在IPS200屏幕上显示路径信息（长度、位姿等）
    if(GPS_WORK_FLAG){if(key2_flag == 1){ key2_flag = 0 ; recode_gps(p);  }}        // GPS辅助记录（可选）：当GPS工作标志为真且按键2被按下时
//     guandao_show();


    if((x6f_out[2] == 200)&&flag1){   Flash_Store_Mode(route_setting_choice);  Buzzer_check(50);  flag1 = 0; };    // Flash存储触发：长按KEY1或遥控器通道2（值为200）
    // flag1确保只存储一次，避免重复写入

}
/*这是一个路径生成器函数，
 * 用于自动构建一个对称的8段式复杂路径。
 * 它通过复制、镜像、插值等方式，
 * 从初始的几段路径数据生成完整的往返赛道路径。*/
/**
 * 函数说明：portion2_points_build()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 portion2_points_build(void)
{

    for( uint8 i = PORTION_TWO_INDEX*5 , j =PORTION_TWO_INDEX*0 ; i <PORTION_TWO_INDEX*5 +PORTION_TWO_INDEX; i++ , j++)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    for( uint8 i = PORTION_TWO_INDEX*6  , j =PORTION_TWO_INDEX*2 ; i <PORTION_TWO_INDEX*6 +PORTION_TWO_INDEX; i++ ,j++)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    for( uint8 i = PORTION_TWO_INDEX*7 , j =PORTION_TWO_INDEX*4; i <PORTION_TWO_INDEX*7 +PORTION_TWO_INDEX; i++ , j++)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    for( uint8 i = PORTION_TWO_INDEX*0 , j =PORTION_TWO_INDEX*4-1; i <PORTION_TWO_INDEX*0 +PORTION_TWO_INDEX; i++ ,j--)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    for( uint8 i = PORTION_TWO_INDEX*3  , j =PORTION_TWO_INDEX*0 ; i <PORTION_TWO_INDEX*3 +PORTION_TWO_INDEX; i++ ,j++)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    for( uint8 i = PORTION_TWO_INDEX*1  , j =PORTION_TWO_INDEX*1; i <PORTION_TWO_INDEX*1 +PORTION_TWO_INDEX; i++,j++)
    {
        passage.recode_map[i].x =  passage.recode_map[j].x;
        passage.recode_map[i].y =  passage.recode_map[j].y;
    }
    float index = (passage.recode_map[PORTION_TWO_INDEX * 3].x - passage.recode_map[PORTION_TWO_INDEX].x)/2.0f;
    for(uint8 i = 0 ,j = PORTION_TWO_INDEX *2 , m = PORTION_TWO_INDEX *4; i<PORTION_TWO_INDEX ; i++ ,j++ , m++)
    {
        passage.recode_map[i].x =  passage.recode_map[PORTION_TWO_INDEX].x - index;
        passage.recode_map[i].y =  passage.recode_map[PORTION_TWO_INDEX].y +recode_threshold*i;
        passage.recode_map[j].x =  passage.recode_map[PORTION_TWO_INDEX].x + index;
        passage.recode_map[j].y =  passage.recode_map[PORTION_TWO_INDEX].y+recode_threshold*i;
        passage.recode_map[m].x =  passage.recode_map[3*PORTION_TWO_INDEX].x + index;
        passage.recode_map[m].y =  passage.recode_map[PORTION_TWO_INDEX].y+recode_threshold*i;
    }
    passage.length_index = PORTION_TWO_INDEX*8;
    return 1;

}
/*这是一个四阶段状态机函数，用于实现复杂路径的组合跟踪。
 * 它通过从passage（通道路径）中提取不同的路径段，
 * 组合成新的路径portion_2，并交替执行路径跟踪。*/
/**
 * 函数说明：portion2_points_trace()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - channal1：路线点索引或通道编号，用于选择数据来源/目标点。
 * - channal2：路线点索引或通道编号，用于选择数据来源/目标点。
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void portion2_points_trace(uint8 channal1 , uint8 channal2 ,uint8 state )
{
    static uint8 p2p_state = 0;
    switch(p2p_state)
    {
        case 0:
            for(uint8 i  = 0 , j = PORTION_TWO_INDEX*5 ; i <PORTION_TWO_INDEX ; i ++ ,j++)
            {
                portion_2.recode_map[i].x = passage.recode_map[j].x;
                portion_2.recode_map[i].y = passage.recode_map[j].y;
            }
            for(uint8 i  = PORTION_TWO_INDEX*1 , j = PORTION_TWO_INDEX*channal1 ; i <PORTION_TWO_INDEX*1 +PORTION_TWO_INDEX ; i ++ ,j++)
            {
                portion_2.recode_map[i].x = passage.recode_map[j].x;
                portion_2.recode_map[i].y = passage.recode_map[j].y;
            }
            for(uint8 i  = PORTION_TWO_INDEX*2 , j = PORTION_TWO_INDEX*6 ; i <PORTION_TWO_INDEX*2 +PORTION_TWO_INDEX ; i ++ ,j++)
            {
                portion_2.recode_map[i].x = passage.recode_map[j].x;
                portion_2.recode_map[i].y = passage.recode_map[j].y;
            }
            portion_2.length_index = PORTION_TWO_INDEX*3;
            p2p_state++;
               break;
        case 1:
            guandao_trace(&INS);
            if(state) p2p_state++;
            break;

        case 2:
            for(uint8 i  = PORTION_TWO_INDEX*3 , j = PORTION_TWO_INDEX*(channal2+1)-1 ; i <PORTION_TWO_INDEX*3 +PORTION_TWO_INDEX ; i ++ ,j--)
            {
                portion_2.recode_map[i].x = passage.recode_map[j].x;
                portion_2.recode_map[i].y = passage.recode_map[j].y;
            }
            for(uint8 i  = PORTION_TWO_INDEX*4 , j = PORTION_TWO_INDEX*7 ; i <PORTION_TWO_INDEX*4 +PORTION_TWO_INDEX ; i ++ ,j++)
            {
                portion_2.recode_map[i].x = passage.recode_map[j].x;
                portion_2.recode_map[i].y = passage.recode_map[j].y;
            }
            portion_2.length_index = PORTION_TWO_INDEX*5;
            p2p_state++;
            break;
        case 3:
            guandao_trace(&INS);
            break;

        default :break;
    }

}
/*管道路径跟踪主函数

根据路径选择标志选择数据结构

更新当前位姿

调用纯追踪控制算法

如果启用GPS，调用GPS轨迹跟踪

显示跟踪状态*/
/**
 * 函数说明：guandao_trace()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - state：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_trace(guandao_state * state)
{
//    static uint8 flag2 = 1;
    int choice_flag = 0;                           // 路径选择计数器，用于遍历链表找到目标路径

    guandao_state * p = state;                // 工作指针，指向当前路径节点，用于链表遍历
    while(choice_flag < route_setting_choice)
    {
        p = p->next;                             // 指针后移，指向下一个路径节点
        if(p == NULL)return;              // 空指针保护：若链表提前结束则退出函数
        choice_flag++;
    }
    update_state(p,&guandao_ecd); // 基于编码器数据更新当前车辆位姿（x, y, theta）

    // ========== 纯追踪控制 ==========
    pursuit_contral_mode(p ,&out_v_l ,&out_v_r ,&out_servo);
    if(GPS_WORK_FLAG)trace_gps(p);                              // 调用GPS轨迹跟踪函数
    follow_points_show(p);
//    follow_points_show();


}


/*真实速度计算*/
/**
 * 函数说明：speed_calculate()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * - time_tick：时间周期或采样间隔，速度计算时会参与单位换算。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float speed_calculate(Encoder_t * ecd , float time_tick)
{
    float v_speed = (ecd->delta_l +ecd->delta_r)*ONE_TICK_DISTANCE/time_tick/2.0f;
            return v_speed;
}
/*打滑检测

计算理论角速度与实际角速度的差值

当速度>1m/s且差值超阈值时判定为打滑

根据左右轮增量判断打滑方向（左滑/右滑）*/
/**
 * 函数说明：slip_cheak()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - ecd：编码器相关输入或计数值，用于速度、里程或角度换算。
 * - steer_angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void slip_cheak(Encoder_t * ecd,float steer_angle)
{
    static int flag = 0;

    float v_speed = 0 ;
    v_speed =speed_calculate(ecd , 0.007);
    float w = (v_speed * tanf(steer_angle/180.0f*M_PI)) / WHEEL_BASE;


    float slip_index = fabs(w - IMU_Data.gyro_z);

//    ips200_show_int(X(1),  Y(1) ,flag, 5);
//    ips200_show_float(X(1),  Y(2) ,v_speed,3, 2);
//    ips200_show_float(X(1),  Y(3) ,w,3, 2);
//    ips200_show_float(X(1),  Y(4) ,slip_index,3, 2);

    if(v_speed >= 1.0 &&  slip_index >= SLIP_CHEAK_INDEX)
    {
        flag++;

        if(ecd->delta_l > ecd->delta_r){ slip_state =Left_Slip; }
        else if(ecd->delta_r >=ecd->delta_l ) {slip_state =Right_Slip;}
    }
    else slip_state =NONE;
}

uint16 portion3_foint_flag = 0;
/*这个函数用于对第三部分路径进行翻转和反向处理，实现路径的镜像或回程路径生成。主要用于创建往返路径或对称轨迹。*/
/**
 * 函数说明：portion3_points_switch()。执行路线追踪或科目阶段逻辑，输出目标速度和转向角。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 portion3_points_switch(void)
{
    float x_delta = portion_3.recode_map[portion_3.length_index -1 ].x ;
    int8 cut_length =(uint8)(WHEEL_BASE/recode_threshold);
    if(cut_length < 1)cut_length = 1;

    portion3_foint_flag = cut_length;

    for(uint8 i = 1 ; i<=cut_length ; i++ )
    {
        portion_3.recode_map[portion_3.length_index].x = portion_3.recode_map[portion_3.length_index - 1].x ;
        portion_3.recode_map[portion_3.length_index].y = portion_3.recode_map[portion_3.length_index - 1].y - i*recode_threshold;
        portion_3.length_index ++;

        if(portion_3.length_index >=MAX_LENGTH_INDEX)return 0;
    }

    for(int i =cut_length ; i < portion_3.length_index ; i++)
    {
        portion_3.recode_map[i].x -= x_delta;
    }

    float * p = (float *)malloc(sizeof(portion_3.recode_map[0].x)*portion_3.length_index*2);
    for( int i = portion_3.length_index -1 , j = 1 ; i >= cut_length ; i-- , j++)
    {
        p[2*j - 2] =  portion_3.recode_map[i].x;
        p[2*j -1] =  portion_3.recode_map[i].y;
    }
    portion_3.length_index -=cut_length;
    for(int i = 0  , j = 1; i < portion_3.length_index-1 ; i++ , j++)
    {
        portion_3.recode_map[i].x = p[2*j - 2];
        portion_3.recode_map[i].y = p[2*j -1];
    }
    free(p);

    return 1;
}
/*在IPS200屏幕上显示路径点地图

自动计算坐标范围并缩放到屏幕

绘制路径轨迹（白色线）

绘制进度条（绿色线）

每点延时20ms，形成动画效果*/
/**
 * 函数说明：Guandao_Points_Show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Guandao_Points_Show(guandao_state * e)
{
    int choice_flag = 0;

    guandao_state * p = e;
    while(choice_flag < route_setting_choice)
    {
        p = p->next;
        if(p == NULL)return;
        choice_flag++;
    }
    if(p->length_index == 0)return;

    float Max_R_Line = -10000.0f , Max_D_Line = -10000.0f , Min_L_Line = 10000.0f , Min_U_Line = 10000.0f;
    float Center_H = 0 ,  Center_W = 0,  INDEX_H = 0,  INDEX_W = 0 , INDEX_Progress = 0;
    uint16 GD_Show [2][p->length_index] ; uint16 Progress_Show [2][p->length_index] ;
    for( int i = 0 ; i< p->length_index ;i ++)
    {
        if(p->recode_map[i].x < Min_L_Line)Min_L_Line =p->recode_map[i].x;
        if(p->recode_map[i].x > Max_R_Line)Max_R_Line =p->recode_map[i].x;
        if(p->recode_map[i].y < Min_U_Line)Min_U_Line =p->recode_map[i].y;
        if(p->recode_map[i].y > Max_D_Line)Max_D_Line =p->recode_map[i].y;
    }
    Center_W = (Max_R_Line + Min_L_Line)/2.0f;
    Center_H = (Max_D_Line + Min_U_Line)/2.0f;
    INDEX_W = Max_R_Line - Min_L_Line;
    INDEX_H = Max_D_Line - Min_U_Line;

    for(  int i = 0 ; i< p->length_index ; i ++ )
    {
        GD_Show[0][i] = 110.0f + (p->recode_map[i].x - Center_W)*(200.0f/INDEX_W);
        GD_Show[1][i] = 150.0f - (p->recode_map[i].y - Center_H)*(280.0f/INDEX_H);

    }

    INDEX_Progress = 1040.0f/p->length_index;
    for(int i = 0 ; i <p->length_index*300/1040 ; i++){Progress_Show [0][i] = 0;Progress_Show [1][i]= 300 - i*INDEX_Progress; }
    for(int i = p->length_index*300/1040 , j =0 ; i <p->length_index/2 ; i++ , j++){Progress_Show [0][i] = j*INDEX_Progress ;Progress_Show [1][i] = 0; }
    for(int i = p->length_index/2 , j =0 ; i <p->length_index*820/1040 ; i++ , j++){Progress_Show [0][i] = 220 ;Progress_Show [1][i] = j *INDEX_Progress; }
    for(int i = p->length_index*820/1040 , j =0 ; i <p->length_index ; i++ , j++){Progress_Show [0][i] = 220 - j*INDEX_Progress ;Progress_Show [1][i] = 300; }

    for(int i = 0 ; i< p->length_index - 1  ; i ++ )
    {
        ips200_draw_point(GD_Show[0][i],GD_Show[1][i],RGB565_WHITE);
        ips200_draw_line(GD_Show[0][i] ,GD_Show[1][i] ,GD_Show[0][i+1] ,GD_Show[1][i+1] , RGB565_WHITE);
        ips200_draw_line(Progress_Show[0][i] ,Progress_Show[1][i] ,Progress_Show[0][i+1] ,Progress_Show[1][i+1] , RGB565_GREEN);
        system_delay_ms(20);

    }


}
/**
 * 函数说明：guandao_show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void guandao_show(guandao_state * p)
{

    ips200_show_int(X(1),  Y(0) ,p->length_index, 5);                                              ips200_show_float(X(10),Y(0),p->current_state.theta,3,2);
    ips200_show_float(X(1),  Y(3) ,p->recode_map[INS.length_index-1].x, 5,2);     ips200_show_float(X(10),  Y(3) ,p->recode_map[INS.length_index-1].y, 5,2);
    ips200_show_float(X(1),  Y(4) ,p->current_state.x, 5,2);                                     ips200_show_float(X(10),  Y(4) ,p->current_state.y, 5,2);


}
/**
 * 函数说明：follow_points_show()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void follow_points_show(guandao_state * p)
{

    ips200_show_float(X(10),Y(10),p->recode_map[INS.current_point_index].x,3,2); ips200_show_float(X(15),Y(10),p->recode_map[INS.current_point_index].y,3,2);
    ips200_show_float(X(10),Y(11),p->current_state.x,3,2);                                         ips200_show_float(X(15),Y(11),p->current_state.y,3,2);
    ips200_show_int(X(10),  Y(15) ,p->current_point_index, 5);                                   ips200_show_int(X(15),  Y(15) ,p->length_index, 5);
    ips200_show_float(X(15),Y(16),p->current_state.theta,3,2);
}
/**
 * 函数说明：Key_Recode_Point()。记录当前位置/路线点，用于后续自动追踪。
 * 所属模块：科目一惯导路线记录、纯追踪和自动驾驶决策核心模块。
 * 参数说明：
 * - e：惯导路线/车辆状态结构体指针，保存当前位姿、路线点和追踪索引。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Key_Recode_Point(guandao_state * e)
{
    if(e->length_index >=MAX_LENGTH_INDEX) return;
    e->recode_map[e->length_index] =e->current_state;
    e->length_index  ++;


}


//void guandao_mode_recode(void)
//{
//    static uint8 flag0 = 1;
//    static uint8 flag1 = 1;
//    update_state(&INS,&guandao_ecd);
//    if(flag0){  guandao_state_init(&INS);   flag0 =0;}
//
//     recode_waypoint(&INS);
////     guandao_show();
//
//
////    if(gpio_get_level(SWITCH1)&&flag1){   Flash_Write_mappoints();  Buzzer_check(50);  flag1 = 0; };
//
//}

//void guandao_mode_trace(void)
//{
//
////    static uint8 flag2 = 1;
////    while(flag2){Guandao_Points_Show();ips200_clear();flag2 =0;}  //轨迹显示
//
//     update_state(&INS,&guandao_ecd);
//     pursuit_contral_mode(&INS ,&out_v_l ,&out_v_r ,&out_servo);
//     follow_points_show();
////        Steer_UpPID(&SteerUpPID ,out_servo);
//      Steer_PID(&SteerPID,out_servo);
//}
