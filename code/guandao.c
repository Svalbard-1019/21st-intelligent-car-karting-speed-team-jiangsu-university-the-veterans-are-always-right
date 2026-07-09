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
#include "auto_park_plan.h"
#include <string.h>

guandao_state INS;                               //0 = route_setting_choice
guandao_state passage;                    //1 = route_setting_choice
guandao_state portion_3;                     //2 = route_setting_choice
guandao_state portion_2;                    //3 = route_setting_choice

SLIP_Cheak slip_state = NONE;         // 打滑检测状态，初始为无打滑

uint8 route_setting_choice = 0;        // 路径选择标志（0-3）

float base_speed = 10.0f;
float persuit_threshold = 0.4f;         // 纯追踪阈值（到达目标点的距离容差）
float recode_threshold = 0.2f;         // 路径记录阈值
int16 preview_spets = 2;                  // 预瞄步数
float daoche_speed = -10.0;           //倒车速度
float final_dsts = 3.0f;                     // 终点距离减速阈值
float guandao_debug_distance = 0.0f;
float guandao_debug_angle_diff = 0.0f;
float guandao_debug_dist_final = 0.0f;
uint8 guandao_debug_stop_reason = 0;
uint8 guandao_debug_approach_active = 0;
uint8 guandao_debug_entry_gate = 0;
float guandao_debug_entry_long = 0.0f;
float guandao_debug_entry_lat = 0.0f;
float guandao_debug_entry_yaw = 0.0f;
int16 guandao_debug_steer_preview = 0;
int16 guandao_debug_curve_preview = 0;
float guandao_debug_upcoming_turn = 0.0f;
float guandao_debug_steer_raw = 0.0f;
float guandao_debug_steer_limited = 0.0f;
float guandao_debug_steer_final = 0.0f;

int16 daoche_target_length = 0;
state_t daoche_start_state = {0.0f, 0.0f, 0.0f};
uint8 daoche_start_flag = 0;
state_t daoche_target_state = {0.0f, 0.0f, 0.0f};
uint8 daoche_target_flag = 0;
int16 daoche_point_length = 0;    // 倒车点长度
uint8 daoche_flag =0;                    // 倒车标志
uint8 daoche_flash_cheack =0;// 倒车Flash检查标志
static uint8 park_record_stage = 0;
static uint32 park_start_record_ms = 0;
static uint8 guandao_record_init_pending = 1;
static uint8 portion1_state_flag = 0;
static uint16 portion1_finally_length = 0;
static uint8 portion1_reverse_state = 0;
static uint32 portion1_reverse_wait_start_ms = 0;
static uint32 portion1_reverse_run_start_ms = 0;
static state_t portion1_reverse_start_state = {0.0f, 0.0f, 0.0f};
static state_t portion1_reverse_segment_start_state = {0.0f, 0.0f, 0.0f};
static float portion1_reverse_steer_cmd = 0.0f;
static uint8 portion1_reverse_segment = 0;
static AutoParkPlan portion1_reverse_plan;
static uint8 portion1_reverse_plan_ready = 0;
static uint8 portion1_reverse_route_index = 0;
static state_t portion1_taught_reverse_map[MAX_LENGTH_INDEX];
static state_t portion1_taught_reverse_target = {0.0f, 0.0f, 0.0f};
static int16 portion1_taught_reverse_length = 0;
static int16 portion1_taught_reverse_index = 0;
static uint8 portion1_taught_reverse_ready = 0;
static uint32 portion1_taught_reverse_hold_ms = 0;
static uint32 portion1_taught_reverse_steer_ms = 0;
static uint8 portion1_approach_active = 0;
static float portion1_approach_steer_cmd = 0.0f;
static uint32 portion1_approach_steer_ms = 0;

static void guandao_record_park_target_now(guandao_state *state)
{
    if(state != &INS || !daoche_start_flag || daoche_target_flag) return;

    daoche_target_state = state->current_state;
    daoche_target_state.theta = Yaw_1;
    daoche_target_length = state->length_index;
    daoche_target_flag = 1;
    daoche_flash_cheack = 1;
    park_record_stage = 2;
}

#define GUANDAO_START_SEARCH_POINTS    10
#define GUANDAO_TRACE_SEARCH_POINTS    8
#define GUANDAO_REVERSE_TRIGGER_DIST   0.35f
#define GUANDAO_REVERSE_FINAL_DIST     0.55f
#define GUANDAO_REVERSE_MIN_ROUTE_POINTS 5
#define GUANDAO_PARK_ENTRY_DIST        0.15f
#define GUANDAO_PARK_APPROACH_DIST     1.20f
#define GUANDAO_PARK_APPROACH_SPEED_FAST 10.0f
#define GUANDAO_PARK_APPROACH_SPEED_MID  8.0f
#define GUANDAO_PARK_APPROACH_SPEED_SLOW 5.0f
#define GUANDAO_PARK_APPROACH_LAT_KP   18.0f
#define GUANDAO_PARK_APPROACH_YAW_KP   0.80f
#define GUANDAO_PARK_APPROACH_STEER_LIMIT 25.0f
#define GUANDAO_PARK_APPROACH_STEER_RATE 2.0f
#define GUANDAO_PARK_GATE_LAT_LIMIT    0.45f
#define GUANDAO_PARK_GATE_ABORT_LIMIT  1.20f
#define GUANDAO_PARK_GATE_NEAR_LONG    0.08f
#define GUANDAO_PARK_FINAL_GATE_LIMIT  0.45f
#define GUANDAO_PARK_FINAL_YAW_LIMIT   10.0f
#define GUANDAO_PARK_FRAME_POINTS      5
#define GUANDAO_REVERSE_WAIT_MS        300u
#define GUANDAO_REVERSE_GPS_WAIT_MS    1500u
#define GUANDAO_REVERSE_DISTANCE       0.55f
#define GUANDAO_REVERSE_MIN_MS         1800u
#define GUANDAO_REVERSE_MAX_MS         6500u
#define GUANDAO_REVERSE_SPEED_UNITS    -6.0f
#define GUANDAO_STEERING_GAIN          2.2f
#define GUANDAO_STEERING_CMD_LIMIT     40.0f
#define GUANDAO_HIGH_SPEED_THRESHOLD   5.0f
#define GUANDAO_HIGH_SPEED_GAIN        1.55f
#define GUANDAO_HIGH_SPEED_CMD_LIMIT   32.0f
#define GUANDAO_CURVE_SPEED_RATIO      0.70f
#define GUANDAO_VERY_HIGH_SPEED_GAIN   1.20f
#define GUANDAO_VERY_HIGH_CMD_LIMIT    35.0f
#define GUANDAO_STEER_RATE_LOW         3.0f
#define GUANDAO_STEER_RATE_HIGH        2.5f
#define GUANDAO_STEER_RATE_SHARP       6.0f
#define GUANDAO_CURVE_TRIGGER_ANGLE    35.0f
#define GUANDAO_SHARP_TURN_ANGLE       40.0f
#define GUANDAO_EARLY_TURN_LOOKAHEAD   28
#define GUANDAO_EARLY_TURN_ANGLE       28.0f
#define GUANDAO_EARLY_TURN_SPEED_RATIO 0.60f
#define GUANDAO_EARLY_STEER_GAIN       0.16f
#define GUANDAO_EARLY_STEER_MAX        8.0f
#define GUANDAO_EARLY_STEER_RATE       2.0f
#define GUANDAO_EARLY_STEER_FILTER     0.25f
#define GUANDAO_EARLY_TURN_FINAL_POINTS 70
#define GUANDAO_FRONT_TARGET_ANGLE     100.0f
#define GUANDAO_REVERSE_STEERING_GAIN  1.0f
#define GUANDAO_REVERSE_TARGET_DIST    0.12f
#define GUANDAO_REVERSE_TARGET_YAW     3.0f
#define GUANDAO_REVERSE_TARGET_KP_D    30.0f
#define GUANDAO_REVERSE_TARGET_KP_YAW  0.75f
#define GUANDAO_REVERSE_ENTRY_DIST     0.25f
#define GUANDAO_REVERSE_ENTRY_MS       900u
#define GUANDAO_REVERSE_FINE_DIST      0.35f
#define GUANDAO_REVERSE_FINE_SPEED     -4.0f
#define GUANDAO_REVERSE_FORWARD_SPEED  5.0f
#define GUANDAO_REVERSE_PLAN_TOL_DIST  0.16f
#define GUANDAO_REVERSE_PLAN_TOL_YAW   6.0f
#define GUANDAO_TAUGHT_REVERSE_SPEED   -4.0f
#define GUANDAO_TAUGHT_REVERSE_FINE_SPEED -3.0f
#define GUANDAO_TAUGHT_REVERSE_LOOKAHEAD 0.35f
#define GUANDAO_TAUGHT_REVERSE_POINT_DIST 0.20f
#define GUANDAO_TAUGHT_REVERSE_SEARCH  6
#define GUANDAO_TAUGHT_REVERSE_GAIN    1.40f
#define GUANDAO_TAUGHT_REVERSE_RATE    2.0f
#define GUANDAO_TAUGHT_REVERSE_HOLD_MS 300u
#define GUANDAO_PARK_GPS_SAMPLES        10
#define GUANDAO_PARK_GPS_SAMPLE_MS      100u
#define GUANDAO_PARK_GPS_MAX_ERROR      2.0f
#define GUANDAO_PARK_GPS_MAX_DISTANCE   5.0f
// 低速教学倒车的弯线路程明显长于起终点直线距离；实测 18 s 仍会在距目标约 0.23 m 时超时。
// 末端另有 12 cm 位置停车保护，因此延长运行时间而不放宽停车边界。
#define GUANDAO_TAUGHT_REVERSE_MAX_MS  34000u
#define GUANDAO_PARK_SECOND_MIN_MS      1000u
#define GUANDAO_PARK_SECOND_MIN_DIST    0.30f
#define GUANDAO_PARK_SECOND_MIN_POINTS  2
#define GUANDAO_AUTO_GPS_RECORD_DIST   1.0f
#define PORTION3_PURSUIT_THRESHOLD     0.25f
#define PORTION3_FINAL_STOP_DIST       0.6f
#define GUANDAO_SYSTEM_MS_WRAP         42950u

static float guandao_normalize_angle(float angle);
static uint32 guandao_elapsed_ms(uint32 now_ms, uint32 start_ms);

static uint8 portion1_park_gps_ready = 0;
static uint8 portion1_park_gps_count = 0;
static int16 portion1_park_gps_reference = -1;
static uint32 portion1_park_gps_sample_ms = 0;
static float portion1_park_gps_offset_x_sum = 0.0f;
static float portion1_park_gps_offset_y_sum = 0.0f;
static float portion1_park_gps_offset_x = 0.0f;
static float portion1_park_gps_offset_y = 0.0f;

static uint8 guandao_gps_to_ins_vector(int16 reference, double latitude,
        double longitude, float *ins_x, float *ins_y)
{
    int16 pair;
    int16 route_a;
    int16 route_b;
    double latitude_rad;
    float gps_e;
    float gps_n;
    float pair_e;
    float pair_n;
    float route_x;
    float route_y;
    float denominator;
    float scale;
    float a;
    float b;

    if(reference < 0 || reference >= INS.gps_recode_length) return 0;
    /* Use points about three metres apart; adjacent one-metre GNSS points make
     * the frame angle too sensitive to ordinary positioning noise. */
    pair = (reference >= 3) ? reference - 3 : reference + 3;
    if(pair < 0 || pair >= INS.gps_recode_length) return 0;
    route_a = INS.recode_gpsmap[reference].cheak_flag;
    route_b = INS.recode_gpsmap[pair].cheak_flag;
    if(route_a < 0 || route_a >= portion1_finally_length
            || route_b < 0 || route_b >= portion1_finally_length) return 0;

    latitude_rad = INS.recode_gpsmap[reference].lat * M_PI / 180.0;
    gps_e = (float)((INS.recode_gpsmap[reference].lon - longitude)
            * 111320.0 * cos(latitude_rad));
    gps_n = (float)((INS.recode_gpsmap[reference].lat - latitude) * 111320.0);
    pair_e = (float)((INS.recode_gpsmap[reference].lon - INS.recode_gpsmap[pair].lon)
            * 111320.0 * cos(latitude_rad));
    pair_n = (float)((INS.recode_gpsmap[reference].lat - INS.recode_gpsmap[pair].lat)
            * 111320.0);
    route_x = INS.recode_map[route_a].x - INS.recode_map[route_b].x;
    route_y = INS.recode_map[route_a].y - INS.recode_map[route_b].y;
    denominator = pair_e * pair_e + pair_n * pair_n;
    if(denominator < 2.25f) return 0;
    scale = sqrtf((route_x * route_x + route_y * route_y) / denominator);
    if(scale < 0.5f || scale > 1.5f) return 0;

    a = (route_x * pair_e + route_y * pair_n) / denominator;
    b = (route_y * pair_e - route_x * pair_n) / denominator;
    *ins_x = a * gps_e - b * gps_n;
    *ins_y = b * gps_e + a * gps_n;
    return 1;
}

static void guandao_park_gps_reset(void)
{
    portion1_park_gps_ready = 0;
    portion1_park_gps_count = 0;
    portion1_park_gps_reference = -1;
    portion1_park_gps_sample_ms = 0;
    portion1_park_gps_offset_x_sum = 0.0f;
    portion1_park_gps_offset_y_sum = 0.0f;
    portion1_park_gps_offset_x = 0.0f;
    portion1_park_gps_offset_y = 0.0f;
}

/* Each GNSS fix is paired with the simultaneous INS pose, so averaging does
 * not require stopping the car at the parking entrance. */
static void guandao_park_gps_update(void)
{
    int16 reference = -1;
    int16 route_index;
    int best_difference = 32767;
    float reference_dx;
    float reference_dy;
    float offset_x;
    float offset_y;
    uint32 now_ms;

    if(portion1_park_gps_ready || !daoche_start_flag || gnss.state != 1) return;
    if(INS.current_point_index + 20 < daoche_point_length) return;
    if(INS.gps_recode_length < 2) return;

    for(int16 i = 0; i < INS.gps_recode_length; i++)
    {
        int difference = INS.recode_gpsmap[i].cheak_flag - daoche_point_length;
        if(difference < 0) difference = -difference;
        if(difference < best_difference)
        {
            best_difference = difference;
            reference = i;
        }
    }
    if(reference < 0) return;
    route_index = INS.recode_gpsmap[reference].cheak_flag;
    if(route_index < 0 || route_index >= portion1_finally_length) return;

    now_ms = system_getval_ms();
    if(portion1_park_gps_sample_ms != 0
            && guandao_elapsed_ms(now_ms, portion1_park_gps_sample_ms) < GUANDAO_PARK_GPS_SAMPLE_MS) return;
    portion1_park_gps_sample_ms = now_ms;
    if(get_two_points_distance(gnss.latitude, gnss.longitude,
            INS.recode_gpsmap[reference].lat, INS.recode_gpsmap[reference].lon)
            > GUANDAO_PARK_GPS_MAX_DISTANCE) return;
    if(!guandao_gps_to_ins_vector(reference, gnss.latitude, gnss.longitude,
            &reference_dx, &reference_dy)) return;

    offset_x = INS.current_state.x + reference_dx - INS.recode_map[route_index].x;
    offset_y = INS.current_state.y + reference_dy - INS.recode_map[route_index].y;
    if(hypotf(offset_x, offset_y) > GUANDAO_PARK_GPS_MAX_ERROR) return;

    portion1_park_gps_reference = reference;
    portion1_park_gps_offset_x_sum += offset_x;
    portion1_park_gps_offset_y_sum += offset_y;
    portion1_park_gps_count++;
    if(portion1_park_gps_count >= GUANDAO_PARK_GPS_SAMPLES)
    {
        portion1_park_gps_offset_x = portion1_park_gps_offset_x_sum / portion1_park_gps_count;
        portion1_park_gps_offset_y = portion1_park_gps_offset_y_sum / portion1_park_gps_count;
        portion1_park_gps_ready = 1;
    }
}

uint8 guandao_park_gps_debug_ready(void) { return portion1_park_gps_ready; }
uint8 guandao_park_gps_debug_count(void) { return portion1_park_gps_count; }
int16 guandao_park_gps_debug_reference(void) { return portion1_park_gps_reference; }
float guandao_park_gps_debug_offset_x(void) { return portion1_park_gps_offset_x; }
float guandao_park_gps_debug_offset_y(void) { return portion1_park_gps_offset_y; }

// system_getval_ms() is derived from a 32-bit 10 ns counter and wraps about every 42.95 s.
// Plain uint32 subtraction on the divided millisecond value does not preserve that modulus.
static uint32 guandao_elapsed_ms(uint32 now_ms, uint32 start_ms)
{
    if(now_ms >= start_ms) return now_ms - start_ms;
    return (GUANDAO_SYSTEM_MS_WRAP - start_ms) + now_ms;
}

static int16 guandao_clamp_length(int16 length)
{
    if(length < 0) return 0;
    if(length > MAX_LENGTH_INDEX) return MAX_LENGTH_INDEX;
    return length;
}

static int16 guandao_route_length(guandao_state *state)
{
    if(state->plan_ready && state->planned_length > 0) return state->planned_length;
    return guandao_clamp_length(state->length_index);
}

static state_t guandao_route_point(guandao_state *state, int index)
{
    int16 route_length = guandao_route_length(state);
    if(route_length <= 0) return state->current_state;
    if(index < 0) index = 0;
    if(index >= route_length) index = route_length - 1;
    if(state->plan_ready && state->planned_length > 0) return state->planned_map[index];
    return state->recode_map[index];
}

// Build a local parking frame from the taught points before the first parking marker.
// dir points in the forward approach direction; right is its positive lateral direction.
static uint8 guandao_parking_entry_frame(float *dir_x, float *dir_y, float *heading)
{
    int16 reference_index;
    float dx;
    float dy;
    float length;

    if(!daoche_start_flag || daoche_point_length <= 0) return 0;
    reference_index = daoche_point_length - GUANDAO_PARK_FRAME_POINTS;
    if(reference_index < 0) reference_index = 0;
    dx = daoche_start_state.x - INS.recode_map[reference_index].x;
    dy = daoche_start_state.y - INS.recode_map[reference_index].y;
    length = hypotf(dx, dy);
    if(length < 0.05f)
    {
        *heading = daoche_start_state.theta;
        *dir_x = sinf(*heading / 180.0f * M_PI);
        *dir_y = cosf(*heading / 180.0f * M_PI);
        return 1;
    }

    *dir_x = dx / length;
    *dir_y = dy / length;
    *heading = atan2f(*dir_x, *dir_y) / M_PI * 180.0f;
    return 1;
}

static uint8 guandao_parking_entry_errors(float *longitudinal, float *lateral,
        float *yaw_error, uint8 *gate_passed)
{
    float dir_x;
    float dir_y;
    float heading;
    float position_x;
    float position_y;

    if(!guandao_parking_entry_frame(&dir_x, &dir_y, &heading)) return 0;
    position_x = INS.current_state.x - daoche_start_state.x;
    position_y = INS.current_state.y - daoche_start_state.y;
    *longitudinal = position_x * dir_x + position_y * dir_y;
    *lateral = position_x * dir_y - position_y * dir_x;
    *yaw_error = guandao_normalize_angle(heading - Yaw_1);
    *gate_passed = (*longitudinal >= 0.0f) ? 1 : 0;
    return 1;
}

static void guandao_parking_approach_update(float longitudinal, float lateral, float yaw_error)
{
    float speed = GUANDAO_PARK_APPROACH_SPEED_FAST;
    float target_steering;
    float desired_servo;
    float steer_delta;
    float steer_delta_limit;
    uint32 now_ms = system_getval_ms();
    uint32 elapsed_ms = (portion1_approach_steer_ms == 0) ? 20u
            : guandao_elapsed_ms(now_ms, portion1_approach_steer_ms);

    if(elapsed_ms > 100u) elapsed_ms = 20u;
    if(longitudinal > -0.30f) speed = GUANDAO_PARK_APPROACH_SPEED_SLOW;
    else if(longitudinal > -0.60f) speed = GUANDAO_PARK_APPROACH_SPEED_MID;

    target_steering = GUANDAO_PARK_APPROACH_YAW_KP * yaw_error
            - GUANDAO_PARK_APPROACH_LAT_KP * lateral;
    Value_Limit_float(&target_steering,
            -GUANDAO_PARK_APPROACH_STEER_LIMIT, GUANDAO_PARK_APPROACH_STEER_LIMIT);
    desired_servo = -target_steering;
    steer_delta = desired_servo - portion1_approach_steer_cmd;
    steer_delta_limit = GUANDAO_PARK_APPROACH_STEER_RATE * ((float)elapsed_ms / 20.0f);
    Value_Limit_float(&steer_delta, -steer_delta_limit, steer_delta_limit);
    portion1_approach_steer_cmd += steer_delta;
    Value_Limit_float(&portion1_approach_steer_cmd,
            -GUANDAO_PARK_APPROACH_STEER_LIMIT, GUANDAO_PARK_APPROACH_STEER_LIMIT);
    portion1_approach_steer_ms = now_ms;

    out_v_l = speed;
    out_v_r = speed;
    out_servo = portion1_approach_steer_cmd;
    guandao_debug_stop_reason = 12;
}

static int guandao_find_closest_index(guandao_state *state, int start_index, int end_index)
{
    int best_index = start_index;
    float best_distance = 0.0f;
    int16 route_length = guandao_route_length(state);

    if(route_length <= 0) return 0;
    if(start_index < 0) start_index = 0;
    if(end_index >= route_length) end_index = route_length - 1;
    if(start_index > end_index) return start_index;

    best_distance = get_distance(state->current_state, guandao_route_point(state, start_index));
    for(int i = start_index + 1; i <= end_index; i++)
    {
        float distance = get_distance(state->current_state, guandao_route_point(state, i));
        if(distance + 0.05f < best_distance)
        {
            best_distance = distance;
            best_index = i;
        }
    }

    return best_index;
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
    e->planned_length=0;
    e->plan_ready=0;

    e ->gps_recode_length =0;
    if(e == &INS)
    {
        daoche_start_state.x = 0.0f;
        daoche_start_state.y = 0.0f;
        daoche_start_state.theta = 0.0f;
        daoche_start_flag = 0;
    }

}

static float guandao_normalize_angle(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

static float guandao_segment_yaw(state_t from, state_t to)
{
    return atan2f(to.x - from.x, to.y - from.y) / M_PI * 180.0f;
}

static float guandao_max_route_turn(guandao_state *state, int start_index, int lookahead)
{
    float max_turn = 0.0f;
    int16 route_length = guandao_route_length(state);
    int end_index = start_index + lookahead;

    if(route_length < 3) return 0.0f;
    if(start_index < 1) start_index = 1;
    if(end_index > route_length - 2) end_index = route_length - 2;

    for(int i = start_index; i <= end_index; i++)
    {
        float yaw_in = guandao_segment_yaw(guandao_route_point(state, i - 1), guandao_route_point(state, i));
        float yaw_out = guandao_segment_yaw(guandao_route_point(state, i), guandao_route_point(state, i + 1));
        float turn = fabsf(guandao_normalize_angle(yaw_out - yaw_in));
        if(turn > max_turn) max_turn = turn;
    }

    return max_turn;
}

static float guandao_accum_route_turn(guandao_state *state, int start_index, int lookahead)
{
    float turn_sum = 0.0f;
    int16 route_length = guandao_route_length(state);
    int end_index = start_index + lookahead;

    if(route_length < 3) return 0.0f;
    if(start_index < 1) start_index = 1;
    if(end_index > route_length - 2) end_index = route_length - 2;

    for(int i = start_index; i <= end_index; i++)
    {
        float yaw_in = guandao_segment_yaw(guandao_route_point(state, i - 1), guandao_route_point(state, i));
        float yaw_out = guandao_segment_yaw(guandao_route_point(state, i), guandao_route_point(state, i + 1));
        turn_sum += fabsf(guandao_normalize_angle(yaw_out - yaw_in));
    }

    return turn_sum;
}

static float guandao_signed_accum_route_turn(guandao_state *state, int start_index, int lookahead)
{
    float turn_sum = 0.0f;
    int16 route_length = guandao_route_length(state);
    int end_index = start_index + lookahead;

    if(route_length < 3) return 0.0f;
    if(start_index < 1) start_index = 1;
    if(end_index > route_length - 2) end_index = route_length - 2;

    for(int i = start_index; i <= end_index; i++)
    {
        float yaw_in = guandao_segment_yaw(guandao_route_point(state, i - 1), guandao_route_point(state, i));
        float yaw_out = guandao_segment_yaw(guandao_route_point(state, i), guandao_route_point(state, i + 1));
        turn_sum += guandao_normalize_angle(yaw_out - yaw_in);
    }

    return turn_sum;
}

static int guandao_find_front_index(guandao_state *state, int start_index, int end_index)
{
    int best_index = start_index;
    float best_score = 1000000.0f;
    int16 route_length = guandao_route_length(state);

    if(route_length <= 0) return 0;
    if(start_index < 0) start_index = 0;
    if(end_index >= route_length) end_index = route_length - 1;
    if(start_index > end_index) return start_index;

    for(int i = start_index; i <= end_index; i++)
    {
        state_t point = guandao_route_point(state, i);
        float dx = point.x - state->current_state.x;
        float dy = point.y - state->current_state.y;
        float distance = hypotf(dx, dy);
        float angle_to_point = atan2f(dx, dy) / M_PI * 180.0f;
        float angle_error = fabsf(guandao_normalize_angle(angle_to_point - state->current_state.theta));
        float score = distance + angle_error * 0.015f;

        if(angle_error <= GUANDAO_FRONT_TARGET_ANGLE && score < best_score)
        {
            best_score = score;
            best_index = i;
        }
    }

    return best_index;
}

static AutoParkPose guandao_pose_from_state(state_t state)
{
    AutoParkPose pose;
    pose.x = state.x;
    pose.y = state.y;
    pose.heading = guandao_normalize_angle(state.theta);
    return pose;
}

static void guandao_reverse_prepare_plan(void)
{
    AutoParkPose start_pose = guandao_pose_from_state(INS.current_state);
    AutoParkPose target_pose = guandao_pose_from_state(daoche_target_state);

    portion1_reverse_plan_ready = 0;
    portion1_reverse_route_index = 0;
    memset(&portion1_reverse_plan, 0, sizeof(portion1_reverse_plan));

    if(daoche_target_flag && auto_park_build_plan(&start_pose, &target_pose, &portion1_reverse_plan))
    {
        portion1_reverse_plan_ready = 1;
        portion1_reverse_segment_start_state = INS.current_state;
    }
}

static uint8 guandao_reverse_plan_target_reached(void)
{
    float target_dist = get_distance(INS.current_state, daoche_target_state);
    float yaw_error = guandao_normalize_angle(daoche_target_state.theta - Yaw_1);
    return (target_dist <= GUANDAO_REVERSE_PLAN_TOL_DIST && fabsf(yaw_error) <= GUANDAO_REVERSE_PLAN_TOL_YAW);
}

static float guandao_reverse_route_finish_distance(const AutoParkRoute *route)
{
    if(route->is_straight || route->turn_radius <= 0.001f)
    {
        return route->distance;
    }

    return 2.0f * route->turn_radius * sinf(route->distance / (2.0f * route->turn_radius));
}

static uint8 guandao_reverse_execute_plan(void)
{
    AutoParkRoute *route = 0;
    float travelled = 0.0f;
    float finish_distance = 0.0f;
    float speed_abs = GUANDAO_REVERSE_FORWARD_SPEED;

    if(!portion1_reverse_plan_ready) return 0;

    while(portion1_reverse_route_index < (uint8)portion1_reverse_plan.route_count)
    {
        route = &portion1_reverse_plan.routes[portion1_reverse_route_index];
        travelled = get_distance(INS.current_state, portion1_reverse_segment_start_state);
        finish_distance = guandao_reverse_route_finish_distance(route);
        if(travelled + 0.03f < finish_distance) break;

        portion1_reverse_route_index++;
        portion1_reverse_segment_start_state = INS.current_state;
    }

    if(portion1_reverse_route_index >= (uint8)portion1_reverse_plan.route_count)
    {
        out_v_l = 0;
        out_v_r = 0;
        out_servo = 0;
        daoche_speed = 0;
        return 1;
    }

    route = &portion1_reverse_plan.routes[portion1_reverse_route_index];
    if(portion1_reverse_route_index + 1 >= (uint8)portion1_reverse_plan.route_count)
    {
        speed_abs = -GUANDAO_REVERSE_FINE_SPEED;
    }

    if(route->is_forward)
    {
        daoche_flag = 0;
        conrtol_mode = GUANDAO;
        out_v_l = speed_abs;
        out_v_r = speed_abs;
        out_servo = -route->steer_angle;
    }
    else
    {
        daoche_flag = 1;
        conrtol_mode = DAOCHE;
        daoche_speed = -speed_abs;
        out_v_l = daoche_speed;
        out_v_r = daoche_speed;
        out_servo = route->steer_angle;
    }

    guandao_debug_stop_reason = 9;
    finish_distance = guandao_reverse_route_finish_distance(route);
    guandao_debug_distance = finish_distance - travelled;
    if(guandao_debug_distance < 0.0f) guandao_debug_distance = 0.0f;
    guandao_debug_dist_final = get_distance(INS.current_state, daoche_target_state);
    return 0;
}

static void guandao_taught_reverse_prepare(void)
{
    int16 source_start;
    int16 search_end;
    int16 best_index;
    float best_distance;

    portion1_taught_reverse_ready = 0;
    portion1_taught_reverse_length = 0;
    portion1_taught_reverse_index = 0;
    portion1_taught_reverse_hold_ms = 0;
    portion1_taught_reverse_steer_ms = 0;
    portion1_approach_active = 0;
    portion1_approach_steer_cmd = 0.0f;
    portion1_approach_steer_ms = 0;

    if(!daoche_start_flag || !daoche_target_flag) return;
    if(daoche_point_length < GUANDAO_REVERSE_MIN_ROUTE_POINTS) return;
    if(daoche_target_length <= daoche_point_length + 1) return;
    if(daoche_target_length > portion1_finally_length) return;
    if(daoche_target_length >= MAX_LENGTH_INDEX) return;

    source_start = daoche_point_length;
    for(int16 source_index = source_start;
            source_index < daoche_target_length && portion1_taught_reverse_length < MAX_LENGTH_INDEX - 1;
            source_index++)
    {
        // Keep the taught route in its recorded INS frame. Moving the whole route to the
        // trigger pose also moves the final parking target by the same entry error.
        portion1_taught_reverse_map[portion1_taught_reverse_length]
                = INS.recode_map[source_index];
        portion1_taught_reverse_length++;
    }

    portion1_taught_reverse_target = daoche_target_state;
    portion1_taught_reverse_map[portion1_taught_reverse_length] = portion1_taught_reverse_target;
    portion1_taught_reverse_length++;

    if(portion1_park_gps_ready)
    {
        for(int16 i = 0; i < portion1_taught_reverse_length; i++)
        {
            portion1_taught_reverse_map[i].x += portion1_park_gps_offset_x;
            portion1_taught_reverse_map[i].y += portion1_park_gps_offset_y;
        }
        portion1_taught_reverse_target.x += portion1_park_gps_offset_x;
        portion1_taught_reverse_target.y += portion1_park_gps_offset_y;
    }

    if(portion1_taught_reverse_length >= 3)
    {
        search_end = GUANDAO_TAUGHT_REVERSE_SEARCH;
        if(search_end >= portion1_taught_reverse_length) search_end = portion1_taught_reverse_length - 1;
        best_index = 0;
        best_distance = get_distance(INS.current_state, portion1_taught_reverse_map[0]);
        for(int16 i = 1; i <= search_end; i++)
        {
            float distance = get_distance(INS.current_state, portion1_taught_reverse_map[i]);
            if(distance < best_distance)
            {
                best_distance = distance;
                best_index = i;
            }
        }
        portion1_taught_reverse_index = best_index;
        if(portion1_taught_reverse_index == 0) portion1_taught_reverse_index = 1;
        portion1_taught_reverse_ready = 1;
    }
}

static uint8 guandao_taught_reverse_update(void)
{
    state_t target;
    int16 search_end;
    int16 best_index;
    int16 lookahead_index;
    float best_distance;
    float target_distance;
    float final_distance;
    float final_yaw_error;
    uint8 final_gate_passed = 0;
    float target_angle;
    float motion_heading;
    float heading_error;
    float target_steering;
    float desired_servo;
    float steer_delta;
    float reverse_speed = GUANDAO_TAUGHT_REVERSE_SPEED;
    uint32 now_ms = system_getval_ms();
    uint32 steer_elapsed_ms;

    if(!portion1_taught_reverse_ready) return 0;

    search_end = portion1_taught_reverse_index + GUANDAO_TAUGHT_REVERSE_SEARCH;
    if(search_end >= portion1_taught_reverse_length) search_end = portion1_taught_reverse_length - 1;
    best_index = portion1_taught_reverse_index;
    best_distance = get_distance(INS.current_state, portion1_taught_reverse_map[best_index]);
    for(int16 i = portion1_taught_reverse_index + 1; i <= search_end; i++)
    {
        float distance = get_distance(INS.current_state, portion1_taught_reverse_map[i]);
        if(distance + 0.03f < best_distance)
        {
            best_distance = distance;
            best_index = i;
        }
    }
    if(best_index > portion1_taught_reverse_index) portion1_taught_reverse_index = best_index;

    while(portion1_taught_reverse_index < portion1_taught_reverse_length - 1
            && get_distance(INS.current_state, portion1_taught_reverse_map[portion1_taught_reverse_index])
                    <= GUANDAO_TAUGHT_REVERSE_POINT_DIST)
    {
        portion1_taught_reverse_index++;
    }

    lookahead_index = portion1_taught_reverse_index;
    target = portion1_taught_reverse_map[lookahead_index];
    while(lookahead_index < portion1_taught_reverse_length - 1
            && get_distance(INS.current_state, target) < GUANDAO_TAUGHT_REVERSE_LOOKAHEAD)
    {
        lookahead_index++;
        target = portion1_taught_reverse_map[lookahead_index];
    }

    target_distance = get_distance(INS.current_state, target);
    target_angle = atan2f(target.x - INS.current_state.x, target.y - INS.current_state.y) / M_PI * 180.0f;
    motion_heading = guandao_normalize_angle(Yaw_1 + 180.0f);
    heading_error = guandao_normalize_angle(target_angle - motion_heading);
    if(target_distance < 0.12f) target_distance = 0.12f;
    target_steering = GUANDAO_TAUGHT_REVERSE_GAIN
            * atan2f(2.0f * WHEEL_BASE * sinf(heading_error / 180.0f * M_PI), target_distance)
            / M_PI * 180.0f;
    Value_Limit_float(&target_steering, -GUANDAO_STEERING_CMD_LIMIT, GUANDAO_STEERING_CMD_LIMIT);

    desired_servo = -target_steering;
    steer_elapsed_ms = (portion1_taught_reverse_steer_ms == 0) ? 20u
            : guandao_elapsed_ms(now_ms, portion1_taught_reverse_steer_ms);
    if(steer_elapsed_ms > 100u) steer_elapsed_ms = 20u;
    steer_delta = desired_servo - portion1_reverse_steer_cmd;
    {
        float steer_delta_limit = GUANDAO_TAUGHT_REVERSE_RATE * ((float)steer_elapsed_ms / 20.0f);
        Value_Limit_float(&steer_delta, -steer_delta_limit, steer_delta_limit);
    }
    portion1_reverse_steer_cmd += steer_delta;
    Value_Limit_float(&portion1_reverse_steer_cmd,
            -GUANDAO_STEERING_CMD_LIMIT, GUANDAO_STEERING_CMD_LIMIT);
    portion1_taught_reverse_steer_ms = now_ms;

    final_distance = get_distance(INS.current_state, portion1_taught_reverse_target);
    final_yaw_error = guandao_normalize_angle(portion1_taught_reverse_target.theta - Yaw_1);
    if(portion1_taught_reverse_length > GUANDAO_PARK_FRAME_POINTS)
    {
        state_t final_reference = portion1_taught_reverse_map[
                portion1_taught_reverse_length - 1 - GUANDAO_PARK_FRAME_POINTS];
        float route_x = portion1_taught_reverse_target.x - final_reference.x;
        float route_y = portion1_taught_reverse_target.y - final_reference.y;
        float passed_x = INS.current_state.x - portion1_taught_reverse_target.x;
        float passed_y = INS.current_state.y - portion1_taught_reverse_target.y;
        if(route_x * route_x + route_y * route_y > 0.0001f
                && route_x * passed_x + route_y * passed_y >= 0.0f)
        {
            final_gate_passed = 1;
        }
    }
    if(final_distance <= GUANDAO_REVERSE_FINE_DIST
            || portion1_taught_reverse_index >= portion1_taught_reverse_length - 1)
    {
        reverse_speed = GUANDAO_TAUGHT_REVERSE_FINE_SPEED;
    }

    daoche_flag = 1;
    conrtol_mode = DAOCHE;
    daoche_speed = reverse_speed;
    out_v_l = reverse_speed;
    out_v_r = reverse_speed;
    out_servo = portion1_reverse_steer_cmd;
    guandao_debug_stop_reason = 10;
    guandao_debug_distance = get_distance(INS.current_state, target);
    guandao_debug_angle_diff = heading_error;
    guandao_debug_dist_final = final_distance;

    // 障碍物附近以停车位置优先：教学路径已经走到末端且进入 12 cm 范围就结束。
    // 航向未完全收敛时继续倒车可能越过目标并撞击立柱。
    if(portion1_taught_reverse_index >= portion1_taught_reverse_length - 2
            && final_distance <= GUANDAO_REVERSE_TARGET_DIST
            && fabsf(final_yaw_error) <= GUANDAO_PARK_FINAL_YAW_LIMIT)
    {
        return 1;
    }
    if(final_gate_passed
            && portion1_taught_reverse_index >= portion1_taught_reverse_length - 2
            && final_distance <= GUANDAO_PARK_FINAL_GATE_LIMIT
            && fabsf(final_yaw_error) <= GUANDAO_PARK_FINAL_YAW_LIMIT)
    {
        return 1;
    }

    if(final_distance <= GUANDAO_REVERSE_PLAN_TOL_DIST
            && fabsf(final_yaw_error) <= GUANDAO_REVERSE_PLAN_TOL_YAW)
    {
        if(portion1_taught_reverse_hold_ms == 0) portion1_taught_reverse_hold_ms = now_ms;
        if(guandao_elapsed_ms(now_ms, portion1_taught_reverse_hold_ms) >= GUANDAO_TAUGHT_REVERSE_HOLD_MS)
        {
            return 1;
        }
    }
    else
    {
        portion1_taught_reverse_hold_ms = 0;
    }

    return 0;
}

uint8 guandao_reverse_debug_state(void)
{
    return portion1_reverse_state;
}

uint8 guandao_reverse_debug_plan_ready(void)
{
    return portion1_taught_reverse_ready || portion1_reverse_plan_ready;
}

int16 guandao_reverse_debug_route_index(void)
{
    if(portion1_taught_reverse_ready) return portion1_taught_reverse_index;
    return (int16)portion1_reverse_route_index;
}

int16 guandao_reverse_debug_route_count(void)
{
    if(portion1_taught_reverse_ready) return portion1_taught_reverse_length;
    return portion1_reverse_plan.route_count;
}

float guandao_reverse_debug_target_distance(void)
{
    if(portion1_taught_reverse_ready)
    {
        return get_distance(INS.current_state, portion1_taught_reverse_target);
    }
    if(!daoche_target_flag) return -1.0f;
    return get_distance(INS.current_state, daoche_target_state);
}

float guandao_reverse_debug_target_yaw_error(void)
{
    if(portion1_taught_reverse_ready)
    {
        return guandao_normalize_angle(portion1_taught_reverse_target.theta - Yaw_1);
    }
    if(!daoche_target_flag) return 0.0f;
    return guandao_normalize_angle(daoche_target_state.theta - Yaw_1);
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
        state->current_state.theta =Yaw_1;
    }
    else
    {
        delta_real_center  = -(delta_real_l+delta_real_r)/2.0f;
        state->current_state.theta =Yaw_1+180.0f;
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
    portion1_state_flag = 0;
    portion1_finally_length = 0;
    portion1_reverse_state = 0;
    portion1_reverse_wait_start_ms = 0;
    portion1_reverse_run_start_ms = 0;
    portion1_reverse_start_state.x = 0.0f;
    portion1_reverse_start_state.y = 0.0f;
    portion1_reverse_start_state.theta = 0.0f;
    portion1_reverse_segment_start_state.x = 0.0f;
    portion1_reverse_segment_start_state.y = 0.0f;
    portion1_reverse_segment_start_state.theta = 0.0f;
    portion1_reverse_steer_cmd = 0.0f;
    portion1_reverse_segment = 0;
    portion1_reverse_plan_ready = 0;
    portion1_reverse_route_index = 0;
    portion1_taught_reverse_length = 0;
    portion1_taught_reverse_index = 0;
    portion1_taught_reverse_ready = 0;
    portion1_taught_reverse_hold_ms = 0;
    portion1_taught_reverse_steer_ms = 0;
    guandao_park_gps_reset();
    portion1_approach_active = 0;
    portion1_approach_steer_cmd = 0.0f;
    portion1_approach_steer_ms = 0;
    guandao_debug_approach_active = 0;
    guandao_debug_entry_gate = 0;
    guandao_debug_entry_long = 0.0f;
    guandao_debug_entry_lat = 0.0f;
    guandao_debug_entry_yaw = 0.0f;
    INS.length_index = guandao_clamp_length(INS.length_index);
    INS.current_point_index = 0;
    INS.planned_length = 0;
    INS.plan_ready = 0;
    daoche_flag = 0;
    out_v_l = 0;
    out_v_r = 0;
    out_servo = 0;
    INS.current_state.x = 0.0f;
    INS.current_state.y = 0.0f;
    INS.current_state.theta = 0.0f;
    Encoder_count_init(&guandao_ecd);
    Encoder_count_init(&Speed_ecd);
    encoder_clear_count(ENCODER_LEFT);
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
            // daoche_point_length 是停车点的数组下标，路线长度必须再加 1 才会包含该点。
            INS.length_index = daoche_point_length + 1;
            if(INS.length_index > portion1_finally_length)
            {
                INS.length_index = portion1_finally_length;
            }
        }
        else
        {
            INS.length_index = portion1_finally_length;            // 没有停车点时跑完整INS路线
        }
        /* Keep subject-one forward tracking identical to the proven kmy
         * baseline: follow the recorded points directly. */
        INS.plan_ready = 0;
        INS.planned_length = 0;
        INS.current_point_index = 0;
        if(INS.length_index > 1)
        {
            int end_index = INS.length_index - 1;
            if(end_index > GUANDAO_START_SEARCH_POINTS) end_index = GUANDAO_START_SEARCH_POINTS;
            INS.current_point_index = guandao_find_closest_index(&INS, 1, end_index);
        }
        portion1_state_flag = 1;
    }

    if(portion1_reverse_state == 1)
    {
        uint32 reverse_wait_ms = guandao_elapsed_ms(system_getval_ms(), portion1_reverse_wait_start_ms);
        out_v_l = 0;
        out_v_r = 0;
        out_servo = portion1_reverse_steer_cmd;
        guandao_debug_stop_reason = 6;
        conrtol_mode = GUANDAO;
        /* GPS correction is deliberately isolated here: the forward route has
         * already ended and both rear motors are stopped. */
        guandao_park_gps_update();
        if((portion1_park_gps_ready && reverse_wait_ms >= GUANDAO_REVERSE_WAIT_MS)
                || reverse_wait_ms >= GUANDAO_REVERSE_GPS_WAIT_MS)
        {
            portion1_reverse_start_state = INS.current_state;
            portion1_reverse_segment_start_state = INS.current_state;
            portion1_reverse_segment = 0;
            guandao_taught_reverse_prepare();
            if(portion1_taught_reverse_ready)
            {
                portion1_reverse_plan_ready = 0;
                portion1_reverse_route_index = 0;
            }
            else
            {
                guandao_reverse_prepare_plan();
            }
            portion1_reverse_run_start_ms = system_getval_ms();
            if(!portion1_taught_reverse_ready
                    && portion1_reverse_plan_ready && portion1_reverse_plan.route_count > 0
                    && portion1_reverse_plan.routes[0].is_forward)
            {
                daoche_flag = 0;
                conrtol_mode = GUANDAO;
            }
            else
            {
                daoche_flag = 1;
                conrtol_mode = DAOCHE;
            }
            portion1_reverse_state = 2;
        }
    }
    else if(portion1_reverse_state == 2)
    {
        uint8 reverse_finished = 0;
        uint32 reverse_elapsed_ms = guandao_elapsed_ms(system_getval_ms(), portion1_reverse_run_start_ms);
        uint32 reverse_max_ms = portion1_taught_reverse_ready
                ? GUANDAO_TAUGHT_REVERSE_MAX_MS : GUANDAO_REVERSE_MAX_MS;
        float reverse_travelled = get_distance(INS.current_state, portion1_reverse_start_state);
        daoche_speed = GUANDAO_REVERSE_SPEED_UNITS;
        guandao_debug_stop_reason = 7;
        conrtol_mode = DAOCHE;
        guandao_debug_dist_final = reverse_travelled;
        if(portion1_taught_reverse_ready)
        {
            if(guandao_taught_reverse_update())
            {
                reverse_finished = 1;
            }
            else if(reverse_elapsed_ms < reverse_max_ms)
            {
                follow_points_show(&INS);
                return;
            }
        }
        if(!portion1_taught_reverse_ready && portion1_reverse_plan_ready)
        {
            if(guandao_reverse_execute_plan())
            {
                if(daoche_target_flag && !guandao_reverse_plan_target_reached())
                {
                    portion1_reverse_plan_ready = 0;
                    portion1_reverse_segment = 1;
                    portion1_reverse_start_state = INS.current_state;
                    portion1_reverse_steer_cmd = 0.0f;
                    daoche_flag = 1;
                    conrtol_mode = DAOCHE;
                }
                else
                {
                    reverse_finished = 1;
                }
            }
            else if(reverse_elapsed_ms < GUANDAO_REVERSE_MAX_MS)
            {
                follow_points_show(&INS);
                return;
            }
        }
        if(!portion1_taught_reverse_ready && daoche_target_flag)
        {
            float target_dx = daoche_target_state.x - INS.current_state.x;
            float target_dy = daoche_target_state.y - INS.current_state.y;
            float target_dist = hypotf(target_dx, target_dy);
            float target_angle = atan2f(target_dx, target_dy) / M_PI * 180.0f;
            float position_error = target_angle - INS.current_state.theta;
            float yaw_error = daoche_target_state.theta - Yaw_1;
            angle_plan(&position_error);
            angle_plan(&yaw_error);
            guandao_debug_dist_final = target_dist;

            if(portion1_reverse_segment == 0
                    && (reverse_travelled >= GUANDAO_REVERSE_ENTRY_DIST
                        || reverse_elapsed_ms >= GUANDAO_REVERSE_ENTRY_MS))
            {
                portion1_reverse_segment = 1;
            }
            if(portion1_reverse_segment >= 1 && target_dist <= GUANDAO_REVERSE_FINE_DIST)
            {
                portion1_reverse_segment = 2;
            }

            if(portion1_reverse_segment >= 1)
            {
                portion1_reverse_steer_cmd = -(GUANDAO_REVERSE_TARGET_KP_D * sinf(position_error / 180.0f * M_PI)
                        + GUANDAO_REVERSE_TARGET_KP_YAW * yaw_error);
                Value_Limit_float(&portion1_reverse_steer_cmd, -GUANDAO_STEERING_CMD_LIMIT, GUANDAO_STEERING_CMD_LIMIT);
            }
            if(portion1_reverse_segment == 2)
            {
                daoche_speed = GUANDAO_REVERSE_FINE_SPEED;
            }
            if(target_dist <= GUANDAO_REVERSE_TARGET_DIST && fabsf(yaw_error) <= GUANDAO_REVERSE_TARGET_YAW
                    && reverse_elapsed_ms >= GUANDAO_REVERSE_MIN_MS)
            {
                reverse_finished = 1;
            }
        }
        else if(!portion1_taught_reverse_ready
                && reverse_travelled >= GUANDAO_REVERSE_DISTANCE
                && reverse_elapsed_ms >= GUANDAO_REVERSE_MIN_MS)
        {
            reverse_finished = 1;
        }
        out_v_l = daoche_speed;
        out_v_r = daoche_speed;
        out_servo = portion1_reverse_steer_cmd;
        if(reverse_finished || reverse_elapsed_ms >= reverse_max_ms)
        {
            uint8 reverse_timeout = (!reverse_finished && reverse_elapsed_ms >= reverse_max_ms);
            out_v_l = 0;
            out_v_r = 0;
            out_servo = 0;
            daoche_flag = 0;
            conrtol_mode = IDLE;
            portion1_reverse_state = reverse_timeout ? 4 : 3;
            guandao_debug_stop_reason = reverse_timeout ? 11 : 8;
            rear_motor_stop();
            Buzzer_check(50);
        }
    }
    else if(portion1_reverse_state == 3)
    {
        out_v_l = 0;
        out_v_r = 0;
        out_servo = 0;
        guandao_debug_stop_reason = 8;
        conrtol_mode = IDLE;
    }
    else if(portion1_reverse_state == 4)
    {
        out_v_l = 0;
        out_v_r = 0;
        out_servo = 0;
        guandao_debug_stop_reason = 11;
        conrtol_mode = IDLE;
    }
    else
    {
        int16 active_route_length = 0;
        uint8 reverse_ready = 0;
        uint8 final_stop_ready = 0;
        uint8 reverse_route_valid = 0;
        uint8 entry_gate_passed = 0;
        float entry_longitudinal = 0.0f;
        float entry_lateral = 0.0f;
        float entry_yaw_error = 0.0f;
        pursuit_contral_mode(&INS ,&out_v_l ,&out_v_r ,&out_servo);
        active_route_length = guandao_route_length(&INS);
        reverse_route_valid = (daoche_point_length >= GUANDAO_REVERSE_MIN_ROUTE_POINTS
                && active_route_length >= GUANDAO_REVERSE_MIN_ROUTE_POINTS
                && daoche_point_length < portion1_finally_length);
        if(reverse_route_valid
                && guandao_parking_entry_errors(&entry_longitudinal, &entry_lateral,
                        &entry_yaw_error, &entry_gate_passed))
        {
            guandao_debug_entry_long = entry_longitudinal;
            guandao_debug_entry_lat = entry_lateral;
            guandao_debug_entry_yaw = entry_yaw_error;
            guandao_debug_entry_gate = entry_gate_passed;
            guandao_debug_dist_final = get_distance(INS.current_state, daoche_start_state);

            if(!portion1_approach_active
                    && INS.current_point_index >= active_route_length - 30
                    && entry_longitudinal >= -GUANDAO_PARK_APPROACH_DIST)
            {
                portion1_approach_active = 1;
                portion1_approach_steer_cmd = out_servo;
                portion1_approach_steer_ms = system_getval_ms();
            }
            guandao_debug_approach_active = portion1_approach_active;
            if(portion1_approach_active)
            {
                if((entry_gate_passed && fabsf(entry_lateral) <= GUANDAO_PARK_GATE_LAT_LIMIT)
                        || (fabsf(entry_longitudinal) <= GUANDAO_PARK_GATE_NEAR_LONG
                            && fabsf(entry_lateral) <= 0.30f
                            && fabsf(entry_yaw_error) <= 15.0f))
                {
                    reverse_ready = 1;
                }
                else if(entry_gate_passed
                        && fabsf(entry_lateral) > GUANDAO_PARK_GATE_ABORT_LIMIT)
                {
                    // The car crossed the entry line outside the safe corridor. Stop instead of
                    // turning around to chase the point or starting a badly shifted reverse run.
                    out_v_l = 0.0f;
                    out_v_r = 0.0f;
                    out_servo = 0.0f;
                    guandao_debug_stop_reason = 13;
                }
                else
                {
                    guandao_parking_approach_update(entry_longitudinal,
                            entry_lateral, entry_yaw_error);
                }
            }
        }
        if(!reverse_route_valid
                && active_route_length >= GUANDAO_REVERSE_MIN_ROUTE_POINTS
                && INS.current_point_index >= active_route_length - 2
                && guandao_debug_dist_final <= GUANDAO_REVERSE_FINAL_DIST)
        {
            final_stop_ready = 1;
        }
        if(reverse_ready || final_stop_ready)
        {
            if(reverse_ready)
            {
                portion1_reverse_steer_cmd = out_servo * GUANDAO_REVERSE_STEERING_GAIN;
                Value_Limit_float(&portion1_reverse_steer_cmd, -GUANDAO_STEERING_CMD_LIMIT, GUANDAO_STEERING_CMD_LIMIT);
            }
            out_v_l = 0;
            out_v_r = 0;
            out_servo = 0;
            if(reverse_ready)
            {
                portion1_reverse_wait_start_ms = system_getval_ms();
                portion1_reverse_state = 1;
                conrtol_mode = GUANDAO;
                Buzzer_check(30);
            }
            else
            {
                guandao_debug_stop_reason = 4;
                conrtol_mode = GUANDAO;
            }
        }
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
    static uint8 rc_ch4_last_pressed = 0;
    static uint8 rc_ch4_armed = 0;
    static uint8 gps_auto_has_point = 0;
    static state_t gps_auto_last_state = {0.0f, 0.0f, 0.0f};
    uint8 auto_gps_enabled = (state == &INS || state == &portion_3);
    uint8 rc_ch4_pressed = (x6f_out[3] == 200);
    uint8 park_pressed;

    /* Ignore a CH4 high level already present when record mode starts.  The
     * operator must release CH4 once before a new rising edge can mark a
     * parking point; the physical KEY4 remains immediately available. */
    if(!rc_ch4_pressed) rc_ch4_armed = 1;
    park_pressed = (key4_flag == 1
            || (rc_ch4_armed && rc_ch4_pressed && !rc_ch4_last_pressed));
    if(rc_ch4_armed && rc_ch4_pressed && !rc_ch4_last_pressed) rc_ch4_armed = 0;
    rc_ch4_last_pressed = rc_ch4_pressed;
    if(state ->length_index >=MAX_LENGTH_INDEX)return;

    if(state ->length_index ==0)
    {
        if(auto_gps_enabled)
        {
            park_record_stage = 0;
            park_start_record_ms = 0;
            rc_ch4_last_pressed = 0;
            rc_ch4_armed = rc_ch4_pressed ? 0 : 1;
            gps_auto_has_point = 0;
        }
        state->recode_map[state->length_index] =state->current_state;
        state->length_index++;
        if(auto_gps_enabled && GPS_WORK_FLAG && recode_gps(state))
        {
            gps_auto_last_state = state->current_state;
            gps_auto_has_point = 1;
        }
        return;
    }

    state_t last_recoded =  state->recode_map[state->length_index-1];
    float dist = get_distance(state->current_state, last_recoded );

    if(dist >=recode_threshold)
    {
        state->recode_map[state->length_index] =state->current_state;
        state->length_index++;
        if(auto_gps_enabled && GPS_WORK_FLAG
                && (!gps_auto_has_point || get_distance(state->current_state, gps_auto_last_state) >= GUANDAO_AUTO_GPS_RECORD_DIST)
                && recode_gps(state))
        {
            gps_auto_last_state = state->current_state;
            gps_auto_has_point = 1;
        }
    }

    if(state->length_index >= MAX_LENGTH_INDEX)return;
    if(state == &INS && park_pressed)
    {
        key4_flag =0;
        if(park_record_stage == 0)
        {
            state->recode_map[state->length_index] =state->current_state;
            daoche_start_state = state->current_state;
            daoche_start_state.theta = Yaw_1;
            daoche_start_flag = 1;
            daoche_point_length = state->length_index;
            state->length_index++;
            daoche_flag =1;
            daoche_flash_cheack =1;
            park_record_stage = 1;
            park_start_record_ms = system_getval_ms();
            Buzzer_check(30);
        }
        else if(park_record_stage == 1
                && guandao_elapsed_ms(system_getval_ms(), park_start_record_ms) >= GUANDAO_PARK_SECOND_MIN_MS
                && state->length_index >= daoche_point_length + GUANDAO_PARK_SECOND_MIN_POINTS
                && get_distance(state->current_state, daoche_start_state) >= GUANDAO_PARK_SECOND_MIN_DIST)
        {
            daoche_target_state = state->current_state;
            daoche_target_state.theta = Yaw_1;
            daoche_target_length = state->length_index;
            daoche_target_flag = 1;
            daoche_flash_cheack = 1;
            park_record_stage = 2;
            Buzzer_check(30);
        }
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
void guandao_build_smooth_plan(guandao_state * state)
{
    int16 source_length = guandao_clamp_length(state->length_index);
    state->planned_length = 0;
    state->plan_ready = 0;

    if(source_length <= 0) return;
    if(source_length < 3)
    {
        for(int i = 0; i < source_length; i++)
        {
            state->planned_map[i] = state->recode_map[i];
        }
        state->planned_length = source_length;
        state->plan_ready = 1;
        return;
    }

    int samples = (MAX_LENGTH_INDEX - 1) / (source_length - 1);
    if(samples < 1) samples = 1;
    if(samples > 6) samples = 6;

    for(int i = 0; i < source_length - 1 && state->planned_length < MAX_LENGTH_INDEX - 1; i++)
    {
        state_t p0 = state->recode_map[(i > 0) ? i - 1 : i];
        state_t p1 = state->recode_map[i];
        state_t p2 = state->recode_map[i + 1];
        state_t p3 = state->recode_map[(i + 2 < source_length) ? i + 2 : i + 1];
        float turn_in = fabsf(guandao_normalize_angle(guandao_segment_yaw(p0, p1) - guandao_segment_yaw(p1, p2)));
        float turn_out = fabsf(guandao_normalize_angle(guandao_segment_yaw(p1, p2) - guandao_segment_yaw(p2, p3)));
        uint8 keep_corner_linear = (turn_in >= GUANDAO_SHARP_TURN_ANGLE || turn_out >= GUANDAO_SHARP_TURN_ANGLE);

        for(int j = 0; j < samples && state->planned_length < MAX_LENGTH_INDEX - 1; j++)
        {
            float t = (float)j / (float)samples;
            float t2 = t * t;
            float t3 = t2 * t;
            state_t out;
            if(keep_corner_linear)
            {
                out.x = p1.x + (p2.x - p1.x) * t;
                out.y = p1.y + (p2.y - p1.y) * t;
            }
            else
            {
                out.x = 0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t + (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 + (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3);
                out.y = 0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t + (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 + (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3);
            }
            out.theta = p1.theta + (p2.theta - p1.theta) * t;
            state->planned_map[state->planned_length] = out;
            state->planned_length++;
        }
    }

    state->planned_map[state->planned_length] = state->recode_map[source_length - 1];
    state->planned_length++;
    state->plan_ready = 1;
}

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
    float steering_gain = GUANDAO_STEERING_GAIN;
    float steering_limit = GUANDAO_STEERING_CMD_LIMIT;
    float steering_rate_limit = GUANDAO_STEER_RATE_LOW;
    static float last_target_steering = 0.0f;
    static float early_steer_filtered = 0.0f;
    static uint32 last_steer_limit_ms = 0;
    int steer_preview_steps = preview_spets;
    int curve_preview_steps = 5;
    int16 route_length = guandao_route_length(state);
    float arrive_threshold = persuit_threshold;
    float upcoming_turn = 0.0f;
    float local_turn = 0.0f;
    float early_turn = 0.0f;
    float early_turn_signed = 0.0f;
    float dist_to_final = 0.0f;

    guandao_debug_stop_reason = 0;
    guandao_debug_steer_preview = 0;
    guandao_debug_curve_preview = 0;
    guandao_debug_upcoming_turn = 0.0f;
    guandao_debug_steer_raw = 0.0f;
    guandao_debug_steer_limited = 0.0f;
    guandao_debug_steer_final = 0.0f;
    if(route_length == 0 || state->current_point_index == route_length)
    {
        guandao_debug_stop_reason = 1;
        * out_v_l = 0;
        * out_v_r = 0;
        *out_servo = 0;
        last_target_steering = 0.0f;
        early_steer_filtered = 0.0f;
        last_steer_limit_ms = 0;
//        Buzzer_check(50);
        return;
    }

    state_t current_point = state->current_state;
    if(state->current_point_index < 0) state->current_point_index = 0;
    if(state->current_point_index >= route_length) state->current_point_index = route_length - 1;
    if(route_setting_choice == 2)
    {
        arrive_threshold = PORTION3_PURSUIT_THRESHOLD;
    }

    int search_end_index = state->current_point_index + GUANDAO_TRACE_SEARCH_POINTS;
    if(search_end_index >= route_length) search_end_index = route_length - 1;
    int closest_index = guandao_find_closest_index(state, state->current_point_index, search_end_index);
    if(closest_index > state->current_point_index)
    {
        state->current_point_index = closest_index;
        guandao_debug_stop_reason = 5;
    }

    state_t target_point = guandao_route_point(state, state->current_point_index);

    // 科目一第一停车点必须精确到达；中间路线点仍使用较大的追踪阈值保证流畅。
    if(state == &INS
            && daoche_point_length > 0
            && daoche_point_length < portion1_finally_length
            && state->current_point_index >= route_length - 1)
    {
        arrive_threshold = GUANDAO_PARK_ENTRY_DIST;
    }

    float dx = target_point.x - current_point.x;
    float dy = target_point.y - current_point.y;
    float distance_to_target = hypotf(dx, dy);
    guandao_debug_distance = distance_to_target;
    float angle_to_target = atan2f(dx,dy)/M_PI*180.0f;
    float angle_diff = angle_to_target - state->current_state.theta;

    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    if(fabsf(angle_diff) > GUANDAO_FRONT_TARGET_ANGLE)
    {
        int front_index = guandao_find_front_index(state, state->current_point_index, search_end_index);
        if(front_index > state->current_point_index)
        {
            state->current_point_index = front_index;
            target_point = guandao_route_point(state, state->current_point_index);
            dx = target_point.x - current_point.x;
            dy = target_point.y - current_point.y;
            distance_to_target = hypotf(dx, dy);
            angle_to_target = atan2f(dx,dy)/M_PI*180.0f;
            angle_diff = guandao_normalize_angle(angle_to_target - state->current_state.theta);
            guandao_debug_stop_reason = 5;
        }
    }
    guandao_debug_angle_diff = angle_diff;

    // 只允许“距离足够近”时切到下一个路线点。
    // 旧逻辑曾用 |angle_diff| > 90 直接跳点，车头方向一反就会瞬间跳到终点并停车。
    if(distance_to_target <= arrive_threshold)
    {
        if(guandao_debug_stop_reason == 0) guandao_debug_stop_reason = 2;
        state->current_point_index++;
        if(state->current_point_index >= route_length )
        {   state->current_point_index = route_length;
            guandao_debug_stop_reason = 4;
            * out_v_l = 0;
            * out_v_r = 0;
            *out_servo = 0;
            last_target_steering = 0.0f;
            early_steer_filtered = 0.0f;
            last_steer_limit_ms = 0;
//            Buzzer_check(50);
            return;
        }
    }

    // preview_spets 决定转向预瞄点，数值越大越平滑，但弯道响应会更慢。
    if(base_speed > GUANDAO_HIGH_SPEED_THRESHOLD)
    {
        steering_gain = GUANDAO_HIGH_SPEED_GAIN;
        steering_limit = GUANDAO_HIGH_SPEED_CMD_LIMIT;
        if(base_speed >= 15.0f)
        {
            steering_gain = GUANDAO_VERY_HIGH_SPEED_GAIN;
            steering_limit = GUANDAO_VERY_HIGH_CMD_LIMIT;
            if(steer_preview_steps < 8) steer_preview_steps = 8;
            steering_rate_limit = GUANDAO_STEER_RATE_HIGH;
        }
        else if(base_speed >= 10.0f)
        {
            if(steer_preview_steps < 4) steer_preview_steps = 4;
        }
    }
    local_turn = guandao_max_route_turn(state, state->current_point_index, 18);
    upcoming_turn = local_turn;
    early_turn = guandao_accum_route_turn(state, state->current_point_index, GUANDAO_EARLY_TURN_LOOKAHEAD);
    early_turn_signed = guandao_signed_accum_route_turn(state, state->current_point_index, GUANDAO_EARLY_TURN_LOOKAHEAD);
    if(early_turn > upcoming_turn) upcoming_turn = early_turn;
    if(local_turn >= GUANDAO_SHARP_TURN_ANGLE)
    {
        if(steer_preview_steps > 3) steer_preview_steps = 3;
        if(curve_preview_steps > 6) curve_preview_steps = 6;
        steering_rate_limit = GUANDAO_STEER_RATE_SHARP;
        steering_limit = GUANDAO_STEERING_CMD_LIMIT;
    }
    if(curve_preview_steps < steer_preview_steps + 3)
    {
        curve_preview_steps = steer_preview_steps + 3;
    }
    guandao_debug_steer_preview = steer_preview_steps;
    guandao_debug_curve_preview = curve_preview_steps;
    guandao_debug_upcoming_turn = upcoming_turn;
    dist_to_final = get_distance(state->current_state, guandao_route_point(state, route_length - 1));

    pursuit_midhandle(state , &current_point , steer_preview_steps , &preview_alpha , &actual_ld);
    pursuit_midhandle(state , &current_point , curve_preview_steps , &preview_alpha2 , &actual_ld2);

   float k = 0;
   k = -0.0038*fabs(preview_alpha2) + 1;
   Value_Limit_float(&k ,0.6 ,1);

   if(angle_diff >=90)
   {
       target_steering = steering_limit;

   }
   else if(angle_diff <= -90)
   {
       target_steering = -steering_limit;
   }
   else if(fabsf(angle_diff) <=90)
   {
       target_steering = steering_gain*atan2f(2.0f * WHEEL_BASE * sinf(preview_alpha/180.0f*M_PI), actual_ld)/M_PI*180.0f;
   }
   float early_steer_target = 0.0f;
   if(base_speed >= 15.0f && state->current_point_index < route_length - GUANDAO_EARLY_TURN_FINAL_POINTS
           && early_turn > GUANDAO_EARLY_TURN_ANGLE)
   {
       early_steer_target = early_turn_signed * GUANDAO_EARLY_STEER_GAIN;
       Value_Limit_float(&early_steer_target, -GUANDAO_EARLY_STEER_MAX, GUANDAO_EARLY_STEER_MAX);
   }
   float early_steer_delta = early_steer_target - early_steer_filtered;
   Value_Limit_float(&early_steer_delta, -GUANDAO_EARLY_STEER_RATE, GUANDAO_EARLY_STEER_RATE);
   early_steer_filtered += early_steer_delta * GUANDAO_EARLY_STEER_FILTER;
   if(fabsf(early_steer_target) < 0.1f && fabsf(early_steer_filtered) < 0.3f)
   {
       early_steer_filtered = 0.0f;
   }
   target_steering += early_steer_filtered;
   guandao_debug_steer_raw = target_steering;
   ips200_show_float(X(10),  Y(9),target_steering ,5 ,5);

   //限幅
   Value_Limit_float(&target_steering ,-MAX_STEERING_RAD,MAX_STEERING_RAD);

//   slip_cheak(&guandao_ecd,target_steering);

   guandao_debug_dist_final = dist_to_final;
   float v_center = base_speed;


   switch(route_setting_choice)
   {
       case 0:
           azimuth_adjust(state , 1.3 , dist_to_final , &target_steering ,CORRECT_ANGLE_1);
           break;
       case 2:
           break;
       case 3:

           break;
       default : break;
   }

    Value_Limit_float(&target_steering ,-steering_limit,steering_limit);
    guandao_debug_steer_limited = target_steering;

   if(fabsf(preview_alpha2) > GUANDAO_CURVE_TRIGGER_ANGLE)
   {
       float curve_scale = 1.0f - (fabsf(preview_alpha2) - GUANDAO_CURVE_TRIGGER_ANGLE) / 90.0f;
       Value_Limit_float(&curve_scale, 0.55f, 1.0f);
       v_center = base_speed * curve_scale;
       if(v_center < MIN_SPEED) v_center = MIN_SPEED;
   }
   if(base_speed >= 15.0f && (fabsf(angle_diff) > 25.0f || fabsf(preview_alpha2) > GUANDAO_CURVE_TRIGGER_ANGLE))
   {
       if(v_center > base_speed * GUANDAO_CURVE_SPEED_RATIO)
       {
           v_center = base_speed * GUANDAO_CURVE_SPEED_RATIO;
       }
   }
   if(base_speed >= 15.0f && state->current_point_index < route_length - GUANDAO_EARLY_TURN_FINAL_POINTS
           && early_turn > GUANDAO_EARLY_TURN_ANGLE)
   {
       if(v_center > base_speed * GUANDAO_EARLY_TURN_SPEED_RATIO)
       {
           v_center = base_speed * GUANDAO_EARLY_TURN_SPEED_RATIO;
       }
   }
   else if(base_speed > GUANDAO_HIGH_SPEED_THRESHOLD && (fabsf(angle_diff) > 30.0f || fabsf(preview_alpha2) > GUANDAO_CURVE_TRIGGER_ANGLE))
   {
       if(v_center > base_speed * 0.75f)
       {
           v_center = base_speed * 0.75f;
       }
   }

   if (dist_to_final < final_dsts && state->current_point_index >= route_length - 30)
   {

       arrive_threshold = persuit_threshold*(dist_to_final / final_dsts);
       if(arrive_threshold < 0.3f){arrive_threshold = 0.3f;}
       v_center = base_speed * (dist_to_final / final_dsts);
       if (v_center < MIN_SPEED) v_center = MIN_SPEED; // 最低速度限制
   }
   if(route_setting_choice == 2 && state->current_point_index >= route_length - 1
           && dist_to_final <= PORTION3_FINAL_STOP_DIST)
   {
       state->current_point_index = route_length;
       guandao_debug_stop_reason = 8;
       * out_v_l = 0;
       * out_v_r = 0;
       *out_servo = 0;
       last_target_steering = 0.0f;
       early_steer_filtered = 0.0f;
       last_steer_limit_ms = 0;
       return;
   }

   // 差动驱动速度分配：这里仍是惯导旧速度单位，cpu0_main.c 会再换算为 m/s 给 rear_motor。
   if(state->current_point_index <= 2)
   {
       last_target_steering = target_steering;
       last_steer_limit_ms = system_getval_ms();
   }
   uint32 steer_now_ms = system_getval_ms();
   uint32 steer_elapsed_ms = (last_steer_limit_ms == 0) ? 20u
           : guandao_elapsed_ms(steer_now_ms, last_steer_limit_ms);
   if(steer_elapsed_ms > 100u) steer_elapsed_ms = 20u;
   float steer_delta_limit = steering_rate_limit * ((float)steer_elapsed_ms / 20.0f);
   float steer_delta = target_steering - last_target_steering;
   Value_Limit_float(&steer_delta, -steer_delta_limit, steer_delta_limit);
    target_steering = last_target_steering + steer_delta;
    Value_Limit_float(&target_steering ,-steering_limit,steering_limit);
    guandao_debug_steer_final = target_steering;
   last_target_steering = target_steering;
   last_steer_limit_ms = steer_now_ms;

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
    int16 route_length = guandao_route_length(state);
    if(dist_to_final < start_d && state->current_point_index >= route_length - 30)
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
    int16 route_length = guandao_route_length(state);
    if(route_length <= 0)
    {
        *angle = 0.0f;
        *distanse = 0.1f;
        return;
    }
    if(main_mode == Guandao_Recode_Mode)
    {
        preview_index = route_length - 1;
    }
    else
    {
        preview_index = state->current_point_index+index;
    }

   if(preview_index >= route_length)preview_index = route_length - 1;

   state_t preview_point = guandao_route_point(state, preview_index);

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
    static uint32 key1_save_start_ms = 0;
    static uint8 key1_save_wait_release = 0;
    static uint32 rc_ch3_start_ms = 0;
    static uint8 rc_ch3_wait_release = 0;
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

    if(guandao_record_init_pending)
    {
        guandao_state_init(p);
        daoche_point_length = 0;
        daoche_start_flag = 0;
        daoche_target_length = 0;
        daoche_target_flag = 0;
        daoche_flash_cheack = 0;
        park_record_stage = 0;
        park_start_record_ms = 0;
        guandao_record_init_pending = 0;
    }
    update_state(p  , &guandao_ecd);
    gps_recode_average_update(p);


    // KEY4 is used instead of the broken KEY1: released=1, pressed=0.
    // Hold longer than 1.5s to save; short release marks the parking point.
    if(gpio_get_level(KEY4) == 0)
    {
        key4_flag = 0;
        now_ms = system_getval_ms();
        if(key1_save_start_ms == 0) key1_save_start_ms = now_ms;
        if(guandao_elapsed_ms(now_ms, key1_save_start_ms) > 1500 && !key1_save_wait_release)
        {
            // 第二次按键若直接长按保存，先登记当前倒车终点，再写 Flash。
            // 否则旧逻辑会在按键释放时才登记终点，导致 Flash 中 target_flag 仍为 0。
            guandao_record_park_target_now(p);
            Flash_Store_Mode(route_setting_choice);
            Buzzer_check(200);
            key1_save_wait_release = 1;
        }
        return;
    }
    else
    {
        if(key1_save_start_ms != 0 && key1_save_wait_release == 0)
        {
            key4_flag = 1;                 // Short release records the parking point.
        }
        key1_save_start_ms = 0;
        if(key1_save_wait_release)
        {
            key1_save_wait_release = 0;
            return;
        }
    }
    // SBUS CH3: short press records a GPS assist point, long press saves the route.
    // SBUS CH4 is still handled by recode_waypoint() as the stop-point command.
    if(x6f_out[2] == 200)
    {
        now_ms = system_getval_ms();
        if(rc_ch3_start_ms == 0) rc_ch3_start_ms = now_ms;
        if(guandao_elapsed_ms(now_ms, rc_ch3_start_ms) > 1500 && !rc_ch3_wait_release)
        {
            guandao_record_park_target_now(p);
            Flash_Store_Mode(route_setting_choice);
            Buzzer_check(200);
            rc_ch3_wait_release = 1;
        }
        if( p == &passage)portion2_points_recode();
        else recode_waypoint(p);
        return;
    }
    else
    {
        if(rc_ch3_start_ms != 0 && rc_ch3_wait_release == 0)
        {
            if(GPS_WORK_FLAG)
            {
                if(recode_gps(p)) Buzzer_check(20);
            }
        }
        rc_ch3_start_ms = 0;
        if(rc_ch3_wait_release)
        {
            rc_ch3_wait_release = 0;
            return;
        }
    }
    if( p == &passage)portion2_points_recode();     // passage路径：按键手动记录（适合构建复杂赛道）
    else recode_waypoint(p);                                         // 其他路径：自动等距记录（移动超过阈值自动记录）

//    guandao_show(p);                                                // 在IPS200屏幕上显示路径信息（长度、位姿等）
    if(GPS_WORK_FLAG){if(key2_flag == 1){ key2_flag = 0 ; if(recode_gps(p)) Buzzer_check(20);  }}        // GPS辅助记录（可选）：当GPS工作标志为真且按键2被按下时
//     guandao_show();

}

void guandao_record_session_reset(void)
{
    guandao_record_init_pending = 1;
    park_record_stage = 0;
    park_start_record_ms = 0;
    daoche_point_length = 0;
    daoche_start_flag = 0;
    daoche_target_length = 0;
    daoche_target_flag = 0;
    daoche_flash_cheack = 0;
    daoche_flag = 0;
    key1_flag = 0;
    key4_flag = 0;
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
    static state_t reverse_map[MAX_LENGTH_INDEX];
    int16 len = portion_3.length_index;
    state_t origin;
    float return_heading = 0.0f;
    float heading_rad = 0.0f;
    float heading_sin = 0.0f;
    float heading_cos = 1.0f;

    if(len <= 1)
    {
        portion3_foint_flag = 0;
        return 0;
    }

    /*
     * Subject 3 uses portion_3 as a remote-control recording route.
     * The recorded direction is start area -> parking area, while the
     * autonomous run must return parking area -> start area.
     *
     * Reversing the point order alone is not enough: after saving or
     * rebooting, the car starts the return run with its local position at
     * (0,0), but the recorded parking-area endpoint still has the old
     * start-area coordinates.  Use that endpoint as the new origin, then
     * rotate the path so the first return segment points to local +Y.
     */
    origin = portion_3.recode_map[len - 1];
    return_heading = guandao_segment_yaw(origin, portion_3.recode_map[len - 2]);
    heading_rad = return_heading / 180.0f * M_PI;
    heading_sin = sinf(heading_rad);
    heading_cos = cosf(heading_rad);
    portion3_foint_flag = len;

    for(int16 i = 0; i < len; i++)
    {
        state_t src = portion_3.recode_map[len - 1 - i];
        float dx = src.x - origin.x;
        float dy = src.y - origin.y;

        reverse_map[i].x = dx * heading_cos - dy * heading_sin;
        reverse_map[i].y = dx * heading_sin + dy * heading_cos;
        reverse_map[i].theta = src.theta + 180.0f - return_heading;
        angle_plan(&reverse_map[i].theta);
    }

    for(int16 i = 0; i < len; i++)
    {
        portion_3.recode_map[i] = reverse_map[i];
    }

    portion_3.current_state.x = 0.0f;
    portion_3.current_state.y = 0.0f;
    portion_3.current_state.theta = Yaw_1;
    portion_3.current_point_index = 0;
    portion_3.planned_length = 0;
    portion_3.plan_ready = 0;

    return 1;
}

void portion3_return_reset(void)
{
    portion_3.current_state.x = 0.0f;
    portion_3.current_state.y = 0.0f;
    portion_3.current_state.theta = 0.0f;
    portion_3.current_point_index = 0;
    portion_3.planned_length = 0;
    portion_3.plan_ready = 0;
    out_v_l = 0.0f;
    out_v_r = 0.0f;
    out_servo = 0.0f;
    daoche_flag = 0;
    Yaw_1 = 0.0f;
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
