/*
 * UTF-8 详细注释说明：后轮 m/s 速度闭环模块。
 *
 * 模块职责：
 * 1. rear_motor_encoder_update_10ms() 读取 TIM2 编码器，计算 10ms 速度反馈。
 * 2. rear_motor_pid_update_100ms() 按目标 m/s 做前馈 + PID 修正，输出左右后轮 PWM。
 * 3. rear_motor_set_target_mps() 给科目一、直线测试等上层模式设置目标速度。
 *
 * 当前硬件：
 * - 只接左后轮编码器，左右后轮共用同一速度反馈。
 * - HIP 四路 PWM 使用 peripheral.h 里的 PWM_L1/PWM_L2/PWM_R1/PWM_R2。
 *
 * 调试重点：
 * - TgtAct 中目标有值但 PWM=0，查本模块限幅/stop 条件。
 * - PWM 有值但 Act 不变，查编码器接线或电机驱动。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 启动后直接调用 rear_motor_init() 初始化 PWM、方向 GPIO 和后轮速度闭环状态。
 * 2. CCU61_CH0 中断在 GUANDAO/DAOCHE/RACK_TEST 模式下每 10ms 调用 rear_motor_encoder_update_10ms() 采样 TIM2 左后轮编码器。
 * 3. 科目一自动驾驶时，portion_1()/guandao_trace() 先算 out_v_l/out_v_r，主循环末尾 Guandao_Rear_Motor_Update() 把旧惯导速度乘 GUANDAO_SPEED_TO_MPS 换算为 m/s。
 * 4. Guandao_Rear_Motor_Update() 调用 rear_motor_set_target_mps() 和 rear_motor_pid_update_100ms()，最后由 rear_motor_set_pwm() 同步驱动左右后轮。
 */


/*
 * rear_motor.c
 *
 * 后轮独立驱动模块实现
 * 架构: 目标 m/s -> 100ms脉冲目标 -> 前馈 + PID修正 -> PWM -> DIR+PWM驱动
 * 编码器: 固定周期 10ms 读取累计差值, 累加至 100ms 供 PID 使用
 */

#include "zf_common_headfile.h"
#include "rear_motor/rear_motor.h"
#include "rear_motor/rear_odometry_pose_buffer.h"
#include "rear_motor/rear_pwm_slew.h"
#include "rear_motor/rear_reverse_pwm_floor.h"

/* ---- 模块内部状态 ---- */
static float  target_mps      = 0.0f;
static float  actual_mps      = 0.0f;
static float  raw_actual_mps  = 0.0f;
static float  filtered_pulses_100ms = 0.0f;
static uint8  speed_filter_initialized = 0;
static int16  current_pwm     = 0;
static int16  requested_pwm   = 0;
static int16  encoder_10ms    = 0;
static int32  encoder_100ms_last = 0;
static rear_odometry_pose_buffer_t odometry_pose_buffer;
static volatile int32 odometry_total_pulses = 0;
/* The ISR owns the build window. Completed 100ms windows wait here until
 * the main loop atomically takes them, so a delayed loop cannot lose samples. */
static volatile int32 speed_window_build_pulses = 0;
static volatile uint8 speed_window_build_samples = 0;
static volatile int32 speed_window_ready_pulses = 0;
static volatile uint16 speed_window_ready_count = 0;
static int16  last_encoder_count = 0;
static uint8  encoder_first_read = 1;

/* PID 状态 */
static float  integral    = 0.0f;
static float  last_error  = 0.0f;
static int    last_pwm    = 0;
static int16  applied_pwm_l = 0;
static int16  applied_pwm_r = 0;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float ff_gain;
    float high_speed_ff_gain;
    int pwm_rate_limit;
    int pwm_release_limit;
} rear_motor_control_profile_t;

static const rear_motor_control_profile_t rear_kmy_profile = {
    REAR_KMY_KP, REAR_KMY_KI, REAR_KMY_KD, REAR_KMY_FF_GAIN,
    REAR_KMY_HIGH_SPEED_FF_GAIN, REAR_KMY_PWM_RATE_LIMIT,
    REAR_KMY_PWM_RELEASE_LIMIT
};
static const rear_motor_control_profile_t rear_kms_profile = {
    REAR_KMS_KP, REAR_KMS_KI, REAR_KMS_KD, REAR_KMS_FF_GAIN,
    REAR_KMS_HIGH_SPEED_FF_GAIN, REAR_KMS_PWM_RATE_LIMIT,
    REAR_KMS_PWM_RELEASE_LIMIT
};
static uint8 rear_route_profile = 0u;

static const rear_motor_control_profile_t *rear_motor_active_profile(void)
{
    return (rear_route_profile == 2u) ? &rear_kms_profile : &rear_kmy_profile;
}

void rear_motor_select_route(uint8 route_choice)
{
    uint8 next_profile = (route_choice == 2u) ? 2u : 0u;
    if(next_profile != rear_route_profile)
    {
        rear_route_profile = next_profile;
        integral = 0.0f;
        last_error = 0.0f;
    }
}

uint8 rear_motor_get_route_profile(void)
{
    return rear_route_profile;
}

/* Explicit-stop active brake state. Emergency rear_motor_stop() cancels it. */
static uint8 brake_active = 0;
static uint8 brake_exit_reason = REAR_BRAKE_REASON_NONE;
static uint32 brake_start_ms = 0;
static uint32 brake_elapsed_ms = 0;
static int16 brake_output_pwm = 0;
static float brake_start_speed_mps = 0.0f;
static float brake_end_raw_mps = 0.0f;
static int8 brake_motion_sign = 0;

/* ---- HIP4082 电机驱动（每个电机两路 PWM） ---- */
/**
 * 函数说明：rear_motor_set_pwm()。根据符号和限幅要求输出 PWM，占空比正负通常对应电机方向。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - pwm：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
static void rear_motor_set_pwm_internal(int16 pwm, uint8 gradual_release)
{
    extern float out_v_l;
    extern float out_v_r;
    extern MOTER_control_mode conrtol_mode;
    const rear_motor_control_profile_t *profile = rear_motor_active_profile();
    int diff;
    int16 pwm_l;
    int16 pwm_r;

    if(gradual_release)
    {
        last_pwm = rear_pwm_slew_step(
                last_pwm,
                pwm,
                profile->pwm_rate_limit,
                profile->pwm_release_limit);
    }
    else
    {
        diff = pwm - last_pwm;
        if(diff > profile->pwm_rate_limit) diff = profile->pwm_rate_limit;
        if(diff < -profile->pwm_rate_limit) diff = -profile->pwm_rate_limit;
        last_pwm += diff;
    }

    if(last_pwm > REAR_PWM_HARD_LIMIT)  last_pwm = REAR_PWM_HARD_LIMIT;
    if(last_pwm < -REAR_PWM_HARD_LIMIT) last_pwm = -REAR_PWM_HARD_LIMIT;

    current_pwm = last_pwm;
    pwm_l = current_pwm;
    pwm_r = current_pwm;

    /* Match KMY: add open-loop differential feedforward around center PWM. */
    if(conrtol_mode == GUANDAO)
    {
        float diff_val = out_v_l - out_v_r;
        int16 diff_pwm = (int16)(diff_val * REAR_DIFF_PWM_GAIN);
        if(diff_pwm > 1500) diff_pwm = 1500;
        if(diff_pwm < -1500) diff_pwm = -1500;
        pwm_l = current_pwm + diff_pwm;
        pwm_r = current_pwm - diff_pwm;
    }

    if(pwm_l > REAR_PWM_HARD_LIMIT) pwm_l = REAR_PWM_HARD_LIMIT;
    if(pwm_l < -REAR_PWM_HARD_LIMIT) pwm_l = -REAR_PWM_HARD_LIMIT;
    if(pwm_r > REAR_PWM_HARD_LIMIT) pwm_r = REAR_PWM_HARD_LIMIT;
    if(pwm_r < -REAR_PWM_HARD_LIMIT) pwm_r = -REAR_PWM_HARD_LIMIT;

    /* Break-before-make is required only when a wheel actually reverses.
     * Re-clearing an already active channel on every loop modulates the
     * effective duty cycle and makes fixed-PWM speed depend on loop timing. */
    if((pwm_l > 0 && applied_pwm_l < 0)
            || (pwm_l < 0 && applied_pwm_l > 0))
    {
        pwm_set_duty(PWM_L1, 0);
        pwm_set_duty(PWM_L2, 0);
    }
    if((pwm_r > 0 && applied_pwm_r < 0)
            || (pwm_r < 0 && applied_pwm_r > 0))
    {
        pwm_set_duty(PWM_R1, 0);
        pwm_set_duty(PWM_R2, 0);
    }

    if(pwm_l > 0)
    {
        pwm_set_duty(PWM_L2, 0);
        pwm_set_duty(PWM_L1, pwm_l);
    }
    else if(pwm_l < 0)
    {
        pwm_set_duty(PWM_L1, 0);
        pwm_set_duty(PWM_L2, -pwm_l);
    }
    else
    {
        pwm_set_duty(PWM_L1, 0);
        pwm_set_duty(PWM_L2, 0);
    }

    if(pwm_r > 0)
    {
        pwm_set_duty(PWM_R2, 0);
        pwm_set_duty(PWM_R1, pwm_r);
    }
    else if(pwm_r < 0)
    {
        pwm_set_duty(PWM_R1, 0);
        pwm_set_duty(PWM_R2, -pwm_r);
    }
    else
    {
        pwm_set_duty(PWM_R1, 0);
        pwm_set_duty(PWM_R2, 0);
    }

    applied_pwm_l = pwm_l;
    applied_pwm_r = pwm_r;
}

static void rear_motor_set_pwm(int16 pwm)
{
    rear_motor_set_pwm_internal(pwm, 0u);
}

static void rear_motor_set_drive_pwm(int16 pwm)
{
    rear_motor_set_pwm_internal(pwm, 1u);
}

/* ---- 公开接口 ---- */
/**
 * 函数说明：rear_motor_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_init(void)
{
    pwm_init(PWM_L1, 17000, 0);
    pwm_init(PWM_L2, 17000, 0);
    pwm_init(PWM_R1, 17000, 0);
    pwm_init(PWM_R2, 17000, 0);

    rear_route_profile = 0u;
    target_mps  = 0.0f;
    actual_mps  = 0.0f;
    raw_actual_mps = 0.0f;
    filtered_pulses_100ms = 0.0f;
    speed_filter_initialized = 0;
    current_pwm = 0;
    requested_pwm = 0;
    encoder_10ms  = 0;
    encoder_100ms_last = 0;
    rear_odometry_pose_buffer_init(&odometry_pose_buffer);
    odometry_total_pulses = 0;
    speed_window_build_pulses = 0;
    speed_window_build_samples = 0;
    speed_window_ready_pulses = 0;
    speed_window_ready_count = 0;
    last_encoder_count = encoder_get_count(TIM2_ENCODER);
    encoder_first_read = 0;
    integral    = 0.0f;
    last_error  = 0.0f;
    last_pwm    = 0;
    applied_pwm_l = 0;
    applied_pwm_r = 0;
    brake_active = 0;
    brake_exit_reason = REAR_BRAKE_REASON_NONE;
    brake_start_ms = 0;
    brake_elapsed_ms = 0;
    brake_output_pwm = 0;
    brake_start_speed_mps = 0.0f;
    brake_end_raw_mps = 0.0f;
    brake_motion_sign = 0;
}

/**
 * 函数说明：rear_motor_stop()。立即撤销目标输出并关闭执行器，常用于安全停车或目标速度为 0 的场景。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_stop(void)
{
    uint32 interrupt_state;

    brake_active = 0;
    brake_output_pwm = 0;
    brake_motion_sign = 0;
    target_mps  = 0.0f;
    requested_pwm = 0;
    integral    = 0.0f;
    last_error  = 0.0f;
    last_pwm    = 0;
    encoder_100ms_last = 0;
    actual_mps = 0.0f;
    raw_actual_mps = 0.0f;
    filtered_pulses_100ms = 0.0f;
    speed_filter_initialized = 0;

    interrupt_state = interrupt_global_disable();
    encoder_10ms = 0;
    speed_window_build_pulses = 0;
    speed_window_build_samples = 0;
    speed_window_ready_pulses = 0;
    speed_window_ready_count = 0;
    interrupt_global_enable(interrupt_state);
    /* Keep the fixed-period encoder baseline intact. Record mode can call
     * rear_motor_stop() every main-loop pass while the car is pushed. */

    pwm_set_duty(PWM_L1, 0);
    pwm_set_duty(PWM_L2, 0);
    pwm_set_duty(PWM_R1, 0);
    pwm_set_duty(PWM_R2, 0);
    current_pwm = 0;
    applied_pwm_l = 0;
    applied_pwm_r = 0;
}

/**
 * 函数说明：rear_motor_set_target_mps()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - mps：速度参数，mps 表示 m/s，旧惯导速度会在上层换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_set_target_mps(float mps)
{
    if(mps > REAR_SPEED_MAX_MPS)  mps = REAR_SPEED_MAX_MPS;
    if(mps < REAR_SPEED_MIN_MPS)  mps = REAR_SPEED_MIN_MPS;
    target_mps = mps;

    if(mps == 0.0f)
    {
        requested_pwm = 0;
        integral   = 0.0f;
        last_error = 0.0f;
        last_pwm   = 0;
    }
}

/* 每 10ms 调用: 读编码器累计差值, 不清零, 避免和惯导共用TIM2时互相抢数据 */
/**
 * 函数说明：rear_motor_encoder_update_10ms()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_encoder_update_10ms(float yaw_deg)
{
    int16 current_count = encoder_get_count(TIM2_ENCODER);
    int32 raw_encoder_delta = 0;

    if(encoder_first_read)
    {
        last_encoder_count = current_count;
        encoder_10ms = 0;
        encoder_first_read = 0;
    }
    else
    {
        raw_encoder_delta = (int32)REAR_ENCODER_FEEDBACK_DIRECTION
                * (int32)calculate_delta(current_count, last_encoder_count);
        if(raw_encoder_delta > REAR_ENCODER_DELTA_ABS_MAX
                || raw_encoder_delta < -REAR_ENCODER_DELTA_ABS_MAX)
        {
            encoder_10ms = 0;
        }
        else
        {
            encoder_10ms = (int16)raw_encoder_delta;
            odometry_total_pulses += raw_encoder_delta;
            rear_odometry_pose_buffer_add(&odometry_pose_buffer,
                    raw_encoder_delta, yaw_deg);
        }
        last_encoder_count = current_count;
    }

    speed_window_build_pulses += (int32)encoder_10ms;
    speed_window_build_samples++;
    if(speed_window_build_samples >= 10)
    {
        speed_window_ready_pulses += speed_window_build_pulses;
        speed_window_ready_count++;
        speed_window_build_pulses = 0;
        speed_window_build_samples = 0;
    }
}

/* 主循环调用: 有新10ms编码器样本才处理, 每100ms更新一次PID */
/* Atomically take all completed 100ms windows. If the main loop was delayed,
 * pulses contains every completed window and window_count records how many. */
static uint8 rear_motor_take_speed_windows(int32 *pulses, uint16 *window_count)
{
    uint32 interrupt_state = interrupt_global_disable();
    uint8 available = (speed_window_ready_count > 0u);

    if(available)
    {
        *pulses = speed_window_ready_pulses;
        *window_count = speed_window_ready_count;
        speed_window_ready_pulses = 0;
        speed_window_ready_count = 0;
    }
    interrupt_global_enable(interrupt_state);
    return available;
}

static float rear_motor_filter_speed(float measured_pulses)
{
    raw_actual_mps = measured_pulses * REAR_ENCODER_METERS_PER_PULSE / 0.1f;

    if(!speed_filter_initialized)
    {
        filtered_pulses_100ms = measured_pulses;
        speed_filter_initialized = 1;
    }
    else
    {
        filtered_pulses_100ms += REAR_SPEED_FILTER_ALPHA
                * (measured_pulses - filtered_pulses_100ms);
    }

    actual_mps = filtered_pulses_100ms * REAR_ENCODER_METERS_PER_PULSE / 0.1f;
    return filtered_pulses_100ms;
}

/**
 * 函数说明：rear_motor_pid_update_100ms()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_pid_update_100ms(void)
{
    int32 window_pulses;
    uint16 window_count;
    float measured_pulses;
    float filtered_pulses;
    float target_pulses;
    float error;
    float derivative;
    float ff;
    float high_speed_ff = 0.0f;
    float pid;
    float pwm_f;
    const rear_motor_control_profile_t *profile = rear_motor_active_profile();

    if(!rear_motor_take_speed_windows(&window_pulses, &window_count))
    {
        return;
    }

    /* Multiple pending windows are averaged to one true 100ms measurement.
     * No encoder pulses are discarded when the main loop is briefly delayed. */
    measured_pulses = (float)window_pulses / (float)window_count;
    encoder_100ms_last = (int32)measured_pulses;
    filtered_pulses = rear_motor_filter_speed(measured_pulses);

    if(target_mps == 0.0f)
    {
        rear_motor_stop();
        return;
    }

    target_pulses = target_mps / REAR_ENCODER_METERS_PER_PULSE * 0.1f;
    error = target_pulses - filtered_pulses;

    if(error < REAR_INTEGRAL_THRESHOLD && error > -REAR_INTEGRAL_THRESHOLD)
    {
        integral += error * 0.1f;
        if(integral > REAR_INTEGRAL_LIMIT)   integral = REAR_INTEGRAL_LIMIT;
        if(integral < -REAR_INTEGRAL_LIMIT)  integral = -REAR_INTEGRAL_LIMIT;
    }

    derivative = (error - last_error) / 0.1f;
    last_error = error;
    ff = target_pulses * profile->ff_gain;
    if(fabsf(target_mps) > REAR_HIGH_SPEED_FF_START_MPS)
    {
        high_speed_ff = (fabsf(target_mps) - REAR_HIGH_SPEED_FF_START_MPS)
                * profile->high_speed_ff_gain;
        if(target_mps < 0.0f) high_speed_ff = -high_speed_ff;
        ff += high_speed_ff;
    }
    pid = profile->kp * error + profile->ki * integral + profile->kd * derivative;
    pwm_f = ff + pid;
    pwm_f = rear_reverse_apply_startup_pwm_floor(
            target_mps, raw_actual_mps, pwm_f,
            REAR_REVERSE_STARTUP_SPEED_MPS,
            REAR_REVERSE_STARTUP_PWM_MIN);
    requested_pwm = (int16)pwm_f;
    requested_pwm = (int16)rear_pwm_limit_request_sign(target_mps, requested_pwm);
    rear_motor_set_drive_pwm(requested_pwm);
}

/**
 * 函数说明：rear_motor_open_loop_update()。RackTest Stage 4 使用固定 PWM 驱动后轮，
 * 绕过速度 PID，同时继续更新编码器换算得到的实际速度。
 * 参数说明：pwm 为目标 PWM，正负号表示方向，内部仍执行变化率和硬限幅保护。
 * 安全说明：该接口只由 RackTest 调用；切换测试阶段时必须调用 rear_motor_stop()。
 */
void rear_motor_open_loop_update(int16 pwm)
{
    int32 window_pulses;
    uint16 window_count;

    if(rear_motor_take_speed_windows(&window_pulses, &window_count))
    {
        float measured_pulses = (float)window_pulses / (float)window_count;
        encoder_100ms_last = (int32)measured_pulses;
        rear_motor_filter_speed(measured_pulses);
    }

    target_mps = 0.0f;
    requested_pwm = pwm;
    integral = 0.0f;
    last_error = 0.0f;
    rear_motor_set_pwm(pwm);
}

static uint32 rear_motor_brake_time_since(uint32 now_ms, uint32 start_ms)
{
    if(now_ms >= start_ms) return now_ms - start_ms;
    return (REAR_BRAKE_SYSTEM_MS_WRAP - start_ms) + now_ms;
}

static void rear_motor_brake_finish(uint8 reason, float raw_speed_mps)
{
    brake_exit_reason = reason;
    brake_end_raw_mps = raw_speed_mps;
    rear_motor_stop();
}

void rear_motor_brake_start(void)
{
    float direction_speed_mps;

    if(brake_active) return;

    direction_speed_mps = (fabsf(raw_actual_mps) > REAR_BRAKE_STOP_SPEED_MPS)
            ? raw_actual_mps : actual_mps;
    brake_start_speed_mps = fabsf(actual_mps);
    brake_start_ms = system_getval_ms();
    brake_elapsed_ms = 0;
    brake_output_pwm = 0;
    brake_exit_reason = REAR_BRAKE_REASON_NONE;
    brake_end_raw_mps = 0.0f;
    brake_motion_sign = 0;
    target_mps = 0.0f;
    integral = 0.0f;
    last_error = 0.0f;

    if(brake_start_speed_mps <= REAR_BRAKE_STOP_SPEED_MPS
            && fabsf(raw_actual_mps) <= REAR_BRAKE_STOP_SPEED_MPS)
    {
        rear_motor_brake_finish(REAR_BRAKE_REASON_LOW_SPEED, raw_actual_mps);
        return;
    }

    brake_motion_sign = (direction_speed_mps > 0.0f) ? 1 : -1;
    brake_active = 1;
}

void rear_motor_brake_update(void)
{
    float raw_speed_mps;
    float abs_speed_mps;
    float signed_speed_mps;
    uint8 high_speed_guard;

    if(!brake_active) return;

    raw_speed_mps = raw_actual_mps;
    abs_speed_mps = fabsf(raw_speed_mps);
    signed_speed_mps = raw_speed_mps * (float)brake_motion_sign;
    brake_elapsed_ms = rear_motor_brake_time_since(system_getval_ms(), brake_start_ms);
    high_speed_guard = (brake_start_speed_mps >= REAR_BRAKE_HIGH_SPEED_MPS);

    if(brake_elapsed_ms >= REAR_BRAKE_TIMEOUT_MS)
    {
        rear_motor_brake_finish(REAR_BRAKE_REASON_TIMEOUT, raw_speed_mps);
        return;
    }
    if(signed_speed_mps < -REAR_BRAKE_REVERSE_MPS
            && (!high_speed_guard
                    || brake_elapsed_ms >= REAR_BRAKE_HIGH_REVERSE_GUARD_MS))
    {
        rear_motor_brake_finish(REAR_BRAKE_REASON_REVERSE, raw_speed_mps);
        return;
    }
    if(brake_elapsed_ms >= 100u && abs_speed_mps <= REAR_BRAKE_STOP_SPEED_MPS)
    {
        rear_motor_brake_finish(REAR_BRAKE_REASON_LOW_SPEED, raw_speed_mps);
        return;
    }

    if(abs_speed_mps > 2.5f) brake_output_pwm = REAR_BRAKE_PWM_HIGH;
    else if(abs_speed_mps > 1.0f) brake_output_pwm = REAR_BRAKE_PWM_MID;
    else brake_output_pwm = REAR_BRAKE_PWM_LOW;

    rear_motor_open_loop_update((int16)(-brake_motion_sign * brake_output_pwm));
}

uint8 rear_motor_brake_active(void) { return brake_active; }
uint8 rear_motor_brake_reason(void) { return brake_exit_reason; }
uint32 rear_motor_brake_elapsed_ms(void) { return brake_elapsed_ms; }
int16 rear_motor_brake_pwm(void) { return brake_output_pwm; }
float rear_motor_brake_end_raw_mps(void) { return brake_end_raw_mps; }

/* ---- getter ---- */
/**
 * 函数说明：rear_motor_get_target_mps()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float  rear_motor_get_target_mps(void)      { return target_mps; }
float rear_motor_get_error_pulses(void)        { return last_error; }
float rear_motor_get_integral_pulses(void)     { return integral; }
/**
 * 函数说明：rear_motor_get_speed_mps()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float  rear_motor_get_speed_mps(void)       { return actual_mps; }
float  rear_motor_get_raw_speed_mps(void)   { return raw_actual_mps; }
/**
 * 函数说明：rear_motor_get_pwm()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int16 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int16  rear_motor_get_pwm(void)             { return current_pwm; }
int16  rear_motor_get_requested_pwm(void)   { return requested_pwm; }
/**
 * 函数说明：rear_motor_get_encoder_10ms()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int16 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int16  rear_motor_get_encoder_10ms(void)    { return encoder_10ms; }
/**
 * 函数说明：rear_motor_get_encoder_100ms()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int32  rear_motor_get_encoder_100ms(void)   { return encoder_100ms_last; }

uint8 rear_motor_take_odometry_sample(int32 *pulses, float *yaw_deg)
{
    uint32 interrupt_state = interrupt_global_disable();
    rear_odometry_pose_sample_t sample;
    uint8 available = rear_odometry_pose_buffer_take(&odometry_pose_buffer, &sample);
    interrupt_global_enable(interrupt_state);

    if(available)
    {
        *pulses = (int32)sample.pulses;
        *yaw_deg = sample.yaw_deg;
    }
    return available;
}

uint32 rear_motor_get_odometry_merged_samples(void)
{
    return odometry_pose_buffer.merged_samples;
}

int32 rear_motor_get_odometry_total_pulses(void)
{
    return odometry_total_pulses;
}

uint8 rear_motor_get_odometry_pending_samples(void)
{
    return odometry_pose_buffer.count;
}
