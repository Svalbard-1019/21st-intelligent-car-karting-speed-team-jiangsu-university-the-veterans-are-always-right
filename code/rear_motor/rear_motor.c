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
 * - PWM/GPIO 仍使用 peripheral.h 里的 PWM_L/PWM_R/MOTOR_GPIO_L/MOTOR_GPIO_R。
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

/* ---- 模块内部状态 ---- */
static float  target_mps      = 0.0f;
static float  actual_mps      = 0.0f;
static int16  current_pwm     = 0;
static int16  encoder_10ms    = 0;
static int32  encoder_100ms   = 0;
static int32  encoder_100ms_last = 0;
static volatile uint32 encoder_sample_count = 0;
static uint32 last_encoder_sample_count = 0;
static uint8  encoder_div = 0;
static int16  last_encoder_count = 0;
static uint8  encoder_first_read = 1;

/* PID 状态 */
static float  integral    = 0.0f;
static float  last_error  = 0.0f;
static int    last_pwm    = 0;

/* ---- 电机驱动 (DIR + PWM, 旧工程方式) ---- */
/**
 * 函数说明：rear_motor_set_pwm()。根据符号和限幅要求输出 PWM，占空比正负通常对应电机方向。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - pwm：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
static void rear_motor_set_pwm(int16 pwm)
{
    int diff = pwm - last_pwm;
    if(diff > REAR_PWM_RATE_LIMIT)  diff = REAR_PWM_RATE_LIMIT;
    if(diff < -REAR_PWM_RATE_LIMIT) diff = -REAR_PWM_RATE_LIMIT;
    last_pwm += diff;

    if(last_pwm > REAR_PWM_HARD_LIMIT)  last_pwm = REAR_PWM_HARD_LIMIT;
    if(last_pwm < -REAR_PWM_HARD_LIMIT) last_pwm = -REAR_PWM_HARD_LIMIT;

    current_pwm = last_pwm;

    if(current_pwm >= 0)
    {
        pwm_set_duty(PWM_L,  current_pwm);
        gpio_set_level(MOTOR_GPIO_L, 1);
        pwm_set_duty(PWM_R,  current_pwm);
        gpio_set_level(MOTOR_GPIO_R, 1);
    }
    else
    {
        pwm_set_duty(PWM_L,  -current_pwm);
        gpio_set_level(MOTOR_GPIO_L, 0);
        pwm_set_duty(PWM_R,  -current_pwm);
        gpio_set_level(MOTOR_GPIO_R, 0);
    }
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
    pwm_init(PWM_L, 17000, 0);
    pwm_init(PWM_R, 17000, 0);
    gpio_init(MOTOR_GPIO_L, GPO, 1, GPO_PUSH_PULL);
    gpio_init(MOTOR_GPIO_R, GPO, 1, GPO_PUSH_PULL);

    target_mps  = 0.0f;
    actual_mps  = 0.0f;
    current_pwm = 0;
    encoder_10ms  = 0;
    encoder_100ms = 0;
    encoder_100ms_last = 0;
    encoder_sample_count = 0;
    last_encoder_sample_count = 0;
    encoder_div = 0;
    last_encoder_count = encoder_get_count(TIM2_ENCODER);
    encoder_first_read = 0;
    integral    = 0.0f;
    last_error  = 0.0f;
    last_pwm    = 0;
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
    target_mps  = 0.0f;
    integral    = 0.0f;
    last_error  = 0.0f;
    last_pwm    = 0;
    encoder_100ms = 0;
    encoder_100ms_last = 0;
    encoder_div = 0;
    encoder_10ms = 0;
    last_encoder_count = encoder_get_count(TIM2_ENCODER);
    encoder_first_read = 0;

    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
    gpio_set_level(MOTOR_GPIO_L, 1);
    gpio_set_level(MOTOR_GPIO_R, 1);
    current_pwm = 0;
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
void rear_motor_encoder_update_10ms(void)
{
    int16 current_count = encoder_get_count(TIM2_ENCODER);

    if(encoder_first_read)
    {
        last_encoder_count = current_count;
        encoder_10ms = 0;
        encoder_first_read = 0;
    }
    else
    {
        encoder_10ms = (int16)calculate_delta(current_count, last_encoder_count);
        if(encoder_10ms > REAR_ENCODER_DELTA_ABS_MAX || encoder_10ms < -REAR_ENCODER_DELTA_ABS_MAX)
        {
            encoder_10ms = 0;
        }
        last_encoder_count = current_count;
    }

    encoder_sample_count++;
}

/* 主循环调用: 有新10ms编码器样本才处理, 每100ms更新一次PID */
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
    if(last_encoder_sample_count == encoder_sample_count)
    {
        return;
    }

    last_encoder_sample_count = encoder_sample_count;
    actual_mps = (float)((int32)encoder_10ms * 10) / REAR_EFFECTIVE_PPR * REAR_WHEEL_CIRCUM_M / 0.1f;
    encoder_100ms += (int32)encoder_10ms;
    encoder_div++;

    if(encoder_div < 10)
    {
        return;
    }

    encoder_div = 0;
    encoder_100ms_last = encoder_100ms;

    if(target_mps == 0.0f)
    {
        encoder_100ms = 0;
        rear_motor_stop();
        return;
    }

    float target_pulses = target_mps * REAR_EFFECTIVE_PPR / REAR_WHEEL_CIRCUM_M * 0.1f;
    float error = target_pulses - (float)encoder_100ms;

    if(error < REAR_INTEGRAL_THRESHOLD && error > -REAR_INTEGRAL_THRESHOLD)
    {
        integral += error * 0.1f;
        if(integral > REAR_INTEGRAL_LIMIT)   integral = REAR_INTEGRAL_LIMIT;
        if(integral < -REAR_INTEGRAL_LIMIT)  integral = -REAR_INTEGRAL_LIMIT;
    }

    float derivative = (error - last_error) / 0.1f;
    last_error = error;

    float ff     = target_pulses * REAR_FF_GAIN;
    float pid    = REAR_KP * error + REAR_KI * integral + REAR_KD * derivative;
    float pwm_f  = ff + pid;

    encoder_100ms = 0;
    rear_motor_set_pwm((int16)pwm_f);
}

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
