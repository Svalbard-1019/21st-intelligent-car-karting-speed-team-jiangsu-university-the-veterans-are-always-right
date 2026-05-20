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
        last_encoder_count = current_count;
    }

    encoder_sample_count++;
}

/* 主循环调用: 有新10ms编码器样本才处理, 每100ms更新一次PID */
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
float  rear_motor_get_target_mps(void)      { return target_mps; }
float  rear_motor_get_speed_mps(void)       { return actual_mps; }
int16  rear_motor_get_pwm(void)             { return current_pwm; }
int16  rear_motor_get_encoder_10ms(void)    { return encoder_10ms; }
int32  rear_motor_get_encoder_100ms(void)   { return encoder_100ms_last; }
