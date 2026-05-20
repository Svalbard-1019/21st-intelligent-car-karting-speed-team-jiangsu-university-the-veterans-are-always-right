/*
 * UTF-8 详细注释说明：前轮转向电机闭环控制实现。
 *
 * 模块职责：
 * 1. 使用 TIM4 编码器读取转向电机角度反馈。
 * 2. 使用两个 PWM 通道控制转向电机正反转。
 * 3. 把目标角度与当前角度做误差计算，再通过 PID 输出电机 PWM。
 *
 * 当前硬件方案：
 * - 转向电机 PWM 正转：ANGLE_PWM_IN1，对应 P02_7。
 * - 转向电机 PWM 反转：ANGLE_PWM_IN2，对应 P02_6。
 * - 转向编码器 A/B：TIM4 P02_8 / P00_9。
 *
 * 调试重点：
 * - 如果转向方向反了，优先检查 ANGLE_PWM_IN1/IN2 或编码器 A/B 相序。
 * - 如果角度只往一侧正常，通常是编码器方向、PID 输出符号或电机驱动线序问题。
 * - PID 参数在本模块中独立维护，不要和后轮 rear_motor PID 混淆。
 */

/*
 * angle_control.c
 *
 * Description: 转向电机角度控制模块
 */

#include "zf_common_headfile.h"
#include "angle_control.h"

AngleControl_TypeDef angle_ctrl;

#define ANGLE_DEGREE_PER_PULSE  (360.0f / (ANGLE_PPR * ANGLE_GEAR_RATIO))

// 溢出处理：把int16累积成int32
int32 accumulated_encoder_count = 0;
static int16 prev_raw_count = 0;
static uint8 encoder_first_read = 1;

static int32 encoder_get_accumulated_count(void)
{
    int16 raw = encoder_get_count(ANGLE_ENCODER);

    if (encoder_first_read) {
        prev_raw_count = raw;
        accumulated_encoder_count = (int32)raw;
        encoder_first_read = 0;
    } else {
        int32 diff = (int32)raw - (int32)prev_raw_count;
        // encoder_get_count quad模式下已除4，范围±8192，溢出阈值相应缩小
        if (diff > 8192) diff -= 16384;
        else if (diff < -8192) diff += 16384;
        accumulated_encoder_count += diff;
        prev_raw_count = raw;
    }
    return accumulated_encoder_count;
}

static float angle_control_get_angle_from_count(int32 count)
{
    return (float)(count - angle_ctrl.encoder_zero_count) * ANGLE_DEGREE_PER_PULSE;
}

void angle_control_init(void) {
    pwm_init(ANGLE_PWM_IN1, 10000, 0);
    pwm_init(ANGLE_PWM_IN2, 10000, 0);

    encoder_quad_init(ANGLE_ENCODER, ANGLE_ENCODER_A_PIN, ANGLE_ENCODER_B_PIN);

    PID_Init(&angle_ctrl.pid, ANGLE_DEFAULT_KP, ANGLE_DEFAULT_KI, ANGLE_DEFAULT_KD, ANGLE_OUTPUT_MAX);
    angle_ctrl.target_angle = 0;
    angle_ctrl.current_angle = 0;
    angle_ctrl.encoder_zero_count = 0;
    angle_ctrl.control_count = 0;

    // 初始化溢出处理状态
    encoder_first_read = 1;
    accumulated_encoder_count = 0;
    prev_raw_count = 0;

    // 零位设为0，与accumulated_encoder_count保持一致
    angle_ctrl.encoder_zero_count = 0;
}

void angle_motor_set_pwm(int32 pwm_value) {
    if (pwm_value > ANGLE_OUTPUT_MAX) {
        pwm_value = ANGLE_OUTPUT_MAX;
    } else if (pwm_value < -ANGLE_OUTPUT_MAX) {
        pwm_value = -ANGLE_OUTPUT_MAX;
    }

    if (pwm_value > 0) {
        pwm_set_duty(ANGLE_PWM_IN1, pwm_value);
        pwm_set_duty(ANGLE_PWM_IN2, 0);
    } else if (pwm_value < 0) {
        pwm_set_duty(ANGLE_PWM_IN1, 0);
        pwm_set_duty(ANGLE_PWM_IN2, -pwm_value);
    } else {
        pwm_set_duty(ANGLE_PWM_IN1, 0);
        pwm_set_duty(ANGLE_PWM_IN2, 0);
    }
}

void angle_control_update(void) {
    int32 current_count = encoder_get_accumulated_count();
    float pid_output = 0;

    angle_ctrl.current_angle = angle_control_get_angle_from_count(current_count);

    pid_output = PID_Compute(&angle_ctrl.pid, angle_ctrl.target_angle, angle_ctrl.current_angle);

    if ((angle_ctrl.target_angle - angle_ctrl.current_angle < ANGLE_DEAD_BAND) &&
        (angle_ctrl.target_angle - angle_ctrl.current_angle > -ANGLE_DEAD_BAND)) {
        pid_output = 0;
        angle_ctrl.pid.Integral = 0;
    }

    angle_motor_set_pwm((int32)pid_output);
    angle_ctrl.control_count++;
}

void angle_control_set_target(int32 target_angle) {
    if (target_angle > ANGLE_MAX_DEGREE) {
        angle_ctrl.target_angle = ANGLE_MAX_DEGREE;
    } else if (target_angle < ANGLE_MIN_DEGREE) {
        angle_ctrl.target_angle = ANGLE_MIN_DEGREE;
    } else {
        angle_ctrl.target_angle = (float)target_angle;
    }
}

void angle_control_rotate_relative(int32 delta_angle) {
    angle_control_set_target((int32)(angle_ctrl.current_angle + (float)delta_angle));
}

int32 angle_control_get_current_angle(void) {
    return (int32)angle_ctrl.current_angle;
}

void angle_control_reset(void) {
    PID_Reset(&angle_ctrl.pid);
    angle_ctrl.target_angle = 0;
    angle_ctrl.current_angle = 0;
    // 重置溢出处理状态
    encoder_first_read = 1;
    accumulated_encoder_count = 0;
    prev_raw_count = 0;
    // 零位直接设为0，与accumulated_encoder_count保持一致
    angle_ctrl.encoder_zero_count = 0;
    angle_ctrl.control_count = 0;
    pwm_set_duty(ANGLE_PWM_IN1, 0);
    pwm_set_duty(ANGLE_PWM_IN2, 0);
}
