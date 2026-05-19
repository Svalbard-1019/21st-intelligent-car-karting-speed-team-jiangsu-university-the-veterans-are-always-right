/*
 * angle_control.c
 *
 * Front steering motor closed-loop angle control.
 */

#include "zf_common_headfile.h"
#include "angle_control.h"

AngleControl_TypeDef angle_ctrl;

#define ANGLE_DEGREE_PER_PULSE  (-(360.0f / ((float)ANGLE_PPR * (float)ANGLE_GEAR_RATIO)))

static int32 accumulated_encoder_count = 0;
static int16 prev_raw_count = 0;
static uint8 encoder_first_read = 1;

static int32 angle_encoder_get_accumulated_count(void)
{
    int16 raw = encoder_get_count(ANGLE_ENCODER);

    if (encoder_first_read) {
        prev_raw_count = raw;
        accumulated_encoder_count = (int32)raw;
        encoder_first_read = 0;
    } else {
        int32 diff = (int32)raw - (int32)prev_raw_count;

        if (diff > 32767) {
            diff -= 65536;
        }
        if (diff < -32768) {
            diff += 65536;
        }

        accumulated_encoder_count += diff;
        prev_raw_count = raw;
    }

    return accumulated_encoder_count;
}

static float angle_control_get_angle_from_count(int32 count)
{
    return (float)(count - angle_ctrl.encoder_zero_count) * ANGLE_DEGREE_PER_PULSE;
}

void angle_control_init(void)
{
    pwm_init(ANGLE_PWM_IN1, ANGLE_PWM_FREQ, 0);
    pwm_init(ANGLE_PWM_IN2, ANGLE_PWM_FREQ, 0);
    encoder_quad_init(ANGLE_ENCODER, ANGLE_ENCODER_A_PIN, ANGLE_ENCODER_B_PIN);

    PID_Init(&angle_ctrl.pid, ANGLE_DEFAULT_KP, ANGLE_DEFAULT_KI, ANGLE_DEFAULT_KD, ANGLE_OUTPUT_MAX);
    angle_ctrl.target_angle = 0;
    angle_ctrl.current_angle = 0;
    angle_ctrl.encoder_zero_count = 0;
    angle_ctrl.control_count = 0;
    encoder_first_read = 1;
    accumulated_encoder_count = 0;
    prev_raw_count = 0;
}

void angle_motor_set_pwm(int32 pwm_value)
{
    if (pwm_value > ANGLE_OUTPUT_MAX) {
        pwm_value = ANGLE_OUTPUT_MAX;
    }
    if (pwm_value < -ANGLE_OUTPUT_MAX) {
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

void angle_control_update(void)
{
    int32 current_count = angle_encoder_get_accumulated_count();
    float error = 0;
    float pid_output = 0;

    angle_ctrl.current_angle = angle_control_get_angle_from_count(current_count);
    error = angle_ctrl.target_angle - angle_ctrl.current_angle;
    pid_output = PID_Compute(&angle_ctrl.pid, angle_ctrl.target_angle, angle_ctrl.current_angle);

    if ((error < ANGLE_DEAD_BAND) && (error > -ANGLE_DEAD_BAND)) {
        pid_output = 0;
        angle_ctrl.pid.Integral = 0;
    }

    angle_motor_set_pwm((int32)pid_output);
    angle_ctrl.control_count++;
}

void angle_control_set_target(int32 target_angle)
{
    if (target_angle > ANGLE_MAX_DEGREE) {
        target_angle = ANGLE_MAX_DEGREE;
    } else if (target_angle < ANGLE_MIN_DEGREE) {
        target_angle = ANGLE_MIN_DEGREE;
    }

    angle_ctrl.target_angle = (float)target_angle;
}

void angle_control_rotate_relative(int32 delta_angle)
{
    angle_control_set_target((int32)(angle_ctrl.current_angle + (float)delta_angle));
}

int32 angle_control_get_current_angle(void)
{
    return (int32)angle_ctrl.current_angle;
}

void angle_control_reset(void)
{
    PID_Reset(&angle_ctrl.pid);
    angle_ctrl.target_angle = 0;
    angle_ctrl.current_angle = 0;
    encoder_first_read = 1;
    accumulated_encoder_count = 0;
    prev_raw_count = 0;
    angle_ctrl.encoder_zero_count = 0;
    angle_ctrl.control_count = 0;
    angle_motor_set_pwm(0);
}