/*
 * angle_control.h
 *
 * Front steering motor closed-loop angle control.
 *
 * Hardware:
 *   Motor driver: IN1 PWM=P02_7, IN2 PWM=P02_6
 *   Encoder: TIM4, A=P02_8, B=P00_9
 */

#ifndef ANGLE_CONTROL_H_
#define ANGLE_CONTROL_H_

#include "zf_common_typedef.h"
#include "zf_driver_encoder.h"
#include "zf_driver_pwm.h"
#include "PID.h"

#define ANGLE_PWM_IN1       ATOM0_CH7_P02_7
#define ANGLE_PWM_IN2       ATOM0_CH6_P02_6
#define ANGLE_ENCODER       TIM4_ENCODER
#define ANGLE_ENCODER_A_PIN TIM4_ENCODER_CH1_P02_8
#define ANGLE_ENCODER_B_PIN TIM4_ENCODER_CH2_P00_9

#define ANGLE_PWM_FREQ      10000
#define ANGLE_PPR           1024
#define ANGLE_GEAR_RATIO    600

#define ANGLE_MAX_DEGREE    360
#define ANGLE_MIN_DEGREE    -360
#define ANGLE_DEFAULT_KP    30.0f
#define ANGLE_DEFAULT_KI    0.0f
#define ANGLE_DEFAULT_KD    0.0f
#define ANGLE_OUTPUT_MAX    PWM_DUTY_MAX
#define ANGLE_DEAD_BAND     5.0f

typedef struct {
    PID_TypeDef pid;
    float target_angle;
    float current_angle;
    int32 encoder_zero_count;
    uint32 control_count;
} AngleControl_TypeDef;

extern AngleControl_TypeDef angle_ctrl;

void  angle_control_init(void);
void  angle_control_update(void);
void  angle_control_set_target(int32 target_angle);
void  angle_control_rotate_relative(int32 delta_angle);
void  angle_motor_set_pwm(int32 pwm_value);
int32 angle_control_get_current_angle(void);
void  angle_control_reset(void);

#endif