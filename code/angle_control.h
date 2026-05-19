/*
 * angle_control.h
 *
 * Description: 转向电机角度控制模块
 */

#ifndef CODE_ANGLE_CONTROL_H_
#define CODE_ANGLE_CONTROL_H_
#include "zf_common_headfile.h"
#include "PID.h"

// 角度电机 PWM 引脚定义
#define ANGLE_PWM_IN1       ATOM0_CH7_P02_7        // 正转 PWM
#define ANGLE_PWM_IN2       ATOM0_CH6_P02_6        // 反转 PWM

// 编码器定义
#define ANGLE_ENCODER       TIM4_ENCODER
#define ANGLE_ENCODER_A_PIN TIM4_ENCODER_CH1_P02_8 // 编码器 A 相
#define ANGLE_ENCODER_B_PIN TIM4_ENCODER_CH2_P00_9 // 编码器 B 相

// 电机参数定义
#define ANGLE_PPR           1024                   // quad模式下每转脉冲数(硬件1024/4)
#define ANGLE_GEAR_RATIO    300                    // 减速比
#define ANGLE_MAX_DEGREE    60                     // 最大目标角度
#define ANGLE_MIN_DEGREE    -60                    // 最小目标角度
#define ANGLE_DEFAULT_KP    200.0f
#define ANGLE_DEFAULT_KI    4.0f
#define ANGLE_DEFAULT_KD    10.0f
#define ANGLE_OUTPUT_MAX    10000
#define ANGLE_DEAD_BAND     0.1f

typedef struct {
    PID_TypeDef pid;          // 角度环 PID 控制器状态
    float target_angle;       // 目标角度
    float current_angle;      // 当前角度
    int32 encoder_zero_count; // 零位对应的编码器计数
    uint32 control_count;     // 控制循环执行次数
} AngleControl_TypeDef;

extern AngleControl_TypeDef angle_ctrl;
extern int32 accumulated_encoder_count;

// 初始化角度控制模块，配置 PWM、编码器、PID 控制器
void angle_control_init(void);

// 读取当前角度并执行一次位置式 PID 计算
void angle_control_update(void);

// 设置目标角度，自动限制范围
void angle_control_set_target(int32 target_angle);

// 在当前角度基础上旋转指定角度
void angle_control_rotate_relative(int32 delta_angle);

// 根据控制器输出设置电机 PWM 占空比
void angle_motor_set_pwm(int32 pwm_value);

// 获取当前角度
int32 angle_control_get_current_angle(void);

// 重置 PID 状态，重新记录零位，关闭电机
void angle_control_reset(void);

#endif /* CODE_ANGLE_CONTROL_H_ */
