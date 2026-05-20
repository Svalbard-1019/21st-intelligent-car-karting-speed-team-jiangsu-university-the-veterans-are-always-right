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
 * 主函数/科目一调用链：
 * 1. 主函数 core0_main() 调用 Init_All()，Init_All() 内部会进入 Steer_Moter_Init()，最终调用 angle_control_init() 完成转向 PWM 和 TIM4 编码器初始化。
 * 2. 科目一自动驾驶时，主循环 portion_1()/guandao_trace() 计算 out_servo；CCU61_CH0 中断按周期调用 Steer_Moter_Contral(out_servo)。
 * 3. Steer_Moter_Contral() 内部调用 angle_control_set_target() 与 angle_control_update()，把科目一规划出的方向角转换成前轮转向电机 PWM。
 * 4. Rack_Test 的 Stage1/Stage3 也复用同一条转向链路，用来单独验证方向盘角度闭环和直线保持。
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

/**
 * 函数说明：encoder_get_accumulated_count()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 static int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：angle_control_get_angle_from_count()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - count：编码器相关输入或计数值，用于速度、里程或角度换算。
 * 返回值：返回 static float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
static float angle_control_get_angle_from_count(int32 count)
{
    return (float)(count - angle_ctrl.encoder_zero_count) * ANGLE_DEGREE_PER_PULSE;
}

/**
 * 函数说明：angle_control_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：angle_motor_set_pwm()。根据符号和限幅要求输出 PWM，占空比正负通常对应电机方向。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - pwm_value：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：angle_control_update()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：angle_control_set_target()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - target_angle：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_set_target(int32 target_angle) {
    if (target_angle > ANGLE_MAX_DEGREE) {
        angle_ctrl.target_angle = ANGLE_MAX_DEGREE;
    } else if (target_angle < ANGLE_MIN_DEGREE) {
        angle_ctrl.target_angle = ANGLE_MIN_DEGREE;
    } else {
        angle_ctrl.target_angle = (float)target_angle;
    }
}

/**
 * 函数说明：angle_control_rotate_relative()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - delta_angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_rotate_relative(int32 delta_angle) {
    angle_control_set_target((int32)(angle_ctrl.current_angle + (float)delta_angle));
}

/**
 * 函数说明：angle_control_get_current_angle()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int32 angle_control_get_current_angle(void) {
    return (int32)angle_ctrl.current_angle;
}

/**
 * 函数说明：angle_control_reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
