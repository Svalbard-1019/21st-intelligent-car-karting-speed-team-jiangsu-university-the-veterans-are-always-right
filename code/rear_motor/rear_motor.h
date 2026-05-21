/*
 * UTF-8 详细注释说明：后轮 m/s 闭环模块接口和参数。
 *
 * 这里集中定义轮径、减速比、编码器 PPR、速度上限、PID 参数和前馈系数。
 * 空载/载人参数建议都保留注释，现场测试时只切换一组，避免不知道当前跑的是哪套参数。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 启动后直接调用 rear_motor_init() 初始化 PWM、方向 GPIO 和后轮速度闭环状态。
 * 2. CCU61_CH0 中断在 GUANDAO/DAOCHE/RACK_TEST 模式下每 10ms 调用 rear_motor_encoder_update_10ms() 采样 TIM2 左后轮编码器。
 * 3. 科目一自动驾驶时，portion_1()/guandao_trace() 先算 out_v_l/out_v_r，主循环末尾 Guandao_Rear_Motor_Update() 把旧惯导速度乘 GUANDAO_SPEED_TO_MPS 换算为 m/s。
 * 4. Guandao_Rear_Motor_Update() 调用 rear_motor_set_target_mps() 和 rear_motor_pid_update_100ms()，最后由 rear_motor_set_pwm() 同步驱动左右后轮。
 */


/*
 * rear_motor.h
 *
 * 后轮独立驱动模块
 * 目标速度 m/s -> 编码器目标脉冲 -> 前馈 + PID 修正 -> PWM 输出
 */

#ifndef CODE_REAR_MOTOR_H_
#define CODE_REAR_MOTOR_H_

#include "zf_common_headfile.h"

/* 车轮与编码器参数 */
#define REAR_WHEEL_DIAMETER_M        0.24f
#define REAR_GEAR_RATIO              1.6f
#define REAR_ENCODER_PPR             1024
#define REAR_EFFECTIVE_PPR           ((float)REAR_ENCODER_PPR * REAR_GEAR_RATIO)
#define REAR_WHEEL_CIRCUM_M          (3.14159265358979323846f * REAR_WHEEL_DIAMETER_M)

/* PID 参数: 载人旧工程参数 (当前科目一测试使用) */
#define REAR_KP                 10.0f
#define REAR_KI                 0.3f
#define REAR_KD                 0.8f
#define REAR_FF_GAIN            13.0f

/* PID 参数: 空载实测 (2026-05-13), 架上测试需要时切回 */
// #define REAR_KP              8.0f
// #define REAR_KI              0.5f
// #define REAR_KD              0.2f
// #define REAR_FF_GAIN         10.0f

/* PID 参数: 空载微调, 降低架空超调, 需要时切回 */
// #define REAR_KP              6.5f
// #define REAR_KI              0.25f
// #define REAR_KD              0.3f
// #define REAR_FF_GAIN         9.0f
#define REAR_PWM_HARD_LIMIT     9500
#define REAR_PWM_RATE_LIMIT     1000
#define REAR_INTEGRAL_LIMIT     2000.0f
#define REAR_INTEGRAL_THRESHOLD 60.0f
#define REAR_ENCODER_DELTA_ABS_MAX 300

/* 速度限幅 (架上测试) */
#define REAR_SPEED_MAX_MPS      5.0f
#define REAR_SPEED_MIN_MPS      -5.0f

/* 公开接口 */
/**
 * 接口说明：rear_motor_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_init(void);
/**
 * 接口说明：rear_motor_stop()。立即撤销目标输出并关闭执行器，常用于安全停车或目标速度为 0 的场景。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_stop(void);

/**
 * 接口说明：rear_motor_set_target_mps()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - target_mps：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_set_target_mps(float target_mps);
/**
 * 接口说明：rear_motor_encoder_update_10ms()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_encoder_update_10ms(void);
/**
 * 接口说明：rear_motor_pid_update_100ms()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void rear_motor_pid_update_100ms(void);

/**
 * 接口说明：rear_motor_get_target_mps()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float  rear_motor_get_target_mps(void);
/**
 * 接口说明：rear_motor_get_speed_mps()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float  rear_motor_get_speed_mps(void);
/**
 * 接口说明：rear_motor_get_pwm()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int16 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int16  rear_motor_get_pwm(void);
/**
 * 接口说明：rear_motor_get_encoder_10ms()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int16 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int16  rear_motor_get_encoder_10ms(void);
/**
 * 接口说明：rear_motor_get_encoder_100ms()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：后轮 m/s 速度闭环模块，是当前科目一实际驱动后轮的主要模块。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int32  rear_motor_get_encoder_100ms(void);

#endif /* CODE_REAR_MOTOR_H_ */
