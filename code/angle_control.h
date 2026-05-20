/*
 * UTF-8 详细注释说明：前轮转向电机闭环控制接口与硬件宏。
 *
 * 本头文件集中定义转向电机的 PWM、编码器、机械零点和 PID 控制接口。
 * 主流程只需要调用 Steer_Moter_Init() 初始化，再周期性调用 Steer_Moter_Contral(target_deg)。
 *
 * 单位约定：
 * - 对外目标角度使用“度”。
 * - 编码器原始计数会在 angle_control.c 内部换算到角度。
 * - PWM 限幅用于保护电机和驱动，不建议在未架空测试时直接调大。
 */

/*
 * 主函数/科目一调用链：
 * 1. 主函数 core0_main() 调用 Init_All()，Init_All() 内部会进入 Steer_Moter_Init()，最终调用 angle_control_init() 完成转向 PWM 和 TIM4 编码器初始化。
 * 2. 科目一自动驾驶时，主循环 portion_1()/guandao_trace() 计算 out_servo；CCU61_CH0 中断按周期调用 Steer_Moter_Contral(out_servo)。
 * 3. Steer_Moter_Contral() 内部调用 angle_control_set_target() 与 angle_control_update()，把科目一规划出的方向角转换成前轮转向电机 PWM。
 * 4. Rack_Test 的 Stage1/Stage3 也复用同一条转向链路，用来单独验证方向盘角度闭环和直线保持。
 */


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

/**
 * 结构体说明：用于集中保存本模块的一组状态量/参数，字段通常会被初始化函数清零，并在周期函数中持续更新。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
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
/**
 * 接口说明：angle_control_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_init(void);

// 读取当前角度并执行一次位置式 PID 计算
/**
 * 接口说明：angle_control_update()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_update(void);

// 设置目标角度，自动限制范围
/**
 * 接口说明：angle_control_set_target()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - target_angle：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_set_target(int32 target_angle);

// 在当前角度基础上旋转指定角度
/**
 * 接口说明：angle_control_rotate_relative()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - delta_angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_rotate_relative(int32 delta_angle);

// 根据控制器输出设置电机 PWM 占空比
/**
 * 接口说明：angle_motor_set_pwm()。根据符号和限幅要求输出 PWM，占空比正负通常对应电机方向。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - pwm_value：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_motor_set_pwm(int32 pwm_value);

// 获取当前角度
/**
 * 接口说明：angle_control_get_current_angle()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 int32 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int32 angle_control_get_current_angle(void);

// 重置 PID 状态，重新记录零位，关闭电机
/**
 * 接口说明：angle_control_reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：前轮转向电机闭环模块，主要由旧转向接口 Steer_Moter_Contral() 间接调用。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void angle_control_reset(void);

#endif /* CODE_ANGLE_CONTROL_H_ */
