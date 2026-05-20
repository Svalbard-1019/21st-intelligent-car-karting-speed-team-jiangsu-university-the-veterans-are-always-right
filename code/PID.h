/*
 * UTF-8 详细注释说明：PID 参数结构体和通用接口。
 *
 * PID_Incremental 结构保存目标、反馈、误差、积分和输出限幅。
 * 调参时要区分后轮 PID、转向 PID 和其他 PID 实例，不能只看结构体名。
 */

/*
 * 主函数/科目一调用链：
 * 1. angle_control_init() 调用 PID_Init() 初始化前轮转向角度环参数。
 * 2. angle_control_update() 周期调用 PID_Compute()，用目标角和当前磁编码器角度计算转向 PWM。
 * 3. angle_control_reset() 或误差进入死区时会调用/等价执行 PID_Reset()，避免积分残留导致方向盘自己偏。
 * 4. 后轮 rear_motor 当前使用独立 PID 状态，没有直接复用 PID_TypeDef。
 */


/*
 * PID.h
 *
 * Description: 位置式 PID 控制算法
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

/**
 * 结构体说明：用于集中保存本模块的一组状态量/参数，字段通常会被初始化函数清零，并在周期函数中持续更新。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
typedef struct {
    float Target;      // 目标值
    float Current;     // 当前反馈值
    float Error;       // 当前误差
    float LastError;   // 上一次误差
    float Integral;    // 积分累计值
    float Kp, Ki, Kd;  // PID 系数
    float Output;      // 当前输出值
    float MaxOutput;   // 输出限幅，默认 10000
    float IntegralMax; // 积分限幅
} PID_TypeDef;

// 函数声明
/**
 * 接口说明：PID_Compute()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 参数说明：
 * - pid：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - target：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - current：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float PID_Compute(PID_TypeDef *pid, float target, float current);
/**
 * 接口说明：PID_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 参数说明：
 * - pid：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - kp：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - ki：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - kd：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - max_output：限幅边界值，用于保护控制输出或参数范围。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output);
/**
 * 接口说明：PID_Reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 参数说明：
 * - pid：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void PID_Reset(PID_TypeDef *pid);

#endif /* CODE_PID_H_ */
