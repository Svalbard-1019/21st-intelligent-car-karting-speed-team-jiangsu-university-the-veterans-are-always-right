/*
 * UTF-8 详细注释说明：通用 PID 运算实现。
 *
 * 模块职责：
 * - PID_Init() 初始化 Kp/Ki/Kd、限幅和死区。
 * - PID_Up() 按目标值与反馈值计算输出。
 *
 * 注意事项：
 * - 本 PID 被旧后轮、转向或其他模块复用，改算法会影响多个控制环。
 * - 积分项和输出限幅是防止电机突然打满的重要保护。
 */

/*
 * 主函数/科目一调用链：
 * 1. angle_control_init() 调用 PID_Init() 初始化前轮转向角度环参数。
 * 2. angle_control_update() 周期调用 PID_Compute()，用目标角和当前磁编码器角度计算转向 PWM。
 * 3. angle_control_reset() 或误差进入死区时会调用/等价执行 PID_Reset()，避免积分残留导致方向盘自己偏。
 * 4. 后轮 rear_motor 当前使用独立 PID 状态，没有直接复用 PID_TypeDef。
 */


/*
 * PID.c
 *
 * Description: 位置式 PID 控制算法
 */

#include "zf_common_headfile.h"

// 初始化 PID 控制器
/**
 * 函数说明：PID_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
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
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->MaxOutput = max_output;
    pid->IntegralMax = 2000;  // 积分限幅

    PID_Reset(pid);
}

// 清除 PID 历史状态
/**
 * 函数说明：PID_Reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 参数说明：
 * - pid：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void PID_Reset(PID_TypeDef *pid) {
    pid->Target = 0;
    pid->Current = 0;
    pid->Error = 0;
    pid->LastError = 0;
    pid->Integral = 0;
    pid->Output = 0;
}

// 计算位置式 PID 输出
/**
 * 函数说明：PID_Compute()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：轻量 PID 控制器模块，当前主要被 angle_control 转向闭环调用。
 * 参数说明：
 * - pid：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - target：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - current：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
float PID_Compute(PID_TypeDef *pid, float target, float current) {
    pid->Target = target;
    pid->Current = current;
    pid->Error = pid->Target - pid->Current;

    // 积分累加，超限直接链接到限幅
    pid->Integral += pid->Error;
    if (pid->Integral > pid->IntegralMax) pid->Integral = pid->IntegralMax;
    if (pid->Integral < -pid->IntegralMax) pid->Integral = -pid->IntegralMax;

    // 位置式PID，可直接得到
    pid->Output = (pid->Kp * pid->Error) +
                  (pid->Ki * pid->Integral) +
                  (pid->Kd * (pid->Error - pid->LastError));

    pid->LastError = pid->Error;

    // 双边限幅，防止转速过大。
    if (pid->Output > pid->MaxOutput)  pid->Output = pid->MaxOutput;
    if (pid->Output < -pid->MaxOutput) pid->Output = -pid->MaxOutput;

    return pid->Output;
}
