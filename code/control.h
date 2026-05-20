/*
 * UTF-8 详细注释说明：车辆控制模式枚举与旧后轮 PID 接口。
 *
 * conrtol_mode 是主循环和中断之间共享的控制状态：
 * - IDLE：停止输出。
 * - YAOKONG：遥控模式，旧 Speed_Control 控制后轮。
 * - GUANDAO：惯导/科目一自动驾驶模式。
 * - DAOCHE：倒车模式。
 * - RACK_TEST：架子测试模式。
 *
 * 调试时先确认 main_mode 和 conrtol_mode 是否匹配，否则可能出现屏幕在某个模式但中断输出走另一套逻辑。
 */

/*
 * 主函数/科目一调用链：
 * 1. Init_All() 调用 Steer_Moter_Init()，该函数已经改为初始化 angle_control 磁编码器转向闭环。
 * 2. CCU61_CH0 中断在 GUANDAO 模式下调用 Steer_Moter_Contral(out_servo)，所以科目一方向盘控制仍从本文件入口进入。
 * 3. 旧 Speed_Control() 现在不负责科目一后轮自动驾驶，科目一后轮已经改由 rear_motor 模块闭环。
 * 4. YAOKONG 模式仍可能使用 Speed_Control()，因此保留旧 PID 和旧电机控制逻辑，避免遥控/旧测试入口断掉。
 */


/*
 * control.h
 *
 *  Created on: 2025年11月21日
 *      Author: 18905
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

//外部变量 extern
#define VEER_MOTOR_MID         180
#define VEER_MOTOR_MAX         280
#define VEER_MOTOR_MIN         92

/**
 * 枚举说明：用于描述运行模式、阶段或状态，实际取值会被菜单、控制流程或调试显示读取。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 字段/取值：结构体成员或枚举项旁边保留行内注释；调试时优先关注索引、目标值、反馈值和输出量。
 * 科目一关系：这些状态会沿着“菜单选择 -> 主循环模式 -> 中断周期执行器输出”的链路被读取或更新。
 * 使用边界：不要在未初始化前直接使用，也不要在多个控制模式之间混用同一份状态而不清零。
 */
typedef  enum{
    IDLE,
    YAOKONG,
    GUANDAO,
    GPS,
    DAOCHE,
    RACK_TEST
}MOTER_control_mode;

extern MOTER_control_mode conrtol_mode;

extern int angle ;
extern int angle_speed ;
extern int angle_test ;
extern  _pid MoterPID_L ;
extern  _pid MoterPID_R ;
extern  _pid SteerPID ;
extern _pid SteerUpPID ;
extern _pid Steer_S_Loop;
extern _pid Steer_D_Loop;
extern int num;
extern int num1;
extern float C;
extern uint8 Main_Key_Flag;

extern int16 Steer_Mid_Value;
//宏定义

//函数

/**
 * 接口说明：Speed_Control()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - tar_l：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - tar_r：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Speed_Control( float tar_l, float tar_r);
/**
 * 接口说明：Steer_Control()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - tar：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Control(int tar);
/**
 * 接口说明：Steer_PID()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - error：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_PID(_pid*p ,float error);
/**
 * 接口说明：Steer_UpPID()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - error：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_UpPID(_pid*p ,float error);
/**
 * 接口说明：Steer_Moter_Contral()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - servo_out：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Moter_Contral(float servo_out);
/**
 * 接口说明：Steer_Mid_Cheak()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
uint8 Steer_Mid_Cheak(void);
/**
 * 接口说明：Steer_Moter_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Moter_Init(void);

#endif /* CODE_CONTROL_H_ */
