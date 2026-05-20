/*
 * UTF-8 详细注释说明：旧后轮速度 PID 控制模块。
 *
 * 模块职责：
 * 1. 保存旧工程使用的左右后轮 PID 参数 MoterPID_L / MoterPID_R。
 * 2. 读取 Encoder_Get() 的 delta_l / delta_r 作为速度反馈。
 * 3. 调用 PID_Up() 后通过 Moter_Set() 输出左右电机 PWM。
 *
 * 当前工程注意：
 * - 科目一后轮主要使用 code/rear_motor/rear_motor.c 的新 m/s 闭环。
 * - 本文件仍被遥控/旧测试模式使用，不能删除。
 * - Flash_Main_Read() 可能覆盖 PID 参数，所以源码初始化值不一定等于实际上车参数。
 */

/*
 * 主函数/科目一调用链：
 * 1. Init_All() 调用 Steer_Moter_Init()，该函数已经改为初始化 angle_control 磁编码器转向闭环。
 * 2. CCU61_CH0 中断在 GUANDAO 模式下调用 Steer_Moter_Contral(out_servo)，所以科目一方向盘控制仍从本文件入口进入。
 * 3. 旧 Speed_Control() 现在不负责科目一后轮自动驾驶，科目一后轮已经改由 rear_motor 模块闭环。
 * 4. YAOKONG 模式仍可能使用 Speed_Control()，因此保留旧 PID 和旧电机控制逻辑，避免遥控/旧测试入口断掉。
 */


/*
 * control.c
 *
 *  Created on: 2025年11月21日
 *      Author: 18905
 */
#include "zf_common_headfile.h"


_pid MoterPID_L ={ .Kp =0.5 , .Ki = 1.0 , .Kd = 0 ,  .out_max = 3000 , .out_min = -3000};
_pid MoterPID_R ={ .Kp =0.5 , .Ki = 1.0 , .Kd = 0 ,  .out_max = 3000 , .out_min = -3000};
_pid SteerPID ={ .Kp =1.0 , .Ki = 0 , .Kd = 0.5 ,  .out_max = 13.0f , .out_min = -13.0};
_pid SteerUpPID = {.Kp =0.03 , .Ki = 0.08 , .Kd = 0 ,  .out_max = 13.0f , .out_min = -13.0 ,.target_val =0.0f};
_pid Steer_S_Loop = {.Kp =500.0f , .Ki = 0 , .Kd = 0.2 ,  .out_max = 9000.0f , .out_min = -9000.0f ,.target_val =0.0f};
_pid Steer_D_Loop = {.Kp =1000.0f , .Ki = 0.0f , .Kd = 0.1f ,  .out_max = 9000.0f , .out_min = -9000.0f ,.target_val =0.0f , .limit = 100.0f};

MOTER_control_mode conrtol_mode = IDLE;
int num =0;
int num1 =500;
float C;
uint8 Main_Key_Flag = 0;

int16 Steer_Mid_Value = 0;
int angle = 0;
int angle_speed = 0;
int angle_test = 0;
/**
 * 函数说明：Steer_Moter_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Moter_Init(void)
{
    angle_control_init();
    angle_control_set_target(0);
}


/**
 * 函数说明：Speed_Control()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - tar_l：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * - tar_r：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Speed_Control(float tar_l, float tar_r)
{
    static int all_l = 0;
    static int all_r = 0;
    static float Kp =0;

    MoterPID_L.target_val =tar_l;
    MoterPID_R.target_val =tar_r;
    Value_Limit_float(&MoterPID_L.target_val , -200 , 200);
    Value_Limit_float(&MoterPID_R.target_val , -200 , 200);
    Encoder_Get(&Speed_ecd);
    angle_speed += Speed_ecd.delta_r;
    PID_Up(&MoterPID_L , (float)Speed_ecd.delta_l);
    PID_Up(&MoterPID_R , (float)Speed_ecd.delta_r);
    all_l += MoterPID_L.out;
    all_r += MoterPID_R.out;
    Value_Limit_int(&all_l , -MOTER_MAX , MOTER_MAX);
    Value_Limit_int(&all_r , -MOTER_MAX , MOTER_MAX);
    Moter_Set(all_l , all_r );

}


/**
 * 函数说明：Steer_Moter_Contral()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - servo_out：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Moter_Contral(float servo_out)
{
    Value_Limit_float(&servo_out, ANGLE_MIN_DEGREE, ANGLE_MAX_DEGREE);
    angle_control_set_target((int32)servo_out);
    angle_control_update();

    angle = angle_control_get_current_angle();
    angle_speed = (int)angle_ctrl.pid.Output;


}
/**
 * 函数说明：Steer_UpPID()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - error：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_UpPID(_pid*p ,float error)
{
    static float final_out = 0;
    PID_Up(p, -error);
    final_out += p->out;
    Value_Limit_float(&final_out , -13.0 ,13.0);
    Steer_set(SERVO_MOTOR_MID-final_out);
}
/**
 * 函数说明：Steer_Control()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - tar：目标值，单位由函数名决定，常见为角度 deg、速度 m/s 或 PWM 计数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_Control(int tar)
{
    static int num = 0;
    num = (tar - 500)*(tar - 500)*(tar - 500)/2000000+80;
    Steer_set(num);

}

/**
 * 函数说明：Steer_PID()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：旧版速度/转向控制兼容模块，目前科目一主要只借用其中的转向接口。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - error：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_PID(_pid*p ,float error)
{
    static int num = 0;
    PID_Place(p,error);
    num = (int)p->out;
    Steer_set(SERVO_MOTOR_MID-num);

}


