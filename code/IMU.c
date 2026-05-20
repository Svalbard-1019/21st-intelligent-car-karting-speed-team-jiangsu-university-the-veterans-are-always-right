/*
 * UTF-8 详细注释说明：当前使用的 IMU963RA 初始化和姿态读取入口。
 *
 * 模块职责：
 * 1. 初始化 IMU963RA。
 * 2. 周期读取陀螺仪/加速度数据。
 * 3. 更新 Yaw_1 等全局姿态量，供惯导 update_state() 使用。
 *
 * 调试重点：
 * - 科目一方向不稳时先看 Yaw_1 是否连续、是否静止漂移过大。
 * - 如果航向反向，自动驾驶 A 角会长期接近 ±180°。
 */

/*
 * 主函数/科目一调用链：
 * 1. Init_All() 中初始化 IMU963RA，随后 CCU61_CH1 中断周期读取陀螺仪和姿态数据。
 * 2. CCU61_CH1 中断调用 imu963ra_get_gyro() 和 IMU_GetValues()，更新 IMU_Data 与 Yaw_1。
 * 3. 记录模式 update_state() 使用 Yaw_1/IMU 航向把编码器里程积分成 X/Y/Theta 路线点。
 * 4. 自动驾驶 pursuit_contral_mode() 根据当前位姿和目标点方向算角度误差，所以 IMU 航向正负会直接影响车是否朝路线追踪。
 */


/*
 * IMU.c
 *
 *  Created on: 2025年1月21日
 *      Author: ORRN
 */
#include "zf_common_headfile.h"

gyro_param_t Gyro_Offset;
IMU_param_t  IMU_Data;

float Yaw_1 = 0;
float Roll_1 = 0;
float Picth_1 = 0;

int IMU_1_Open_flag = 0;//开启IMU标志位

/**
 * 函数说明：IMU_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：当前使用的 IMU963RA 姿态模块，给科目一提供车头航向 Yaw_1。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void IMU_init(void)//IMU初始化
{
    imu963ra_init();  //IMU963RA惯导初始化
    IMU_gyro_Offset_Init();// 陀螺仪零漂初始化
}

/**
 * 函数说明：IMU_gyro_Offset_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：当前使用的 IMU963RA 姿态模块，给科目一提供车头航向 Yaw_1。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void IMU_gyro_Offset_Init(void)
{

    Gyro_Offset.Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
        imu963ra_get_gyro();
        Gyro_Offset.Zdata += imu963ra_gyro_z;
        system_delay_ms(5);   // 最大 1Khz
    }

    Gyro_Offset.Zdata /= 1000.0;
}

/**
 * 函数说明：IMU_GetValues()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：当前使用的 IMU963RA 姿态模块，给科目一提供车头航向 Yaw_1。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void IMU_GetValues(void)//将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
{

    IMU_Data.gyro_z = ((float) imu963ra_gyro_z - Gyro_Offset.Zdata)* PI / 180.0f/ 16.384f;

    if(IMU_Data.gyro_z<0.025&&IMU_Data.gyro_z>-0.025)//滤波
    {
        Yaw_1-=0;
    }
    else
    {
        IMU_Handle_180();
     }

}

/**
 * 函数说明：IMU_Handle_180()。处理 IMU/陀螺仪数据，用于更新车体姿态和航向角。
 * 所属模块：当前使用的 IMU963RA 姿态模块，给科目一提供车头航向 Yaw_1。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void IMU_Handle_180(void)
{

    Yaw_1-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.00916  );//(积分过程)本来是逆时针为正,现在改为顺时针为正

   if(Yaw_1>180 && Yaw_1<=360)
    {
        Yaw_1-=360;
    }
    else if(Yaw_1<(-180) && Yaw_1>=(-360))
    {
        Yaw_1+=360;
    }

}

/**
 * 函数说明：IMU_data_get()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：当前使用的 IMU963RA 姿态模块，给科目一提供车头航向 Yaw_1。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void IMU_data_get(void)
{
    imu963ra_get_gyro();

    if(IMU_1_Open_flag==1)
    {
        IMU_GetValues();
    }
}

//void IMU_text(void)
//{
//    IMU_1_Open_flag=1;
//    while(1)
//    {
//        ips_show_string(8*0, 16*0, "Yaw:");      ips_show_float(8*5,16*0,Yaw_1,3,6);
//    }
//}
