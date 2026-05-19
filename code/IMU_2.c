/*
 * IMU_2.c
 *
 *  Created on: 2025年2月18日
 *      Author: ORRN
 */

#include "zf_common_headfile.h"

float param_kp = 2.12;          // 加速度计收敛速率的比例增益,用于调整加速度计数据的收敛速度
float i_error_x, i_error_y, i_error_z;// 误差积分
float param_ki = 0.0028;        // 陀螺仪收敛速率的积分增益,用于调整陀螺仪数据的积分收敛效果


Imu_gyro_param_t gyro_offset;
Imu_data_param_t imu_data;
euler_param_t    euler_angle;          // 定义全局欧拉角结构体变量
quater_param_t   q_info = {1, 0, 0};  // 定义全局四元数结构体变量

//void IMU_init(void)//IMU初始化
//{
//    imu660ra_init();   //IMU660惯导初始化
//    Init_Gyro_Offset();// 陀螺仪零漂初始化
//}
float Sqrt_Fast(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//去gyro的零点漂移
void Init_Gyro_Offset(void)
{
      unsigned int i;

    gyro_offset.x_data = 0;
    gyro_offset.y_data = 0;
    gyro_offset.z_data = 0;

    for (i = 0; i < 100;)
    {

        imu660ra_get_gyro();// 获取陀螺仪GYRO数据

        // 舍弃杂波数据
      if(imu660ra_gyro_x > -10 && imu660ra_gyro_x < 10&& imu660ra_gyro_y > -10 && imu660ra_gyro_y < 10&& imu660ra_gyro_z > -10 && imu660ra_gyro_z < 10)
                {
                     i++;
                   gyro_offset.x_data += imu660ra_gyro_x;
                   gyro_offset.y_data += imu660ra_gyro_y;
                   gyro_offset.z_data += imu660ra_gyro_z;
                }

        system_delay_ms(1) ;
    }

    gyro_offset.x_data /= 100;
    gyro_offset.y_data /= 100;
    gyro_offset.z_data /= 100;
}

static void Get_Values_ICM(void)
{
    static double last_acc[3] = { 0,0,0 };

    // 一阶低通滤波
    imu_data.acc_x = (((float) imu660ra_acc_x) * alpha)  + last_acc[0] * (1 - alpha);
    imu_data.acc_y = (((float) imu660ra_acc_y) * alpha)  + last_acc[1] * (1 - alpha);
    imu_data.acc_z = (((float) imu660ra_acc_z) * alpha)  + last_acc[2] * (1 - alpha);

        // 更新上一次acc数据
    last_acc[0] = imu_data.acc_x;
    last_acc[1] = imu_data.acc_y;
    last_acc[2] = imu_data.acc_z;

    // 陀螺仪角度转弧度
    imu_data.gyro_x = ((float) imu660ra_gyro_x - gyro_offset.x_data) * PI / 180 / 16.4f;
    imu_data.gyro_y = ((float) imu660ra_gyro_y - gyro_offset.y_data) * PI / 180 / 16.4f;
    imu_data.gyro_z = ((float) imu660ra_gyro_z - gyro_offset.z_data) * PI / 180 / 16.4f;
}

static void Update_AHRS_ICM(float gx, float gy, float gz, float ax, float ay, float az)
{
      // 测量周期的一半，用于四元数微分方程
    float half_t = 0.5 * DELTA_T;
        // 当前的机体坐标系上的重力单位向量,通过四元数计算得到
    float vx, vy, vz;
        // 四元数计算值与加速度计测量值的误差
    float ex, ey, ez;
      // 从四元数结构体中获取当前的四元数分量
    float q0 = q_info.q0;
    float q1 = q_info.q1;
    float q2 = q_info.q2;
    float q3 = q_info.q3;
      // 计算四元数分量的平方和两两乘积
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
//    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 归一化加速度数据得到单位向量
    float norm = Sqrt_Fast(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // 根据当前四元数姿态估算重力分量与实际测量的重力分量对比
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 计算误差向量用于修正陀螺仪数据
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 使用PI控制器修正陀螺仪数据减少积分误差
    i_error_x += half_t * ex;
    i_error_y += half_t * ey;
    i_error_z += half_t * ez;
    gx = gx + param_kp * ex + param_ki * i_error_x;
    gy = gy + param_kp * ey + param_ki * i_error_y;
    gz = gz + param_kp * ez + param_ki * i_error_z;

    // 使用一阶龙格库塔法求解四元数微分方程更新四元数
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;

    // 归一化四元数
    norm = Sqrt_Fast(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q_info.q0 = q0 * norm;
    q_info.q1 = q1 * norm;
    q_info.q2 = q2 * norm;
    q_info.q3 = q3 * norm;
}

void Get_Angles_ICM(void)
{

    float q0,q1,q2,q3;  // 存储四元数的四个分量

    imu660ra_get_acc(); // 获取imu660ra陀螺仪加速度数据

    imu660ra_get_gyro();// 获取imu660ra陀螺仪角加速度数据

    Get_Values_ICM();   // 陀螺仪数据滤波和转化为实际物理值

      // 使用陀螺仪和加速度计的数据更新AHRS（姿态和航向参考系统）算法得到当前的四元数
    Update_AHRS_ICM(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
      // 从更新后的姿态信息中获取四元数的四个分量
    q0 = q_info.q0;
    q1 = q_info.q1;
    q2 = q_info.q2;
    q3 = q_info.q3;

    // 四元数计算欧拉角
    euler_angle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI - 180;
    euler_angle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI;
    euler_angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;

    // 姿态限制
    if (euler_angle.pitch > 0)
    {
        euler_angle.pitch = 180 - euler_angle.pitch;
    }
    else if (euler_angle.pitch < 0)
    {
        euler_angle.pitch = -(180 + euler_angle.pitch);
    }
    if (euler_angle.roll > 0)
    {
        euler_angle.roll = 180 - euler_angle.roll;
    }
    else if (euler_angle.roll < 0)
    {
        euler_angle.roll = -(180 + euler_angle.roll);
    }
    if (euler_angle.yaw > 360)
    {
        euler_angle.yaw -= 360;
    }
    else if (euler_angle.yaw < 0)
    {
        euler_angle.yaw += 360;
    }
}
