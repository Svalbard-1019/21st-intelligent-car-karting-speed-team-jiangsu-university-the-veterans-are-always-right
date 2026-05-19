/*
 * IMU_2.h
 *
 *  Created on: 2025年2月18日
 *      Author: ORRN
 */

#ifndef CODE_IMU_2_H_
#define CODE_IMU_2_H_


#define DELTA_T     0.0051f     // 定义计算周期,5ms
#define alpha           0.3f    // 定义一阶低通滤波器的alpha值,用于平滑加速度数据



// 存储陀螺仪的零点偏移数据
typedef struct {
    float x_data;
    float y_data;
    float z_data;
} Imu_gyro_param_t;

// 存储陀螺仪和加速度计的数据
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} Imu_data_param_t;

// 存储四元数参数,四元数用于表示旋转
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

// 存储欧拉角参数,欧拉角用于表示三维空间中的旋转
typedef struct {
        float pitch;
        float roll;
        float yaw;
} euler_param_t;
extern  euler_param_t    euler_angle ;
void Init_Gyro_Offset(void);
void Get_Angles_ICM(void);
void IMU_init(void);//IMU初始化

#endif /* CODE_IMU_2_H_ */
