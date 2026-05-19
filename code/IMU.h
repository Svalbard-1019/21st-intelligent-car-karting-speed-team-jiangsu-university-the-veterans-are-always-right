/*
 * IMU.h
 *
 *  Created on: 2025年1月21日
 *      Author: ORRN
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

//结构体声明
typedef struct{
    float Xdata;   //零飘参数X
    float Ydata;   //零飘参数Y
    float Zdata;   //零飘参数Z
}gyro_param_t ;

typedef struct{
    float acc_x;   //x轴加速度
    float acc_y;   //y轴加速度
    float acc_z;   //z轴加速度

    float gyro_x;  //x轴角速度
    float gyro_y;  //y轴角速度
    float gyro_z;  //z轴角速度
}IMU_param_t ;

extern int IMU_1_Open_flag;
extern float Yaw_1;
extern IMU_param_t  IMU_Data;
void IMU_init(void);
void IMU_gyro_Offset_Init(void);
void IMU_GetValues(void);
void IMU_Handle_180(void);
void IMU_data_get(void);
void IMU_text(void);

extern float Yaw_1;



#endif /* CODE_IMU_H_ */
