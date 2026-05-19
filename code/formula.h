/*
 * formula.h
 *
 *  Created on: 2025年11月21日
 *      Author: 18905
 */

#ifndef CODE_FORMULA_H_
#define CODE_FORMULA_H_

//外部变量 extern


//宏定义
typedef struct
{
    float target_val;               //目标值
    float actual_val;               //实际值
//    float integral;                 //定义积分值

    float err;                      //定义偏差值
    float err_last;                 //  e(k-1)
    float err_previous;             //  e(k-2)

    float Kp;               //定义比例、积分、微分系数
    float Ki;               //定义比例、积分、微分系数
    float Kd;               //定义比例、积分、微分系数
    float limit;            //积分限幅
    float p_result;         //比例、积分、微分运算结果
    float i_result;         //比例、积分、微分运算结果
    float d_result;         //比例、积分、微分运算结果
//    float inte_exce;        //积分过度

    float out;                 //输出
    float out_max;                 //输出限幅
    float out_min;                 //输出限幅
} _pid;

typedef struct {
    // 卡尔曼核心
    double x;      // 滤波后的位置
    double P;      // 误差方差

    // 参数
    double Q;      // 过程噪声
    double R;      // 测量噪声

    // 滞后补偿
    double last_x;     // 上一次滤波值
    double velocity;   // 估计速度（m/s）
    double dt;         // 采样周期（秒）

    // 配置
    double comp_ratio; // 补偿系数（0~1）
} KalmanWithComp;

extern KalmanWithComp klm_lat;
extern KalmanWithComp klm_lon;

//函数

void PID_Place(_pid*p,float now);
void PID_Up(_pid*p,float now);
void Value_Limit_float(float* value , float min ,float max);
void Value_Limit_int(int * value , int min ,int max);
int32_t calculate_delta(int16_t current, int16_t last);
void KWC_Init(KalmanWithComp *kf, double Q, double R, double dt, double comp_ratio);
double KWC_UpdateFast(KalmanWithComp *kf, double z);
#endif /* CODE_FORMULA_H_ */
