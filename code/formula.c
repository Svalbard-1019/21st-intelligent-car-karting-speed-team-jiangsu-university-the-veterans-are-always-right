/*
 * formula.c
 *
 *  Created on: 2025年11月21日
 *      Author: 18905
 */

#include "zf_common_headfile.h"

KalmanWithComp klm_lat;
KalmanWithComp klm_lon;

void KWC_Init(KalmanWithComp *kf, double Q, double R, double dt, double comp_ratio) {
    kf->x = 0.0f;
    kf->P = 1.0;      // 初始不确定度大 1e-9f
    kf->Q = Q;
    kf->R = R;

    kf->last_x = 0.0f;
    kf->velocity = 0.0f;
    kf->dt = dt;
    kf->comp_ratio = comp_ratio;  // 推荐 0.3~0.5
}

double KWC_UpdateFast(KalmanWithComp *kf, double z) {
    // 卡尔曼滤波
        // 卡尔曼滤波
        float P_pred = kf->P + kf->Q;
        float K = P_pred / (P_pred + kf->R);
        float filtered = kf->x + K * (z - kf->x);
        kf->P = (1.0f - K) * P_pred;

        // 速度估计
        float raw_vel = (filtered - kf->last_x) / kf->dt;
        kf->velocity = 0.7f * kf->velocity + 0.3f * raw_vel;

        // 滞后补偿
        float output = filtered + kf->velocity * kf->dt * kf->comp_ratio;

        // 更新状态
        kf->last_x = filtered;
        kf->x = filtered;

        return output;

}

 int32_t calculate_delta(int16_t current, int16_t last)
{
    int32_t delta = (int32_t)current - (int32_t)last;
    if (delta > 32767)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }

    return delta;
}

void Value_Limit_float(float* value , float min ,float max)
{
    *value =((*value)<(min)?(min):((*value)>(max)?(max):(*value)));
}

void Value_Limit_int(int * value , int min ,int max)
{
    *value =((*value)<(min)?(min):((*value)>(max)?(max):(*value)));
}
void PID_Place(_pid*p ,float now)
{
    p->err = now;

    p->err_last = p->err;

    if(p->Ki == 0)p->i_result =0;
    else if (p->Ki != 0)p->i_result +=p->Ki*p->err;
    if(p->i_result >p->limit)p->i_result = p->limit;
    if(p->i_result < -p->limit)p->i_result = p->limit;

    p->p_result =p->Kp * p->err;

    p->d_result =p->Kd * (p->err - p->err_last) ;

    p->out = p->p_result + p->i_result + p->d_result;


   Value_Limit_float(&p->out,p->out_min,p->out_max);



}

void PID_Up(_pid*p  ,float now)
{



    p->err = p->target_val - now;



    p->p_result = p->Kp * (p->err - p->err_last);

    p->i_result = p->Ki * p->err;

    p->d_result = p->Kd * (p->err + p->err_previous - 2.0 * p->err_last);

    p->out = p->p_result + p->i_result + p->d_result;

    if(p->out >p->out_max)p->out = p->out_max;
    if(p->out <p->out_min)p->out = p->out_min;

    p->err_last = p->err;
    p->err_previous  = p->err_last  ;


}


