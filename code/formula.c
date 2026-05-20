/*
 * UTF-8 详细注释说明：通用数学和限幅工具函数。
 *
 * 模块职责：
 * - 对 int/float 做上下限限制，保护 PID 输出和目标速度。
 * - 处理编码器计数溢出，calculate_delta() 用于从当前计数和上次计数得到增量。
 *
 * 调试重点：
 * - 编码器 delta 不对时，不要只看 Encoder_Get()，也要看 calculate_delta() 的溢出阈值是否匹配编码器计数范围。
 */

/*
 * 主函数/科目一调用链：
 * 1. 科目一记录和自动驾驶会通过 Encoder_Get()、update_state()、rear_motor_encoder_update_10ms() 等路径间接使用 calculate_delta()。
 * 2. 旧 Speed_Control() 使用 PID_Place()/PID_Up() 做速度或转向 PID，当前科目一后轮主链路已改为 rear_motor 自带 PID。
 * 3. KWC_Init()/KWC_UpdateFast() 是 GPS/经纬度滤波预留工具，调 GPS 数据抖动时可以从这里检查滤波参数。
 * 4. 限幅函数用于保护控制量，调车时如果目标值不变化或输出被夹住，要同步检查这些工具函数。
 */


/*
 * formula.c
 *
 *  Created on: 2025年11月21日
 *      Author: 18905
 */

#include "zf_common_headfile.h"

KalmanWithComp klm_lat;
KalmanWithComp klm_lon;

/**
 * 函数说明：KWC_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - kf：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - Q：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - R：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - dt：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - comp_ratio：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：KWC_UpdateFast()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - kf：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - z：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 double 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：calculate_delta()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - current：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - last：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 int32_t 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：Value_Limit_float()。对输入变量进行限幅，防止控制量超过安全范围。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - value：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - min：限幅边界值，用于保护控制输出或参数范围。
 * - max：限幅边界值，用于保护控制输出或参数范围。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Value_Limit_float(float* value , float min ,float max)
{
    *value =((*value)<(min)?(min):((*value)>(max)?(max):(*value)));
}

/**
 * 函数说明：Value_Limit_int()。对输入变量进行限幅，防止控制量超过安全范围。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - value：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - min：限幅边界值，用于保护控制输出或参数范围。
 * - max：限幅边界值，用于保护控制输出或参数范围。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Value_Limit_int(int * value , int min ,int max)
{
    *value =((*value)<(min)?(min):((*value)>(max)?(max):(*value)));
}
/**
 * 函数说明：PID_Place()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - now：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：PID_Up()。执行 PID 参数初始化、清零或控制量计算。
 * 所属模块：通用数学工具模块，给编码器差值、PID 和滤波提供基础函数。
 * 参数说明：
 * - p：PID 控制器结构体指针，函数会读取或修改其中的误差、积分和输出字段。
 * - now：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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


