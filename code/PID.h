/*
 * UTF-8 详细注释说明：PID 参数结构体和通用接口。
 *
 * PID_Incremental 结构保存目标、反馈、误差、积分和输出限幅。
 * 调参时要区分后轮 PID、转向 PID 和其他 PID 实例，不能只看结构体名。
 */

/*
 * PID.h
 *
 * Description: 位置式 PID 控制算法
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct {
    float Target;      // 目标值
    float Current;     // 当前反馈值
    float Error;       // 当前误差
    float LastError;   // 上一次误差
    float Integral;    // 积分累计值
    float Kp, Ki, Kd;  // PID 系数
    float Output;      // 当前输出值
    float MaxOutput;   // 输出限幅，默认 10000
    float IntegralMax; // 积分限幅
} PID_TypeDef;

// 函数声明
float PID_Compute(PID_TypeDef *pid, float target, float current);
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output);
void PID_Reset(PID_TypeDef *pid);

#endif /* CODE_PID_H_ */
