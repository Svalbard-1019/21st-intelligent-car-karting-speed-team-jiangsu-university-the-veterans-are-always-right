/*
 * UTF-8 详细注释说明：整车外设初始化和底层 IO 驱动集中实现。
 *
 * 模块职责：
 * 1. Init_All() 初始化屏幕、按键、蜂鸣器、编码器、电机、IMU、GPS、路线结构。
 * 2. Encoder_Get() 读取后轮编码器，目前左右反馈共用左编码器。
 * 3. Moter_Set()/VeerMoter_Set() 输出旧电机 PWM。
 * 4. Rack_Test_Run() 提供机架测试页面。
 *
 * 硬件注意：
 * - 后轮编码器当前使用 TIM2：P33_7/P33_6。
 * - 蜂鸣器为无源蜂鸣器，Buzzer_check() 会输出约 2kHz 方波。
 * - 如果 Enc 没数据，优先查 P33_7/P33_6 是否被遥控/摄像头/其他外设占用。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 首先调用 Init_All()，Init_All() 集中初始化屏幕、按键、蜂鸣器、编码器、电机、IMU、GPS 和惯导状态。
 * 2. CCU61_CH1 中断周期调用 Key_Scan()、IMU_GetValues()；CCU61_CH0 中断周期调用转向控制、GPS 解析和后轮编码器采样。
 * 3. 科目一记录模式通过 Encoder_Get(&guandao_ecd) 获取后轮里程；当前左右反馈共用左后轮编码器。
 * 4. Rack_Test_Run() 是架上调试入口，用来分别验证前轮转向、后轮速度和 IMU 直线保持，避免一上来就跑完整科目一。
 */


/*
 * peripheral.c
 *
 *  Created on: 2025年11月20日
 *      Author: 18905
 */

#include "zf_common_headfile.h"
#include "rear_motor/rear_motor.h"

uint8 TIM_FLAG1 = 0;
uint8 TIM_FLAG2 = 0;
uint8 TIM_FLAG3 = 0;
Encoder_t Speed_ecd;
Encoder_t guandao_ecd;
Encoder_t Steer_ecd;
#define ENCODER_DELTA_ABS_MAX       (200)
uint8 rack_test_stage = 0;
int16 rack_test_speed_target = 0;
int32 rack_test_steer_target = 0;
float rack_straight_target_yaw = 0.0f;
float rack_straight_yaw_error = 0.0f;
float rack_straight_steer_target = 0.0f;
static float rack_straight_last_yaw_error = 0.0f;

#define RACK_STRAIGHT_KP        (2.0f)
#define RACK_STRAIGHT_KD        (0.45f)
#define RACK_STRAIGHT_LIMIT_DEG (22.0f)
/**
 * 函数说明：Init_All()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Init_All(void)
{

    Display_Init();
    Key_Init();
    Buzzer_Init();
    Steer_init();
    Encoder_Init();
    Motor_init();
    IMU_init();
    GPS_Init();

    Encoder_count_init(&Speed_ecd);
    Encoder_count_init(&guandao_ecd);
    Encoder_count_init(&Steer_ecd);
    guandao_state_init(&INS);
    guandao_state_init(&passage);
    guandao_state_init(&portion_2);
    guandao_state_init(&portion_3);
    guandao_chain_init();
//    gps_work_init();

    KWC_Init(&klm_lat,2,1,0.01,0.0);
    KWC_Init(&klm_lon,2,1,0.01,0.0 );
    Steer_Moter_Init();
}

/**
 * 函数说明：Key_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Key_Init(void)
{
    gpio_init(KEY1, GPI, GPIO_LOW, GPI_PULL_UP);           // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY4 输入 默认高电平 上拉输入

    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH1 输入 默认高电平 浮空输入
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_FLOATING_IN);    // 初始化 SWITCH2 输入 默认高电平 浮空输入

}


// **************************** 变量定义 ****************************
uint8 key1_state = 1;                                                               // 按键动作状态
uint8 key2_state = 1;                                                               // 按键动作状态
uint8 key3_state = 1;                                                               // 按键动作状态
uint8 key4_state = 1;                                                               // 按键动作状态

uint8 switch1_state = 0;                                                            // 拨码开关动作状态
uint8 switch2_state = 0;                                                            // 拨码开关动作状态

uint8 key1_state_last = 0;                                                          // 上一次按键动作状态
uint8 key2_state_last = 0;                                                          // 上一次按键动作状态
uint8 key3_state_last = 0;                                                          // 上一次按键动作状态
uint8 key4_state_last = 0;                                                          // 上一次按键动作状态

uint8 switch1_state_last = 0;                                                       // 上一次拨码开关动作状态
uint8 switch2_state_last = 0;                                                       // 上一次拨码开关动作状态

uint8 key1_flag =0 ;
uint8 key2_flag =0 ;
uint8 key3_flag =0 ;
uint8 key4_flag= 0 ;

uint8 key_val;
uint8 key_value;
/**
 * 函数说明：Key_Scan()。处理按键输入、按键标志或按键触发的参数修改。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Key_Scan(void)
{

    //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费

    //保存按键状态
    key1_state_last = key1_state;
    key2_state_last = key2_state;
    key3_state_last = key3_state;
    key4_state_last = key4_state;

    //读取当前按键状态
    key1_state = gpio_get_level(KEY1);
    key2_state = gpio_get_level(KEY2);
    key3_state = gpio_get_level(KEY3);
    key4_state = gpio_get_level(KEY4);


    //检测到按键按下之后  并放开置位标志位
    if(key1_state && !key1_state_last)   {key1_flag = 1;}
    if(key2_state && !key2_state_last)   {key2_flag = 1;}
    if(key3_state && !key3_state_last)   {key3_flag = 1;}
    if(key4_state && !key4_state_last)   {key4_flag = 1;}


}

/**
 * 函数说明：Buzzer_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Buzzer_Init(void)
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

#define PASSIVE_BUZZER_HALF_PERIOD_US  (250u)   // 2kHz square wave for passive buzzer

/**
 * 函数说明：Buzzer_check()。驱动蜂鸣器发声，用于保存成功、模式切换等提示。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - time2：时间周期或采样间隔，速度计算时会参与单位换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Buzzer_check(int time2)//蜂鸣器的自检函数
{
    uint32 toggle_count = 0;
    uint32 i = 0;

    if(time2 <= 0)
    {
        gpio_set_level(BUZZER_PIN,0);
        return;
    }

    toggle_count = ((uint32)time2 * 1000u) / PASSIVE_BUZZER_HALF_PERIOD_US;
    for(i = 0; i < toggle_count; i++)
    {
        gpio_toggle_level(BUZZER_PIN);
        system_delay_us(PASSIVE_BUZZER_HALF_PERIOD_US);
    }
    gpio_set_level(BUZZER_PIN,0);
}

/**
 * 函数说明：Steer_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_init(void)//舵机初始化
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));

}

/**
 * 函数说明：Steer_set()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - angle：角度或航向相关参数，除特别说明外单位为度。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_set(int angle)//舵机驱动
{
    if(angle<SERVO_MOTOR_LMAX){angle=SERVO_MOTOR_LMAX;}
    if(angle>SERVO_MOTOR_RMAX){angle=SERVO_MOTOR_RMAX;}
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(angle));

}

/**
 * 函数说明：Steer_text()。驱动电机或转向执行器，调用前需要确认方向和限幅。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Steer_text(void)//舵机测试
{

   static int32 angle=SERVO_MOTOR_MID;


      if(key1_flag)
         {
             key1_flag=0;
             angle+=10;
         }
      if(key2_flag)
         {
             key2_flag=0;
             angle-=10;
         }
      if(key3_flag)
         {
             key3_flag=0;
             angle+=1;
//             angle=55;//左打死
         }
      if(key4_flag)
         {
             key4_flag=0;
            angle-=1;
//             angle=85;//右打死
         }
      ips200_show_int(100,  16*3,angle, 5);
      Steer_set(angle);

}

int16 encoder_l = 0;
int16 encoder_r = 0;
int motor_pwm_l = 0;
int motor_pwm_r = 0;


/**
 * 函数说明：Encoder_count_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - count：编码器相关输入或计数值，用于速度、里程或角度换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Encoder_count_init(Encoder_t *count)
{
    count->left_counter =0;
    count->right_counter = 0;
    count->delta_l = 0;
    count->delta_r = 0;
    count->last_ecdcount_l = 0;
    count ->last_ecdcount_r =0;

}
/**
 * 函数说明：Encoder_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Encoder_Init(void)
{
    encoder_quad_init(ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B);

}


/**
 * 函数说明：Encoder_Get()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - count：编码器相关输入或计数值，用于速度、里程或角度换算。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Encoder_Get(Encoder_t *count)
{

    count->left_counter = l_ecdcounter();                  // 获取左编码器计数
    int32 raw_delta = calculate_delta(count->left_counter,count ->last_ecdcount_l);
    if(raw_delta > ENCODER_DELTA_ABS_MAX || raw_delta < -ENCODER_DELTA_ABS_MAX)
    {
        raw_delta = 0;
        count->delta_l = 0;
    }
    else
    {
        count->delta_l = (count->delta_l * 3 + raw_delta) / 4;
    }
    count->right_counter  = count->left_counter;           // 当前只接左编码器，左右后轮共用速度反馈
    count->delta_r = count->delta_l;
//    ips200_show_int(X(1),  Y(8),count->delta_l ,5);
//    ips200_show_int(X(10),  Y(8),count->delta_r ,5);
    count ->last_ecdcount_l = count->left_counter;
    count-> last_ecdcount_r = count->right_counter ;
//    encoder_clear_count(ENCODER_QUADDEC);                                       // 清空编码器计数

}

/**
 * 函数说明：Motor_init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Motor_init(void)//电机初始化
{
       pwm_init(PWM_L, 17000, 0);                                                 // PWM 通道 L1 初始化频率 17KHz 占空比初始为 0
       pwm_init(PWM_R, 17000, 0);                                                // PWM 通道 L2 初始化频率 17KHz 占空比初始为 0
       gpio_init(MOTOR_GPIO_L, GPO, 1, GPO_PUSH_PULL);       //左电机  1正转
       gpio_init(MOTOR_GPIO_R, GPO, 1, GPO_PUSH_PULL);       //右电机  1正转
}
/**
 * 函数说明：VeerMoter_Set()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - moter：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void VeerMoter_Set(int moter )
{
    moter=LimitMax(moter,S_MOTER_MAX);
    angle_motor_set_pwm(moter);

}

/**
 * 函数说明：Moter_Set()。写入上层给定的目标值或执行器输出，并在函数内部做必要限幅。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - moter_l：PWM 或电机输出值，正负号通常表示方向。
 * - moter_r：PWM 或电机输出值，正负号通常表示方向。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Moter_Set(int moter_l , int moter_r)
{
    moter_l =LimitMax(moter_l,MOTER_MAX);
    moter_r = LimitMax(moter_r,MOTER_MAX);
    motor_pwm_l = moter_l;
    motor_pwm_r = moter_r;
    if(moter_l>=0)
    {
        pwm_set_duty(PWM_L, moter_l);
        gpio_set_level(MOTOR_GPIO_L, 1);
    }
    else if(moter_l<0)
    {
        pwm_set_duty(PWM_L, -moter_l);
        gpio_set_level(MOTOR_GPIO_L ,0);
    }
    if(moter_r>=0)
    {
        pwm_set_duty(PWM_R, moter_r);
        gpio_set_level(MOTOR_GPIO_R ,1);
    }
    else if(moter_r<0)
    {
        pwm_set_duty(PWM_R,-moter_r );
        gpio_set_level(MOTOR_GPIO_R ,0);
    }


}
/**
 * 函数说明：LimitMax()。对输入变量进行限幅，防止控制量超过安全范围。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - input：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * - max：限幅边界值，用于保护控制输出或参数范围。
 * 返回值：返回 int 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
int LimitMax(int input, int max)
    {
        if (input > max)
        {
            input = max;
        }
        else if (input < -max)
        {
            input = -max;
        }
        return input;
    }


/**
 * 函数说明：Control()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Control(void)
{
    ips200_show_int(0,  16*3,encoder_l , 5);
    ips200_show_int(50,  16*3,encoder_r, 5);
    ips200_show_int(100,  16*3,num, 5);
    ips200_show_int(150,  16*3,num1, 5);

   Steer_Control(num1);
   if(key1_flag ==1)
   {
       key1_flag = 0;
       num+=30;

   }
   if(key2_flag ==1)
   {
       key2_flag = 0;
       num-=30;

   }
   if(key3_flag ==1)
   {
       key3_flag = 0;
       num1+=100;

   }
   if(key4_flag ==1)
   {
       key4_flag = 0;
       num1-=100;

   }


}
/**
 * 函数说明：Rack_Test_Reset_Targets()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
static void Rack_Test_Reset_Targets(void)
{
    rack_test_speed_target = 0;
    rack_test_steer_target = 0;
    rear_motor_stop();
    if(rack_test_stage == 3)
    {
        Rack_Straight_Reset();
    }
}

/**
 * 函数说明：Rack_Straight_Reset()。清零内部状态和控制输出，用于重新进入测试/自动驾驶前恢复初始状态。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Rack_Straight_Reset(void)
{
    rack_straight_target_yaw = Yaw_1;
    rack_straight_yaw_error = 0.0f;
    rack_straight_last_yaw_error = 0.0f;
    rack_straight_steer_target = 0.0f;
}

/**
 * 函数说明：Rack_Straight_Update()。周期更新内部状态，依赖中断或主循环按固定节拍调用。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Rack_Straight_Update(void)
{
    float yaw_error = rack_straight_target_yaw - Yaw_1;
    float yaw_diff;

    angle_plan(&yaw_error);
    yaw_diff = yaw_error - rack_straight_last_yaw_error;
    angle_plan(&yaw_diff);

    rack_straight_last_yaw_error = yaw_error;
    rack_straight_yaw_error = yaw_error;
    rack_straight_steer_target = -(RACK_STRAIGHT_KP * yaw_error + RACK_STRAIGHT_KD * yaw_diff);
    Value_Limit_float(&rack_straight_steer_target, -RACK_STRAIGHT_LIMIT_DEG, RACK_STRAIGHT_LIMIT_DEG);
}

/**
 * 函数说明：Rack_Test_Run()。完成本模块中的一个独立步骤，具体行为由函数体内的状态变量和硬件调用决定。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Rack_Test_Run(void)
{
    if(key1_flag == 1)
    {
        key1_flag = 0;
        rack_test_stage++;
        if(rack_test_stage > 3) rack_test_stage = 0;
        Rack_Test_Reset_Targets();
    }
    if(key2_flag == 1)
    {
        key2_flag = 0;
        if(rack_test_stage == 0) rack_test_stage = 3;
        else rack_test_stage--;
        Rack_Test_Reset_Targets();
    }
    if(key3_flag == 1)
    {
        key3_flag = 0;
        if(rack_test_stage == 1)
            rack_test_steer_target += 10;
        else if(rack_test_stage == 2 || rack_test_stage == 3)
            rear_motor_set_target_mps(rear_motor_get_target_mps() + 0.5f);
    }
    if(key4_flag == 1)
    {
        key4_flag = 0;
        if(rack_test_stage == 1)
            rack_test_steer_target -= 10;
        else if(rack_test_stage == 2 || rack_test_stage == 3)
            rear_motor_set_target_mps(rear_motor_get_target_mps() - 0.5f);
    }

    if(rack_test_steer_target > 60)  rack_test_steer_target = 60;
    if(rack_test_steer_target < -60) rack_test_steer_target = -60;

    /* 通用显示 */
    ips200_show_string(X(8), Y(0), "Rack_Test");
    ips200_show_string(X(1), Y(2), "Stage");       ips200_show_int(X(10), Y(2), rack_test_stage, 3);

    if(rack_test_stage <= 1)
    {
        /* Stage 0: 传感器显示, Stage 1: 前轮转向测试 */
        ips200_show_string(X(1), Y(3), "Yaw");         ips200_show_float(X(10), Y(3), Yaw_1, 4, 2);
        ips200_show_string(X(1), Y(4), "EncL");        ips200_show_int(X(10), Y(4), Speed_ecd.delta_l, 5);
        ips200_show_string(X(1), Y(5), "EncR");        ips200_show_int(X(10), Y(5), Speed_ecd.delta_r, 5);
        ips200_show_string(X(1), Y(6), "SteerT");      ips200_show_int(X(10), Y(6), rack_test_steer_target, 5);
        ips200_show_string(X(1), Y(7), "SteerA");      ips200_show_int(X(10), Y(7), angle, 5);
        ips200_show_string(X(1), Y(8), "SteerO");      ips200_show_int(X(10), Y(8), angle_speed, 5);
    }
    else if(rack_test_stage == 2)
    {
        /* Stage 2: 后轮速度闭环测试 (m/s) */
        rear_motor_pid_update_100ms();
        ips200_show_string(X(1), Y(3), "TgtMps");     ips200_show_float(X(10), Y(3), rear_motor_get_target_mps(), 3, 2);
        ips200_show_string(X(1), Y(4), "ActMps");     ips200_show_float(X(10), Y(4), rear_motor_get_speed_mps(), 3, 2);
        ips200_show_string(X(1), Y(5), "PWM");        ips200_show_int(X(10), Y(5), rear_motor_get_pwm(), 5);
        ips200_show_string(X(1), Y(6), "Enc10");      ips200_show_int(X(10), Y(6), rear_motor_get_encoder_10ms(), 5);
        ips200_show_string(X(1), Y(7), "Enc100");     ips200_show_int(X(10), Y(7), rear_motor_get_encoder_100ms(), 5);
        ips200_show_string(X(1), Y(8), "Yaw");        ips200_show_float(X(10), Y(8), Yaw_1, 4, 2);
    }
    else
    {
        /* Stage 3: 直线保持测试, 进入本阶段时锁定当前Yaw */
        rear_motor_pid_update_100ms();
        ips200_show_string(X(1), Y(3), "TgtMps");     ips200_show_float(X(10), Y(3), rear_motor_get_target_mps(), 3, 2);
        ips200_show_string(X(1), Y(4), "ActMps");     ips200_show_float(X(10), Y(4), rear_motor_get_speed_mps(), 3, 2);
        ips200_show_string(X(1), Y(5), "TgtYaw");     ips200_show_float(X(10), Y(5), rack_straight_target_yaw, 4, 2);
        ips200_show_string(X(1), Y(6), "YawErr");     ips200_show_float(X(10), Y(6), rack_straight_yaw_error, 3, 2);
        ips200_show_string(X(1), Y(7), "Steer");      ips200_show_float(X(10), Y(7), rack_straight_steer_target, 3, 2);
        ips200_show_string(X(1), Y(8), "PWM");        ips200_show_int(X(10), Y(8), rear_motor_get_pwm(), 5);
    }
}
/**
 * 函数说明：GPS_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：整车外设底层模块，主函数初始化和中断周期任务都依赖这里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void GPS_Init(void)
{
    gnss_init(TAU1201);               // GN42A 为GPS模块 GN43RFA 为RTK模块

}
