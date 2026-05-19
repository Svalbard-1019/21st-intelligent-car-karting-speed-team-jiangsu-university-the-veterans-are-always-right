/*
 * peripheral.c
 *
 *  Created on: 2025年11月20日
 *      Author: 18905
 */

#include "zf_common_headfile.h"

uint8 TIM_FLAG1 = 0;
uint8 TIM_FLAG2 = 0;
uint8 TIM_FLAG3 = 0;
Encoder_t Speed_ecd;
Encoder_t guandao_ecd;
Encoder_t Steer_ecd;
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

void Buzzer_Init(void)
{
    gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);
}

void Buzzer_check(int time2)//蜂鸣器的自检函数
{
    gpio_set_level(BUZZER_PIN,1);
    system_delay_ms(time2);
    gpio_set_level(BUZZER_PIN,0);
}

void Steer_init(void)//舵机初始化
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, (uint32)SERVO_MOTOR_DUTY(SERVO_MOTOR_MID));

}

void Steer_set(int angle)//舵机驱动
{
    if(angle<SERVO_MOTOR_LMAX){angle=SERVO_MOTOR_LMAX;}
    if(angle>SERVO_MOTOR_RMAX){angle=SERVO_MOTOR_RMAX;}
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(angle));

}

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



void Encoder_count_init(Encoder_t *count)
{
    count->left_counter =0;
    count->right_counter = 0;
    count->delta_l = 0;
    count->delta_r = 0;
    count->last_ecdcount_l = 0;
    count ->last_ecdcount_r =0;

}
void Encoder_Init(void)
{
    encoder_dir_init(ENCODER_QUADDEC, ENCODER_QUADDEC_A, ENCODER_QUADDEC_B);
    encoder_dir_init(ENCODER_DIR, ENCODER_DIR_PULSE, ENCODER_DIR_DIR);

}


void Encoder_Get(Encoder_t *count)
{

    count->right_counter  = -r_ecdcounter();                // 获取编码器计数
    count->left_counter = l_ecdcounter();                  // 获取编码器计数
    count->delta_r = calculate_delta(count->right_counter,count-> last_ecdcount_r);
    count->delta_l = calculate_delta(count->left_counter,count ->last_ecdcount_l);
//    ips200_show_int(X(1),  Y(8),count->delta_l ,5);
//    ips200_show_int(X(10),  Y(8),count->delta_r ,5);
    count ->last_ecdcount_l = count->left_counter;
    count-> last_ecdcount_r = count->right_counter ;
//    encoder_clear_count(ENCODER_QUADDEC);                                       // 清空编码器计数
//    encoder_clear_count(ENCODER_DIR);                                           // 清空编码器计数

}

void Motor_init(void)//电机初始化
{
       pwm_init(PWM_L, 17000, 0);                                                 // PWM 通道 L1 初始化频率 17KHz 占空比初始为 0
       pwm_init(PWM_R, 17000, 0);                                                // PWM 通道 L2 初始化频率 17KHz 占空比初始为 0
       pwm_init(PWM_V, 17000, 0);                                                // PWM 通道 L2 初始化频率 17KHz 占空比初始为 0
       gpio_init(MOTOR_GPIO_V, GPO, 1, GPO_PUSH_PULL);       //左电机  1正转
       gpio_init(MOTOR_GPIO_L, GPO, 1, GPO_PUSH_PULL);       //左电机  1正转
       gpio_init(MOTOR_GPIO_R, GPO, 1, GPO_PUSH_PULL);       //右电机  1正转
}
void VeerMoter_Set(int moter )
{
//    moter = -moter;
    moter=LimitMax(moter,S_MOTER_MAX);

    if(moter>=0)
    {
        pwm_set_duty(PWM_V, moter);
        gpio_set_level(MOTOR_GPIO_V, 1);
    }
    else if(moter<0)
    {
        pwm_set_duty(PWM_V, -moter);
        gpio_set_level(MOTOR_GPIO_V ,0);
    }





}

void Moter_Set(int moter_l , int moter_r)
{
    moter_l = -moter_l;
    moter_r = -moter_r;
    moter_l =LimitMax(moter_l,MOTER_MAX);
    moter_r = LimitMax(moter_r,MOTER_MAX);
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
void GPS_Init(void)
{
    gnss_init(TAU1201);               // GN42A 为GPS模块 GN43RFA 为RTK模块

}
