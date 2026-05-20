/*
 * peripheral.h
 *
 *  Created on: 2025年11月20日
 *      Author: 18905
 */

#ifndef CODE_PERIPHERAL_H_
#define CODE_PERIPHERAL_H_


//外部变量
extern uint8 TIM_FLAG1;
extern uint8 TIM_FLAG2;
extern uint8 TIM_FLAG3 ;
extern uint8 key1_flag ;
extern uint8 key2_flag ;
extern uint8 key3_flag ;
extern uint8 key4_flag ;
extern uint8 key_val;
extern uint8 key_value;

extern int16 encoder_l ;
extern int16 encoder_r ;
extern int motor_pwm_l;
extern int motor_pwm_r;
extern uint8 rack_test_stage;
extern int16 rack_test_speed_target;
extern int32 rack_test_steer_target;
extern float rack_straight_target_yaw;
extern float rack_straight_yaw_error;
extern float rack_straight_steer_target;

typedef struct{
        int16 left_counter;
        int16 right_counter;
        int32 delta_l;
        int32 delta_r;
        int16 last_ecdcount_l;
        int16 last_ecdcount_r;
}Encoder_t;

extern Encoder_t Speed_ecd;
extern Encoder_t guandao_ecd;
extern Encoder_t Steer_ecd;

//宏定义
#define r_ecdcounter()    encoder_get_count(ENCODER_QUADDEC)
#define l_ecdcounter()    encoder_get_count(ENCODER_QUADDEC)
#define BUZZER_PIN  (P33_10)      //蜂鸣器

#define KEY1                    (P20_6)    //按键引脚配置   //P20_6  //P11_3
#define KEY2                    (P20_7)                               //P20_7 //P11_2
#define KEY3                    (P11_2)                                //P11_3 //P20_7
#define KEY4                    (P11_3)                               //P11_2 //P20_6
#define SWITCH2                 (P33_12)
#define SWITCH1                 (P33_11)

#define SERVO_MOTOR_PWM             (ATOM1_CH1_P33_9)
#define SERVO_MOTOR_FREQ            (50)
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.27+(float)(x)/90.0))
#define SERVO_MOTOR_MID             (80)                                        //中值80
#define SERVO_MOTOR_LMAX            (65)                                        //左打死60
#define SERVO_MOTOR_RMAX            (95)                                       //右打死100

#define ENCODER_QUADDEC                 (TIM2_ENCODER)
#define ENCODER_QUADDEC_A               (TIM2_ENCODER_CH1_P33_7)
#define ENCODER_QUADDEC_B               (TIM2_ENCODER_CH2_P33_6)

#define PWM_L              (ATOM0_CH3_P21_5)
#define PWM_R              (ATOM0_CH1_P21_3)
#define MOTOR_GPIO_L              (P21_4)
#define MOTOR_GPIO_R              (P21_2)
#define MOTER_MAX       (7000)
#define MOTER_MIN       (-7000)
#define S_MOTER_MAX       (5000)
#define S_MOTER_MIN       (-5000)
//函数

void Init_All(void);
void Key_Init(void);   //按键
void Key_Scan(void);
void Buzzer_Init(void);
void Buzzer_check(int time2); //蜂鸣器的自检函数
void Steer_init(void);               //舵机初始化
void Steer_set(int angle);        //舵机驱动
void Steer_text(void);             //舵机测试
void Encoder_Init(void);             //编码器初始化
void Encoder_Get(Encoder_t *count);             //编码器读取
void Motor_init(void);              //电机初始化
void Moter_Set(int moter_l , int moter_r);   //电机驱动
int LimitMax(int input, int max);    //函数内限幅形参
void Control(void);                          //电机&舵机按键调试
void Rack_Test_Run(void);
void Rack_Straight_Reset(void);
void Rack_Straight_Update(void);
void GPS_Init(void);
void Encoder_count_init(Encoder_t *count);
void VeerMoter_Set(int moter );
#endif /* CODE_PERIPHERAL_H_ */
