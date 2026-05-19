/*
 * control.c
 *
 *  Created on: 2025Äê11ÔÂ21ÈÕ
 *      Author: 18905
 */
#include "zf_common_headfile.h"


_pid MoterPID_L ={ .Kp =0.5 , .Ki = 1.0 , .Kd = 0 ,  .out_max = 3000 , .out_min = -3000};
_pid MoterPID_R ={ .Kp =0.5 , .Ki = 1.0 , .Kd = 0 ,  .out_max = 3000 , .out_min = -3000};
_pid SteerPID ={ .Kp =1.0 , .Ki = 0 , .Kd = 0.5 ,  .out_max = 13.0f , .out_min = -13.0};
_pid SteerUpPID = {.Kp =0.03 , .Ki = 0.08 , .Kd = 0 ,  .out_max = 13.0f , .out_min = -13.0 ,.target_val =0.0f};
_pid Steer_S_Loop = {.Kp =500.0f , .Ki = 0 , .Kd = 0.2 ,  .out_max = 9000.0f , .out_min = -9000.0f ,.target_val =0.0f};
_pid Steer_D_Loop = {.Kp =1000.0f , .Ki = 0.0f , .Kd = 0.1f ,  .out_max = 9000.0f , .out_min = -9000.0f ,.target_val =0.0f , .limit = 100.0f};

MOTER_control_mode conrtol_mode = IDLE;
int num =0;
int num1 =500;
float C;
uint8 Main_Key_Flag = 0;

int16 Steer_Mid_Value = 0;
int angle = 0;
int angle_speed = 0;
int angle_test = 0;
void Steer_Moter_Init(void)
{
    angle_control_init();
    angle_control_set_target(0);
}



void Speed_Control(float tar_l, float tar_r)
{
    static int all_l = 0;
    static int all_r = 0;
    static float Kp =0;

    MoterPID_L.target_val =tar_l;
    MoterPID_R.target_val =tar_r;
    Value_Limit_float(&MoterPID_L.target_val , -200 , 200);
    Value_Limit_float(&MoterPID_R.target_val , -200 , 200);
    Encoder_Get(&Speed_ecd);
    angle_speed += Speed_ecd.delta_r;
    PID_Up(&MoterPID_L , (float)Speed_ecd.delta_l);
    PID_Up(&MoterPID_R , (float)Speed_ecd.delta_r);
    all_l += MoterPID_L.out;
    all_r += MoterPID_R.out;
    Value_Limit_int(&all_l , -MOTER_MAX , MOTER_MAX);
    Value_Limit_int(&all_r , -MOTER_MAX , MOTER_MAX);
    Moter_Set(all_l , all_r );

}



void Steer_Moter_Contral(float servo_out)
{
    Value_Limit_float(&servo_out, ANGLE_MIN_DEGREE, ANGLE_MAX_DEGREE);
    angle_control_set_target((int32)servo_out);
    angle_control_update();

    angle = angle_control_get_current_angle();
    angle_speed = (int)angle_ctrl.pid.Output;


}
void Steer_UpPID(_pid*p ,float error)
{
    static float final_out = 0;
    PID_Up(p, -error);
    final_out += p->out;
    Value_Limit_float(&final_out , -13.0 ,13.0);
    Steer_set(SERVO_MOTOR_MID-final_out);
}
void Steer_Control(int tar)
{
    static int num = 0;
    num = (tar - 500)*(tar - 500)*(tar - 500)/2000000+80;
    Steer_set(num);

}

void Steer_PID(_pid*p ,float error)
{
    static int num = 0;
    PID_Place(p,error);
    num = (int)p->out;
    Steer_set(SERVO_MOTOR_MID-num);

}


