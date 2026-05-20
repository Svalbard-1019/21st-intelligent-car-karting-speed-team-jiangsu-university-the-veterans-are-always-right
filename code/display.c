/*
 * display.c
 *
 *  Created on: 2025年11月20日
 *      Author: 18905
 */
#include "zf_common_headfile.h"
uint8 key_mode1 = 1;
uint8 key_mode2 = 2;
uint8 CarGo_Flag = 0;
float *p;
int16 *p1;
Mode_Choice main_mode = Mode_IDLE;

void Display_Init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE , RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}



/*                                                                          菜单界面设计                                                                                  */
void Menu_Contral(void)
{
    while(1)
    {


        key_value = Key_Get();                                          //按键采集
        if(key_mode2 == 1)          Menu_Main();              //界面选择配置
        else if(key_mode2 == 2)  Menu_1();
        else if(key_mode2 == 3)  Menu_Parameter();
        else if(key_mode2 == 4)  Menu_Mode_Choice();
        else if(key_mode2 == 5)  Menu_Recode_Points();
        else if(key_mode2 == 6)  Menu_Show_Route();
        else if(key_mode2 == 7)  Show_Route();
        else if(key_mode2 == 8) Menu_PID_P();
        else if(key_mode2 == 9) Menu_Control_P();

        if(CarGo_Flag == 1){ips200_clear();break;}         //发车指令

    }

}

void Menu_Main(void)
{
    ips200_show_string( X(10) ,Y(0) ,"Menu_Main");
    ips200_show_string( X(3) ,Y(2) ,"Car_Go");
    ips200_show_string( X(3) ,Y(3) ,"Parameter");
    ips200_show_string( X(3) ,Y(4) ,"Mode_Choice");
    ips200_show_string( X(3) ,Y(5) ,"Show_Route");

    prompt();                                                                                  //提示标识
    if(key_value == 1)key_mode1 ++;                                          //按键控制+限幅
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 5) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 5  : key_mode1;

    if(key_value == 3)                                                                  //确定键执行
    {
        ips200_clear();
        if(key_mode1 == 5 ){ key_mode2 = 6; }
        else{  key_mode2 = key_mode1;}

    }
}

void Menu_Show_Route(void)
{
    ips200_show_string( X(10) ,Y(0) ,"Show_Route");
    ips200_show_string( X(3) ,Y(2) ,"INS");
    ips200_show_string( X(3) ,Y(3) ,"passage");
    ips200_show_string( X(3) ,Y(4) ,"portion_3");

    prompt();                                                                                  //提示标识
    if(key_value == 1)key_mode1 ++;                                          //按键控制+限幅
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 4) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 4  : key_mode1;

    if(key_value == 3){
        switch(key_mode1)
        {
            case 2:
                route_setting_choice = 0;
                break;
            case 3:
                route_setting_choice = 1;
                break;
            case 4:
                route_setting_choice = 2;
                break;
                break;
            default :break;
        }
        key_mode2 = 7; ips200_clear();
    }
    if(key_value == 4){key_mode2 =1; ips200_clear();}


}
void Show_Route(void)
{

    Guandao_Points_Show(&INS);
    while(1){
        key_value = Key_Get();   //按键采集
        if(key_value == 4){key_mode2 =6;ips200_clear();break;}
    }

}
void Menu_Recode_Points(void)
{
    ips200_show_string( X(7) ,Y(0) ,"Recode_Points");
    ips200_show_string( X(3) ,Y(2) ,"INS");
    ips200_show_string( X(3) ,Y(3) ,"passage");
    ips200_show_string( X(3) ,Y(4) ,"portion_3");

    prompt();                                                                                  //提示标识
    if(key_value == 1)key_mode1 ++;                                          //按键控制+限幅
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 4) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 4  : key_mode1;

    if(key_value == 3)
    {
        main_mode = Guandao_Recode_Mode;
        conrtol_mode = YAOKONG;
        switch(key_mode1)
        {
            case 2:
                route_setting_choice = 0;
                break;
            case 3:
                route_setting_choice = 1;
                break;
            case 4:
                route_setting_choice = 2;
                break;
            default :break;
        }
         Buzzer_check(50);
    }
    if(key_value == 4){key_mode2 =4; ips200_clear();}

}
void Menu_1(void)
{
    ips200_show_string( X(10) ,Y(0) ,"Car_Go");
    ips200_show_string( X(3) ,Y(2) ,"Start");
//    if(key_value == 1)key_mode1 ++;
//    else if(key_value == 2)key_mode1 --;
//    key_mode1 =(key_mode1 > 4) ? 2  : key_mode1;
//    key_mode1 =(key_mode1 < 2) ? 4  : key_mode1;

    key_mode1 =2;
    prompt();
    if(key_value == 3&& key_mode1 ==2){CarGo_Flag =1;}

    if(key_value == 4){key_mode2 =1;ips200_clear();}

}
void Menu_Mode_Choice(void)
{
    ips200_show_string( X(10) ,Y(0) ,"Mode");

    ips200_show_string( X(3) ,Y(2) ,"NULL_Mode_IDLE");
    ips200_show_string( X(3) ,Y(3) ,"Guandao_Recode_Mode");
    ips200_show_string( X(3) ,Y(4) ,"Guandao_portion_1");
    ips200_show_string( X(3) ,Y(5) ,"Voice_Mode");
    ips200_show_string( X(3) ,Y(6) ,"Guandao_portion_3");
    ips200_show_string( X(3) ,Y(7) ,"Rack_Test");

    prompt();

    if(key_value == 1)key_mode1 ++;
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 7) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 7  : key_mode1;

    if(key_value == 3)
    {
        if(key_mode1 == 3){key_mode2 =5 ; ips200_clear(); }
        else if (key_mode1 ==4){main_mode = Guandao_portion_1 ;  route_setting_choice = 0; daoche_speed = (float)control[1]; portion_1_reset(); conrtol_mode = GUANDAO ; Buzzer_check(50);}
        else if( key_mode1==5){main_mode = Guandao_Voice ; route_setting_choice = 3; conrtol_mode = GUANDAO ;Buzzer_check(50);}
        else if( key_mode1==6){main_mode = Guandao_portion_3 ; route_setting_choice = 2; conrtol_mode = GUANDAO ;Buzzer_check(50);}
        else if( key_mode1==7){main_mode = Rack_Test_Mode ; conrtol_mode = RACK_TEST ; rack_test_stage = 0; rack_test_speed_target = 0; rack_test_steer_target = 0; Rack_Straight_Reset(); MoterPID_L.Kp = 0.5f; MoterPID_R.Kp = 0.5f; MoterPID_L.Ki = 1.0f; MoterPID_R.Ki = 1.0f; MoterPID_L.Kd = 0.0f; MoterPID_R.Kd = 0.0f; CarGo_Flag = 1; ips200_clear(); Buzzer_check(50);}
//        main_mode  = key_mode1 - 2;
//        Buzzer_check(50);
    }
    if(key_value == 4){key_mode2 =1;ips200_clear();}
}

void Menu_Parameter(void)
{
    ips200_show_string( X(10) ,Y(0) ,"Parameter");
    ips200_show_string( X(3) ,Y(2) ,"PID");
    ips200_show_string( X(3) ,Y(3) ,"Control");



    prompt();

    if(key_value == 1)key_mode1 ++;
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 3) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 3  : key_mode1;

    if(key_value==3)
    {
        if(key_mode1 ==2){key_mode2 = 8;ips200_clear();}
        else if(key_mode1 ==3){key_mode2 = 9;ips200_clear();}

    }
    if(key_value == 4){key_mode2 =1;ips200_clear();}

}
void Menu_PID_P(void)
{
    ips200_show_string( X(10) ,Y(0) ,"PID_P");
    ips200_show_string( X(3) ,Y(2) ,"Speed_kp");
    ips200_show_string( X(3) ,Y(3) ,"Speed_ki");
    ips200_show_string( X(3) ,Y(4) ,"Speed_kd");
    ips200_show_string( X(3) ,Y(5) ,"Recode_The");
    ips200_show_string( X(3) ,Y(6) ,"Persuit_The");
    ips200_show_string( X(3) ,Y(7) ,"End_Dec_D");

    Menu_2Value();
    prompt();

    if(key_value == 1)key_mode1 ++;
    else if(key_value == 2)key_mode1 --;
    key_mode1 =(key_mode1 > 7) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 7  : key_mode1;

    while (gpio_get_level(SWITCH1))
    {

        Menu_2Value();
        Key_Scan();

        for(uint8 i = 0 ; i < 7; i++ )
        {
            if(i == key_mode1-2)
            {
                p =&speed_pid[i];
                Menu_key_Operation_float(p);
            }
        }
    }


    if(key_value == 4){key_mode2 =3 ;ips200_clear();}
}
void Menu_Control_P(void)
{
    static uint8 edit_flag = 0;
    int16 *target;

    ips200_show_string( X(10) ,Y(0) ,"Control_P");
    ips200_show_string( X(3) ,Y(2) ,"Base_Speed");
    ips200_show_string( X(3) ,Y(3) ,"Daoche_Speed");
    ips200_show_string( X(3) ,Y(4) ,"Preview_Spets");
    ips200_show_string( X(3) ,Y(5) ,"Run_Mps");

    key_mode1 =(key_mode1 > 4) ? 2  : key_mode1;
    key_mode1 =(key_mode1 < 2) ? 4  : key_mode1;

    Menu_Control_Value();
    prompt();
    if(edit_flag) ips200_show_string(X(1), Y(7), "EDIT");

    if(edit_flag == 0)
    {
        if(key_value == 1)key_mode1 ++;
        else if(key_value == 2)key_mode1 --;
        key_mode1 =(key_mode1 > 4) ? 2  : key_mode1;
        key_mode1 =(key_mode1 < 2) ? 4  : key_mode1;

        if(key_value == 3){edit_flag = 1; ips200_clear();}
        if(key_value == 4){key_mode2 = 3;ips200_clear();}
    }
    else
    {
        target = &control[key_mode1 - 2];

        if(key_value == 1)*target += 1;
        else if(key_value == 2)*target -= 1;
        else if(key_value == 3)*target += 10;
        else if(key_value == 4){edit_flag = 0; ips200_clear();}

        if(key_mode1 == 2)
        {
            if(control[0] < 0) control[0] = 0;
            if(control[0] > 50) control[0] = 50;
        }
        else if(key_mode1 == 3)
        {
            if(control[1] > 0) control[1] = 0;
            if(control[1] < -50) control[1] = -50;
        }
        else if(key_mode1 == 4)
        {
            if(control[2] < 1) control[2] = 1;
            if(control[2] > 20) control[2] = 20;
        }
    }
}

//
//if(key_value == 4){key_mode2 =1;ips200_clear();}
void Menu_2Value(void)
{
    ips200_show_float(X(17),Y(2),speed_pid[0],3,1);
    ips200_show_float(X(17),Y(3),speed_pid[1],3,1);
    ips200_show_float(X(17),Y(4),speed_pid[2],3,1);
    ips200_show_float(X(17),Y(5),speed_pid[3],3,1);
    ips200_show_float(X(17),Y(6),speed_pid[4],3,1);
    ips200_show_float(X(17),Y(7),speed_pid[5],3,1);
}

void Menu_Control_Value(void)
{
    ips200_show_int(X(16),Y(2),control[0],3);
    ips200_show_int(X(16),Y(3),control[1],3 );
    ips200_show_int(X(16),Y(4),control[2],3 );
    ips200_show_float(X(16),Y(5),(float)control[0] * 0.1f,2,1);
}
/*                                                                                         界面设置                                                                                                                       */
uint8 Key_Get(void)
{
    uint8 value = 0 ;
    Key_Scan();
    if(key4_flag == 1)
    {
        key4_flag = 0;
        value = 2;
    }
    if(key3_flag == 1)
    {
        key3_flag = 0 ;
       value = 1;
    }
    if(key2_flag == 1)
    {
        key2_flag = 0;
        value = 3;
    }
    if(key1_flag == 1)
    {
        key1_flag =0;
        value = 4;
    }
    return value;
}

int16 Menu_key_Operation_int16(int16 *param_t )//按键调节界面操作函数//指针变量作为形参，那函数内的形参也要为指针形式
{

     if(key1_flag){key1_flag=0;key_val=1;}
     if(key2_flag){key2_flag=0;key_val=2;}
     if(key3_flag){key3_flag=0;key_val=3;}
     if(key4_flag){key4_flag=0;key_val=4;}

//                    switch(key_val)
//                    {
//
//                        case 1:*param_t+=10 ,key_val=0; break;
//                        case 2:*param_t-=10 ,key_val=0; break;
//                        case 3:*param_t+=1  ,key_val=0; break;
//                        case 4:*param_t-=1  ,key_val=0; break;
//                        default:break;
//                    }
     if(key_val ==1){*param_t+=10 ;key_val=0;}
     if(key_val ==2){*param_t-=10 ;key_val=0;}
     if(key_val ==3){*param_t+=1 ;key_val=0;}
     if(key_val ==4){*param_t-=1 ;key_val=0;}

   return  *param_t;
}

float Menu_key_Operation_float(float *param_t )
{
    if(key1_flag){key1_flag=0;key_val=1;}
    if(key2_flag){key2_flag=0;key_val=2;}
    if(key3_flag){key3_flag=0;key_val=3;}
    if(key4_flag){key4_flag=0;key_val=4;}

//                    switch(key_val)
//                    {
//
//                        case 1:*param_t+=10 ,key_val=0; break;
//                        case 2:*param_t-=10 ,key_val=0; break;
//                        case 3:*param_t+=1  ,key_val=0; break;
//                        case 4:*param_t-=1  ,key_val=0; break;
//                        default:break;
//                    }
    if(key_val ==1){*param_t+=0.5 ;key_val=0;}
    if(key_val ==2){*param_t-=0.5 ;key_val=0;}
    if(key_val ==3){*param_t+=0.1 ;key_val=0;}
    if(key_val ==4){*param_t-=0.1 ;key_val=0;}

  return  *param_t;

}


void prompt(void)
{
    static uint8 i ;
    for( i = 0 ; i<10; i++)
    {
        if( i == key_mode1 )
        {
            ips200_show_string( 0 ,16*i ,"->");
        }
        else
        {
            ips200_show_string( 0 ,16*i ,"  ");
        }
    }
}


void GPS_prompt(void)
{
    static uint8 i ;
    for( i = 2 ; i<4; i++)
    {
        if( i == key_mode1 )
        {
            ips200_show_string( 0 ,16*i ,"->");
        }
        else
        {
            ips200_show_string( 0 ,16*i ,"  ");
        }
    }
    ips200_show_string( X(0) ,Y(4) ,"[1]");
    ips200_show_string( X(0) ,Y(5) ,"[2]");
    ips200_show_string( X(0) ,Y(6) ,"[3]");
    ips200_show_string( X(0) ,Y(7) ,"[4]");
    ips200_show_string( X(0) ,Y(8) ,"[5]");
    ips200_show_string( X(0) ,Y(9) ,"[6]");
    ips200_show_string( X(0) ,Y(10) ,"[7]");
    ips200_show_string( X(0) ,Y(11) ,"[8]");
    ips200_show_string( X(0) ,Y(12) ,"[9]");
    ips200_show_string( X(0) ,Y(13) ,"[10]");
    ips200_show_string( X(0) ,Y(14) ,"[11]");
    ips200_show_string( X(0) ,Y(15) ,"[12]");
    ips200_show_string( X(0) ,Y(16) ,"[13]");
    ips200_show_string( X(0) ,Y(17) ,"[14]");
    ips200_show_string( X(0) ,Y(18) ,"[15]");
}

