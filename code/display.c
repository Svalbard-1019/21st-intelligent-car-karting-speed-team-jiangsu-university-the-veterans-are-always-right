/*
 * UTF-8 详细注释说明：IPS200 屏幕菜单和模式选择实现。
 *
 * 模块职责：
 * 1. 读取按键结果，绘制菜单页面。
 * 2. 设置 main_mode、conrtol_mode、route_setting_choice 等全局状态。
 * 3. 进入科目一、记录模式、Rack_Test 等功能前做必要状态初始化。
 *
 * 操作关系：
 * - 记录菜单会设置 Guandao_Recode_Mode，并选择写入 INS/passage/portion_3。
 * - 科目一菜单会设置 Guandao_portion_1，同时调用 portion_1_reset()。
 * - Rack_Test 会进入机架测试，不参与科目一自动驾驶路线追踪。
 *
 * 调试重点：
 * - 屏幕只负责切模式，不直接长期控制电机。
 * - 如果按键进入了错误模式，优先看这里的 key_mode1/key_mode2 变化。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 在 Flash_Main_Read() 之后调用 Menu_Contral()，用户通过屏幕菜单选择 main_mode 和 conrtol_mode。
 * 2. 选择 Guandao_portion_1 时，菜单会设置 main_mode=Guandao_portion_1、conrtol_mode=GUANDAO，并调用 portion_1_reset() 清理上次追踪状态。
 * 3. 选择 Guandao_Recode_Mode 时，主循环进入 guandao_recode(&INS)，推车记录路线点。
 * 4. 选择 Rack_Test_Mode 时，主循环进入 Rack_Test_Run()，用于架上测试转向、后轮速度和直线保持。
 */


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

#define SERIAL_MENU_CMD_NONE      (0u)
#define SERIAL_MENU_CMD_DOWN      (1u)
#define SERIAL_MENU_CMD_UP        (2u)
#define SERIAL_MENU_CMD_ENTER     (3u)
#define SERIAL_MENU_CMD_BACK      (4u)

static uint8 serial_menu_redraw_request = 1;
static uint8 serial_control_edit_flag = 0;

static uint8 Serial_Menu_Read_Key(void);
static void Serial_Menu_Print_Page(void);
static void Serial_Menu_Redraw_If_Needed(void);

static const char *Serial_Menu_Main_Mode_Name(void)
{
    switch(main_mode)
    {
        case Mode_IDLE:             return "Mode_IDLE";
        case Guandao_Recode_Mode:   return "Guandao_Recode_Mode";
        case Guandao_portion_1:     return "Guandao_portion_1";
        case Guandao_Voice:         return "Guandao_Voice";
        case Guandao_portion_3:     return "Guandao_portion_3";
        case Rack_Test_Mode:        return "Rack_Test_Mode";
        default:                    return "Unknown";
    }
}

static const char *Serial_Menu_Control_Mode_Name(void)
{
    switch(conrtol_mode)
    {
        case IDLE:       return "IDLE";
        case YAOKONG:    return "YAOKONG";
        case GUANDAO:    return "GUANDAO";
        case GPS:        return "GPS";
        case DAOCHE:     return "DAOCHE";
        case RACK_TEST:  return "RACK_TEST";
        default:         return "Unknown";
    }
}

static void Serial_Menu_Print_Help(void)
{
    printf("\r\nSerial menu keys:\r\n");
    printf("  w/8: up\r\n");
    printf("  s/2: down\r\n");
    printf("  d/6: enter\r\n");
    printf("  a/4: back\r\n");
    printf("  r: redraw\r\n");
    printf("  h: help\r\n");
}

static void Serial_Menu_Print_Item(uint8 row, const char *text)
{
    printf("%c %s\r\n", (row == key_mode1) ? '>' : ' ', text);
}

static void Serial_Menu_Print_Footer(void)
{
    printf("\r\npage=%u cursor=%u route=%u main=%s ctrl=%s go=%u\r\n",
           key_mode2,
           key_mode1,
           route_setting_choice,
           Serial_Menu_Main_Mode_Name(),
           Serial_Menu_Control_Mode_Name(),
           CarGo_Flag);
    printf("w/s move, d enter, a back, r redraw, h help\r\n");
}

static void Serial_Menu_Print_Page(void)
{
    printf("\r\n\r\n===== TC264 MENU =====\r\n");

    if(key_mode2 == 1)
    {
        printf("[Menu_Main]\r\n");
        Serial_Menu_Print_Item(2, "Car_Go");
        Serial_Menu_Print_Item(3, "Parameter");
        Serial_Menu_Print_Item(4, "Mode_Choice");
        Serial_Menu_Print_Item(5, "Show_Route");
    }
    else if(key_mode2 == 2)
    {
        printf("[Car_Go]\r\n");
        Serial_Menu_Print_Item(2, "Start");
    }
    else if(key_mode2 == 3)
    {
        printf("[Parameter]\r\n");
        Serial_Menu_Print_Item(2, "PID");
        Serial_Menu_Print_Item(3, "Control");
    }
    else if(key_mode2 == 4)
    {
        printf("[Mode_Choice]\r\n");
        Serial_Menu_Print_Item(2, "NULL_Mode_IDLE");
        Serial_Menu_Print_Item(3, "Guandao_Recode_Mode");
        Serial_Menu_Print_Item(4, "Guandao_portion_1");
        Serial_Menu_Print_Item(5, "Voice_Mode");
        Serial_Menu_Print_Item(6, "Guandao_portion_3");
        Serial_Menu_Print_Item(7, "Rack_Test");
    }
    else if(key_mode2 == 5)
    {
        printf("[Recode_Points]\r\n");
        Serial_Menu_Print_Item(2, "INS");
        Serial_Menu_Print_Item(3, "passage");
        Serial_Menu_Print_Item(4, "portion_3");
    }
    else if(key_mode2 == 6)
    {
        printf("[Show_Route]\r\n");
        Serial_Menu_Print_Item(2, "INS");
        Serial_Menu_Print_Item(3, "passage");
        Serial_Menu_Print_Item(4, "portion_3");
    }
    else if(key_mode2 == 7)
    {
        printf("[Route_Display]\r\n");
        printf("Route display is drawn on IPS200. Press a/4 to return.\r\n");
    }
    else if(key_mode2 == 8)
    {
        printf("[PID_P]\r\n");
        Serial_Menu_Print_Item(2, "Speed_kp");
        Serial_Menu_Print_Item(3, "Speed_ki");
        Serial_Menu_Print_Item(4, "Speed_kd");
        Serial_Menu_Print_Item(5, "Recode_The");
        Serial_Menu_Print_Item(6, "Persuit_The");
        Serial_Menu_Print_Item(7, "End_Dec_D");
    }
    else if(key_mode2 == 9)
    {
        printf("[Control_P]\r\n");
        Serial_Menu_Print_Item(2, "Base_Speed");
        Serial_Menu_Print_Item(3, "Daoche_Speed");
        Serial_Menu_Print_Item(4, "Preview_Spets");
        printf("Base=%d Daoche=%d Preview=%d RunMps_x10=%d Edit=%u\r\n",
               control[0], control[1], control[2], control[0], serial_control_edit_flag);
        if(serial_control_edit_flag)
        {
            printf("EDIT: w/8=-1, s/2=+1, d/6=+10, a/4=done\r\n");
        }
        else
        {
            printf("SELECT: w/s move, d enter edit, a back\r\n");
        }
    }
    else
    {
        printf("[Unknown page]\r\n");
    }

    Serial_Menu_Print_Footer();
}

static uint8 Serial_Menu_Read_Key(void)
{
    uint8 buffer[16];
    uint32 len = debug_read_ring_buffer(buffer, sizeof(buffer));
    uint32 i;

    for(i = 0; i < len; i++)
    {
        switch(buffer[i])
        {
            case 'w':
            case 'W':
            case '8':
                return SERIAL_MENU_CMD_UP;
            case 's':
            case 'S':
            case '2':
                return SERIAL_MENU_CMD_DOWN;
            case 'd':
            case 'D':
            case '6':
                return SERIAL_MENU_CMD_ENTER;
            case 'a':
            case 'A':
            case '4':
                return SERIAL_MENU_CMD_BACK;
            case 'r':
            case 'R':
                serial_menu_redraw_request = 1;
                return SERIAL_MENU_CMD_NONE;
            case 'h':
            case 'H':
            case '?':
                Serial_Menu_Print_Help();
                serial_menu_redraw_request = 1;
                return SERIAL_MENU_CMD_NONE;
            default:
                break;
        }
    }

    return SERIAL_MENU_CMD_NONE;
}

static void Serial_Menu_Redraw_If_Needed(void)
{
    static uint8 last_key_mode1 = 0xff;
    static uint8 last_key_mode2 = 0xff;
    static Mode_Choice last_main_mode = Mode_IDLE;
    static MOTER_control_mode last_control_mode = IDLE;
    static uint8 last_route_setting_choice = 0xff;
    static uint8 last_cargo_flag = 0xff;
    static int16 last_control0 = 0x7fff;
    static int16 last_control1 = 0x7fff;
    static int16 last_control2 = 0x7fff;
    static uint8 last_serial_control_edit_flag = 0xff;

    if(serial_menu_redraw_request
            || last_key_mode1 != key_mode1
            || last_key_mode2 != key_mode2
            || last_main_mode != main_mode
            || last_control_mode != conrtol_mode
            || last_route_setting_choice != route_setting_choice
            || last_cargo_flag != CarGo_Flag
            || last_control0 != control[0]
            || last_control1 != control[1]
            || last_control2 != control[2]
            || last_serial_control_edit_flag != serial_control_edit_flag)
    {
        serial_menu_redraw_request = 0;
        last_key_mode1 = key_mode1;
        last_key_mode2 = key_mode2;
        last_main_mode = main_mode;
        last_control_mode = conrtol_mode;
        last_route_setting_choice = route_setting_choice;
        last_cargo_flag = CarGo_Flag;
        last_control0 = control[0];
        last_control1 = control[1];
        last_control2 = control[2];
        last_serial_control_edit_flag = serial_control_edit_flag;
        Serial_Menu_Print_Page();
    }
}

/**
 * 函数说明：Display_Init()。完成模块或硬件资源初始化，通常在系统启动阶段调用一次。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Display_Init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_WHITE , RGB565_BLACK);
    ips200_init(IPS200_TYPE);
}


/*                                                                          菜单界面设计                                                                                  */
/**
 * 函数说明：Menu_Contral()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Menu_Contral(void)
{
    uint8 serial_key_value = 0;

    while(1)
    {
        key_value = Key_Get();
        serial_key_value = Serial_Menu_Read_Key();
        if(serial_key_value != SERIAL_MENU_CMD_NONE)
        {
            key_value = serial_key_value;
        }

        if(key_mode2 == 1)          Menu_Main();
        else if(key_mode2 == 2)  Menu_1();
        else if(key_mode2 == 3)  Menu_Parameter();
        else if(key_mode2 == 4)  Menu_Mode_Choice();
        else if(key_mode2 == 5)  Menu_Recode_Points();
        else if(key_mode2 == 6)  Menu_Show_Route();
        else if(key_mode2 == 7)  Show_Route();
        else if(key_mode2 == 8) Menu_PID_P();
        else if(key_mode2 == 9) Menu_Control_P();

        Serial_Menu_Redraw_If_Needed();

        if(CarGo_Flag == 1){ips200_clear();break;}         //发车指令

    }

}

/**
 * 函数说明：Menu_Main()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：Menu_Show_Route()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
/**
 * 函数说明：Show_Route()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Show_Route(void)
{
    uint8 serial_key_value = 0;

    Guandao_Points_Show(&INS);
    serial_menu_redraw_request = 1;
    Serial_Menu_Redraw_If_Needed();
    while(1){
        key_value = Key_Get();
        serial_key_value = Serial_Menu_Read_Key();
        if(serial_key_value != SERIAL_MENU_CMD_NONE)
        {
            key_value = serial_key_value;
        }
        if(key_value == 4){key_mode2 =6;ips200_clear();serial_menu_redraw_request = 1;break;}
    }

}
/**
 * 函数说明：Menu_Recode_Points()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
        sbus_rc_capture_neutral();
        conrtol_mode = YAOKONG;
        switch(key_mode1)
        {
            case 2:
                route_setting_choice = 0;
                guandao_record_session_reset();
                break;
            case 3:
                route_setting_choice = 1;
                break;
            case 4:
                route_setting_choice = 2;
                break;
            default :break;
        }
        angle_control_select_route(route_setting_choice);
         Buzzer_check(50);
    }
    if(key_value == 4){key_mode2 =4; ips200_clear();}

}
/**
 * 函数说明：Menu_1()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
/**
 * 函数说明：Menu_Mode_Choice()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
        else if (key_mode1 ==4){main_mode = Guandao_portion_1 ;  route_setting_choice = 0; angle_control_select_route(route_setting_choice); daoche_speed = (float)control[1]; portion_1_reset(); conrtol_mode = GUANDAO ; Buzzer_check(50);}
        else if( key_mode1==5){main_mode = Guandao_Voice ; route_setting_choice = 3; conrtol_mode = GUANDAO ;Buzzer_check(50);}
        else if( key_mode1==6){main_mode = Guandao_portion_3 ; route_setting_choice = 2; angle_control_select_route(route_setting_choice); portion3_return_reset(); conrtol_mode = GUANDAO ;Buzzer_check(50);}
        else if( key_mode1==7){main_mode = Rack_Test_Mode ; conrtol_mode = RACK_TEST ; rack_test_stage = 0; rack_test_speed_target = 0; rack_test_steer_target = 0; Rack_Straight_Reset(); MoterPID_L.Kp = 0.5f; MoterPID_R.Kp = 0.5f; MoterPID_L.Ki = 1.0f; MoterPID_R.Ki = 1.0f; MoterPID_L.Kd = 0.0f; MoterPID_R.Kd = 0.0f; CarGo_Flag = 1; ips200_clear(); Buzzer_check(50);}
//        main_mode  = key_mode1 - 2;
//        Buzzer_check(50);
    }
    if(key_value == 4){key_mode2 =1;ips200_clear();}
}

/**
 * 函数说明：Menu_Parameter()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
/**
 * 函数说明：Menu_PID_P()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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
/**
 * 函数说明：Menu_Control_P()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Menu_Control_P(void)
{
    static uint8 edit_flag = 0;
    static uint8 params_dirty = 0;
    int16 *target;
    uint8 value_changed = 0;

    serial_control_edit_flag = edit_flag;

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

        if(key_value == 3)
        {
            edit_flag = 1;
            serial_control_edit_flag = edit_flag;
            serial_menu_redraw_request = 1;
            ips200_clear();
        }
        if(key_value == 4){key_mode2 = 3;ips200_clear();}
    }
    else
    {
        target = &control[key_mode1 - 2];

        if(key_value == 1){*target += 1; value_changed = 1;}
        else if(key_value == 2){*target -= 1; value_changed = 1;}
        else if(key_value == 3){*target += 10; value_changed = 1;}
        else if(key_value == 4)
        {
            if(params_dirty)
            {
                Flash_Write_pid();
                params_dirty = 0;
            }
            edit_flag = 0;
            serial_control_edit_flag = edit_flag;
            serial_menu_redraw_request = 1;
            ips200_clear();
        }

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

        if(value_changed)
        {
            params_dirty = 1;
            base_speed = (float)control[0];
            daoche_speed = (float)control[1];
            preview_spets = control[2];
            serial_menu_redraw_request = 1;
        }
    }

    serial_control_edit_flag = edit_flag;
}

//
//if(key_value == 4){key_mode2 =1;ips200_clear();}
/**
 * 函数说明：Menu_2Value()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Menu_2Value(void)
{
    ips200_show_float(X(17),Y(2),speed_pid[0],3,1);
    ips200_show_float(X(17),Y(3),speed_pid[1],3,1);
    ips200_show_float(X(17),Y(4),speed_pid[2],3,1);
    ips200_show_float(X(17),Y(5),speed_pid[3],3,1);
    ips200_show_float(X(17),Y(6),speed_pid[4],3,1);
    ips200_show_float(X(17),Y(7),speed_pid[5],3,1);
}

/**
 * 函数说明：Menu_Control_Value()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Menu_Control_Value(void)
{
    ips200_show_int(X(16),Y(2),control[0],3);
    ips200_show_int(X(16),Y(3),control[1],3 );
    ips200_show_int(X(16),Y(4),control[2],3 );
    ips200_show_float(X(16),Y(5),(float)control[0] * 0.1f,2,1);
}
/*                                                                                         界面设置                                                                                                                       */
/**
 * 函数说明：Key_Get()。读取当前模块保存的状态量，主要用于屏幕显示和调试。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：返回 uint8 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：Menu_key_Operation_int16()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - param_t：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 int16 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

/**
 * 函数说明：Menu_key_Operation_float()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - param_t：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：返回 float 类型结果，通常用于上层判断状态、显示调试值或继续参与控制计算。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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


/**
 * 函数说明：prompt()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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


/**
 * 函数说明：GPS_prompt()。负责屏幕显示或菜单跳转，不直接改变底层硬件接线。
 * 所属模块：IPS200 菜单模块，决定上电后进入记录、科目一自动驾驶、RackTest 等哪个主模式。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
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

