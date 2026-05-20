/*
 * UTF-8 详细注释说明：Flash 页号和存取接口。
 *
 * 所有 Flash 页号在这里集中定义，避免不同数据写到同一页互相覆盖。
 * 修改页号前要确认旧数据是否需要迁移，否则上电读取会读到旧格式数据。
 */

/*
 * 主函数/科目一调用链：
 * 1. core0_main() 上电初始化后调用 Flash_Main_Read()，恢复 PID 参数、INS 路线、passage 路线、portion_3 路线和 GPS 校验点。
 * 2. 记录模式 guandao_recode() 里长按 KEY1 会触发 Flash_Store_Mode(route_setting_choice)，把当前选择的路线写入对应 Flash 页。
 * 3. 科目一自动驾驶 portion_1() 使用的 INS.length_index 和 INS.map[] 来自 Flash_Read_INSpoints() 恢复的数据。
 * 4. 如果重新烧录但没有擦除对应 Flash 页，路线点通常还在；如果换工程或擦 Flash，则需要重新记录。
 */


/*
 * flash.h
 *
 *  Created on: 2025年11月23日
 *      Author: 18905
 */

#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

//外部变量

extern float speed_pid[6];
extern int16 control[5];
extern float kp ;
extern float ki ;
extern float kd ;

//宏定义
#define FLASH_SECTION_INDEX             (0)    //存储数据用的和扇区
#define SPEED_PID_PAGE_INDEX            (11)   //储存页
#define RECODE_MAP_POINTS_INDEX   (10)   //记录地图点位
#define RECODE_PASSAGE                  (9)   //记录地图点位
#define RECODE_PASSAGE_TWO                  (8)   //记录地图点位
#define RECODE_PASSAGE_THREE                  (7)   //记录地图点位
#define RECODE_PASSAGE_FOUR                  (6)   //记录地图点位
#define RECODE_PASSAGE_FIF                  (5)   //记录地图点位
#define RECODE_PORTION_THREE                  (4)   //记录地图点位
#define GPS_CHEAK_FLAG                            (3)   //记录地图点位
//函数
/**
 * 接口说明：Flash_Read_gpscheak()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_gpscheak(void);
/**
 * 接口说明：Flash_Write_gpscheak()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_gpscheak(void);
/**
 * 接口说明：Flash_Read_pid()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_pid(void);
/**
 * 接口说明：Flash_Write_pid()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_pid(void);
/**
 * 接口说明：Flash_Write_INSpoints()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_INSpoints(void);
/**
 * 接口说明：Flash_Read_INSpoints()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_INSpoints(void);
/**
 * 接口说明：Flash_Write_passage_points()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_passage_points(void);
/**
 * 接口说明：Flash_Read_passage_points()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_passage_points(void);
/**
 * 接口说明：Flash_Write_portion_3points()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Write_portion_3points(void);
/**
 * 接口说明：Flash_Read_portion_3points()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Read_portion_3points(void);
/**
 * 接口说明：Flash_Main_Read()。从 Flash、传感器或缓存中读取数据，并同步到全局运行变量。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - 无：该函数不需要外部输入参数。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Main_Read(void);
/**
 * 接口说明：Flash_Store_Mode()。把当前运行参数或路线点写入 Flash，掉电后仍可恢复。
 * 所属模块：Flash 参数和路线持久化模块，负责把调试好的路线/参数保存到板子里。
 * 参数说明：
 * - route_setting_choice：输入/输出参数，具体含义需要结合函数名和调用位置理解。
 * 返回值：无返回值；结果通过全局变量、结构体字段或硬件输出体现。
 * 科目一关系：如果该函数处在科目一链路中，通常由 core0_main() 主循环、CCU61_CH0/CH1 中断或 Menu_Contral() 间接触发。
 * 注意事项：调用前确认相关全局状态和硬件初始化已经完成，避免在中断和主循环中重复抢占同一硬件资源。
 */
void Flash_Store_Mode(uint8 route_setting_choice);
#endif /* CODE_FLASH_H_ */

