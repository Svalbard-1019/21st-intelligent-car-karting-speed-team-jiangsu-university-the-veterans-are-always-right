/*
 * UTF-8 详细注释说明：Flash 页号和存取接口。
 *
 * 所有 Flash 页号在这里集中定义，避免不同数据写到同一页互相覆盖。
 * 修改页号前要确认旧数据是否需要迁移，否则上电读取会读到旧格式数据。
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
void Flash_Read_gpscheak(void);
void Flash_Write_gpscheak(void);
void Flash_Read_pid(void);
void Flash_Write_pid(void);
void Flash_Write_INSpoints(void);
void Flash_Read_INSpoints(void);
void Flash_Write_passage_points(void);
void Flash_Read_passage_points(void);
void Flash_Write_portion_3points(void);
void Flash_Read_portion_3points(void);
void Flash_Main_Read(void);
void Flash_Store_Mode(uint8 route_setting_choice);
#endif /* CODE_FLASH_H_ */

