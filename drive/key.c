/******************** (C) COPYRIGHT 2015 AVER********************
 * 文件名  ：KEY.C
 * 作者    ：李振
 * 描述    ：3*2矩阵键盘扫描
 * 内容    ：扫描完成后出键值
 * 硬件连接：一路3*2矩阵键盘  一路编码器独立按键
 * 修改日期：2015-08-18
********************************************************************/
#include "my_register.h"
#include "beep.h"
#include "flash.h"
#include "gpio.h"
#include "key.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//=================================================================
extern struct bitDefine
{
    unsigned bit0: 1;
    unsigned bit1: 1;
    unsigned bit2: 1;
    unsigned bit3: 1;
    unsigned bit4: 1;
    unsigned bit5: 1;
    unsigned bit6: 1;
    unsigned bit7: 1;
} flagA, flagB,flagC,flagD,flagE,flagF,flagG;
//====================================================================

#define MAX_VOL  3200		
#define MAX_CUR  5100	
vu8 t_KeyScan;
vu32 NewKey;
vu8 t_beep;
vu8 t_lock;
vu8 t_onoff;
vu16 KeyCounter;
vu16 Time_sw;//定时
vu16 Test_Daley;//自动测试判断等待时间，此时间用户可设置
vu16 Test_C_Door;//自动测试门槛电流
/* 定义按键值 */
#define        KEY_ON_OFF                0X01      //
#define        KEY_SET                   0X02      //
#define        KEY_Menu                  0X03      //
#define        KEY_ESC                   0X04      //

/* 定义按键状态值 */
#define        KEY_NULL_VALUE             0X00    //无动作
#define        KEY_PRESS                  0X10    //单击
#define        KEY_LONG                   0X20    //长按
#define        KEY_CONTINUE               0X40    //连击
#define        KEY_UP                     0X80    //按键抬起
/* 定义按键处理步骤 */
#define        KEY_INIT_STATE               0      //初态
#define        KEY_WOBBLE_STATE             1      //消抖
#define        KEY_PRESS_STATE              2      //触发单键
#define        KEY_LONG_STATE               3      //长按触发
#define        KEY_CONTINUE_STATE           4      //连续触发
#define        KEY_RELEASE_STATE            5      //恢复初态
/* 长按时间 */
#define         KEY_LONG_PERIOD             50                /* 长按时间1S */
#define        KEY_CONTINUE_PERIOD          10                /* 双击时间500ms */
/************************************************************************************************************************/
