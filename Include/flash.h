/******************** (C) COPYRIGHT 2015 AVER************************
 * 文件名  ：
 * 作者    ：李振
 * 描述    ：头文件中只能有声明不可定义
 * 内容    ：
 * 修改日期：2015-8-18
*********************************************************************/
#include "my_register.h"
#ifndef _flash_h_
#define _flash_h_
/*********************函数声明*************************************/
void Init_EEPROM(void);
void EEPROM_Write_8(vu8 addr, vu8 data);
vu8 EEPROM_READ_Byte(vu8 addr_eep);
void Flash_Write_all (void);
void EEPROM_READ_Coeff(void);
void Wite_Runcont(void);//将运行参数写入EEPROM
void Read_Runcont(void);//读取运行参数
void READ_TSET(void);//
/******************************************************************/
#endif
/******************* (C) COPYRIGHT 2015 AVER *****END OF FILE***************************/
