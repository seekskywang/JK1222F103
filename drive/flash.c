/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * 文件名  ：FLASH.C
 * 作者    ：李振
 * 描述    ：STM8L151K4T6 FLASH应用函数
 * 内容    ：
 * 硬件连接：
 * 修改日期：2014-12-22
********************************************************************/
#include "my_register.h" //NOP指令在这
#include "AT24C01.h"
#include "flash.h"
//================================================================//
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
} flagA, flagB,flagC,flagD;
CalUni Calibrate;
#define   MAX_Correct_Addr         0X66  //定义校准参数存储的最后一个地址
vu32 REND_TSETING;
//==================================================================//
void Flash_Write_all (void)
{
//	WriteBytes(Calibrate.CalFlash,0,sizeof(CalPara));
	
	vu8 data_8bit;
/***********************测量数据存储****************************/
  data_8bit = REG_ReadV_Offset_LOW >> 24;
	EEPROM_WriteByte(0x01, data_8bit);
	data_8bit = REG_ReadV_Offset_LOW >> 16;
	EEPROM_WriteByte(0x02, data_8bit);
	data_8bit = REG_ReadV_Offset_LOW >> 8;
	EEPROM_WriteByte(0x03, data_8bit);
	EEPROM_WriteByte(0x04, REG_ReadV_Offset_LOW);
	
	data_8bit = REG_ReadV_Offset_HIG >> 24;
	EEPROM_WriteByte(0x05, data_8bit);
	data_8bit = REG_ReadV_Offset_HIG >> 16;
	EEPROM_WriteByte(0x06, data_8bit);
	data_8bit = REG_ReadV_Offset_HIG >> 8;
	EEPROM_WriteByte(0x07, data_8bit);
	EEPROM_WriteByte(0x08, REG_ReadV_Offset_HIG);
	
	data_8bit = REG_ReadA_Offset_LOW >> 24;
	EEPROM_WriteByte(0x09, data_8bit);
	data_8bit = REG_ReadA_Offset_LOW >> 16;
	EEPROM_WriteByte(0x0A, data_8bit);
	data_8bit = REG_ReadA_Offset_LOW >> 8;
	EEPROM_WriteByte(0x0B, data_8bit);
	EEPROM_WriteByte(0x0C, REG_ReadA_Offset_LOW);
	IWDG_ReloadCounter(); //喂狗
	data_8bit = REG_ReadA_Offset_HIG >> 24;
	EEPROM_WriteByte(0x0D, data_8bit);
	data_8bit = REG_ReadA_Offset_HIG >> 16;
	EEPROM_WriteByte(0x0E, data_8bit);
	data_8bit = REG_ReadA_Offset_HIG >> 8;
	EEPROM_WriteByte(0x0F, data_8bit);
	EEPROM_WriteByte(0x10, REG_ReadA_Offset_HIG);
	
	data_8bit=REG_CorrectionV_LOW>>24;
	EEPROM_WriteByte(0x11, data_8bit);
	data_8bit=REG_CorrectionV_LOW>>16;
	EEPROM_WriteByte(0x12, data_8bit);
	data_8bit=REG_CorrectionV_LOW>>8;
	EEPROM_WriteByte(0x13, data_8bit);
	EEPROM_WriteByte(0x14, REG_CorrectionV_LOW);
	
	data_8bit=REG_CorrectionV_HIG>>24;
	EEPROM_WriteByte(0x15, data_8bit);
	data_8bit=REG_CorrectionV_HIG>>16;
	EEPROM_WriteByte(0x16, data_8bit);
	data_8bit=REG_CorrectionV_HIG>>8;
	EEPROM_WriteByte(0x17, data_8bit);
	EEPROM_WriteByte(0x18, REG_CorrectionV_HIG);
	
	data_8bit=REG_CorrectionA_LOW>>24;
	EEPROM_WriteByte(0x19, data_8bit);
	data_8bit=REG_CorrectionA_LOW>>16;
	EEPROM_WriteByte(0x1A, data_8bit);
	data_8bit=REG_CorrectionA_LOW>>8;
	EEPROM_WriteByte(0x1B, data_8bit);
	EEPROM_WriteByte(0x1C, REG_CorrectionA_LOW);
	
	data_8bit=REG_CorrectionA_HIG>>24;
	EEPROM_WriteByte(0x1D, data_8bit);
	data_8bit=REG_CorrectionA_HIG>>16;
	EEPROM_WriteByte(0x1E, data_8bit);
	data_8bit=REG_CorrectionA_HIG>>8;
	EEPROM_WriteByte(0x1F, data_8bit);
	EEPROM_WriteByte(0x20, REG_CorrectionA_HIG);
	
	EEPROM_WriteByte(0x21,Polar0);
	IWDG_ReloadCounter(); //喂狗
/***********************设置数据存储****************************/
	data_8bit = SET_ReadV_Offset_LOW >> 24;
	EEPROM_WriteByte(0x22, data_8bit);
	data_8bit = SET_ReadV_Offset_LOW >> 16;
	EEPROM_WriteByte(0x23, data_8bit);
	data_8bit = SET_ReadV_Offset_LOW >> 8;
	EEPROM_WriteByte(0x24, data_8bit);
	EEPROM_WriteByte(0x25, SET_ReadV_Offset_LOW);
	
	data_8bit = SET_ReadV_Offset_HIG >> 24;
	EEPROM_WriteByte(0x26, data_8bit);
	data_8bit = SET_ReadV_Offset_HIG >> 16;
	EEPROM_WriteByte(0x27, data_8bit);
	data_8bit = SET_ReadV_Offset_HIG >> 8;
	EEPROM_WriteByte(0x28, data_8bit);
	EEPROM_WriteByte(0x29, SET_ReadV_Offset_HIG);
	
	data_8bit = SET_ReadA_Offset_LOW >> 24;
	EEPROM_WriteByte(0x2A, data_8bit);
	data_8bit = SET_ReadA_Offset_LOW >> 16;
	EEPROM_WriteByte(0x2B, data_8bit);
	data_8bit = SET_ReadA_Offset_LOW >> 8;
	EEPROM_WriteByte(0x2C, data_8bit);
	EEPROM_WriteByte(0x2D, SET_ReadA_Offset_LOW);
	IWDG_ReloadCounter(); //喂狗
	data_8bit = SET_ReadA_Offset_HIG >> 24;
	EEPROM_WriteByte(0x2E, data_8bit);
	data_8bit = SET_ReadA_Offset_HIG >> 16;
	EEPROM_WriteByte(0x2F, data_8bit);
	data_8bit = SET_ReadA_Offset_HIG >> 8;
	EEPROM_WriteByte(0x30, data_8bit);
	EEPROM_WriteByte(0x31, SET_ReadA_Offset_HIG);
	
	data_8bit=SET_CorrectionV_LOW>>24;
	EEPROM_WriteByte(0x32, data_8bit);
	data_8bit=SET_CorrectionV_LOW>>16;
	EEPROM_WriteByte(0x33, data_8bit);
	data_8bit=SET_CorrectionV_LOW>>8;
	EEPROM_WriteByte(0x34, data_8bit);
	EEPROM_WriteByte(0x35, SET_CorrectionV_LOW);
	
	data_8bit=SET_CorrectionV_HIG>>24;
	EEPROM_WriteByte(0x36, data_8bit);
	data_8bit=SET_CorrectionV_HIG>>16;
	EEPROM_WriteByte(0x37, data_8bit);
	data_8bit=SET_CorrectionV_HIG>>8;
	EEPROM_WriteByte(0x38, data_8bit);
	EEPROM_WriteByte(0x39, SET_CorrectionV_HIG);
	IWDG_ReloadCounter(); //喂狗
	data_8bit=SET_CorrectionA_LOW>>24;
	EEPROM_WriteByte(0x3A, data_8bit);
	data_8bit=SET_CorrectionA_LOW>>16;
	EEPROM_WriteByte(0x3B, data_8bit);
	data_8bit=SET_CorrectionA_LOW>>8;
	EEPROM_WriteByte(0x3C, data_8bit);
	EEPROM_WriteByte(0x3D, SET_CorrectionA_LOW);
	
	data_8bit=SET_CorrectionA_HIG>>24;
	EEPROM_WriteByte(0x3E, data_8bit);
	data_8bit=SET_CorrectionA_HIG>>16;
	EEPROM_WriteByte(0x3F, data_8bit);
	data_8bit=SET_CorrectionA_HIG>>8;
	EEPROM_WriteByte(0x40, data_8bit);
	EEPROM_WriteByte(0x41, SET_CorrectionA_HIG);
	
	EEPROM_WriteByte(0x42,Polar1);
	IWDG_ReloadCounter(); //喂狗
/***********************************************************/
	EEPROM_WriteByte(0x43,Polar2);
	EEPROM_WriteByte(0x44,Polar3);
	
	
	/***************高段参数存储**********************/
	data_8bit = REG_ReadV_Offset_MID_HIG >> 24;
	EEPROM_WriteByte(0x45, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG >> 16;
	EEPROM_WriteByte(0x46, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG >> 8;
	EEPROM_WriteByte(0x47, data_8bit);
	EEPROM_WriteByte(0x48, REG_ReadV_Offset_MID_HIG);
	
	data_8bit = REG_ReadA_Offset_MID_HIG >> 24;
	EEPROM_WriteByte(0x49, data_8bit);
	data_8bit = REG_ReadA_Offset_MID_HIG >> 16;
	EEPROM_WriteByte(0x4A, data_8bit);
	data_8bit = REG_ReadA_Offset_MID_HIG >> 8;
	EEPROM_WriteByte(0x4B, data_8bit);
	EEPROM_WriteByte(0x4C, REG_ReadA_Offset_MID_HIG);
	
	data_8bit=REG_CorrectionV_MID_HIG>>24;
	EEPROM_WriteByte(0x4D, data_8bit);
	data_8bit=REG_CorrectionV_MID_HIG>>16;
	EEPROM_WriteByte(0x4E, data_8bit);
	data_8bit=REG_CorrectionV_MID_HIG>>8;
	EEPROM_WriteByte(0x4F, data_8bit);
	EEPROM_WriteByte(0x50, REG_CorrectionV_MID_HIG);
	
	data_8bit=REG_CorrectionA_MID_HIG>>24;
	EEPROM_WriteByte(0x51, data_8bit);
	data_8bit=REG_CorrectionA_MID_HIG>>16;
	EEPROM_WriteByte(0x52, data_8bit);
	data_8bit=REG_CorrectionA_MID_HIG>>8;
	EEPROM_WriteByte(0x53, data_8bit);
	EEPROM_WriteByte(0x54, REG_CorrectionA_MID_HIG);
	IWDG_ReloadCounter(); //喂狗
	/************高段设置参数存储********************/
	data_8bit = SET_ReadV_Offset_MID_HIG >> 24;
	EEPROM_WriteByte(0x55, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG >> 16;
	EEPROM_WriteByte(0x56, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG >> 8;
	EEPROM_WriteByte(0x57, data_8bit);
	EEPROM_WriteByte(0x58, SET_ReadV_Offset_MID_HIG);
	
	data_8bit = SET_ReadA_Offset_MID_HIG >> 24;
	EEPROM_WriteByte(0x59, data_8bit);
	data_8bit = SET_ReadA_Offset_MID_HIG >> 16;
	EEPROM_WriteByte(0x5A, data_8bit);
	data_8bit = SET_ReadA_Offset_MID_HIG >> 8;
	EEPROM_WriteByte(0x5B, data_8bit);
	EEPROM_WriteByte(0x5C, SET_ReadA_Offset_MID_HIG);
	IWDG_ReloadCounter(); //喂狗
	
	
	data_8bit=SET_CorrectionV_MID_HIG>>24;
	EEPROM_WriteByte(0x5D, data_8bit);
	data_8bit=SET_CorrectionV_MID_HIG>>16;
	EEPROM_WriteByte(0x5E, data_8bit);
	data_8bit=SET_CorrectionV_MID_HIG>>8;
	EEPROM_WriteByte(0x5F, data_8bit);
	EEPROM_WriteByte(0x60, SET_CorrectionV_MID_HIG);
	
	data_8bit=SET_CorrectionA_MID_HIG>>24;
	EEPROM_WriteByte(0x61, data_8bit);
	data_8bit=SET_CorrectionA_MID_HIG>>16;
	EEPROM_WriteByte(0x62, data_8bit);
	data_8bit=SET_CorrectionA_MID_HIG>>8;
	EEPROM_WriteByte(0x63, data_8bit);
	EEPROM_WriteByte(0x64, SET_CorrectionA_MID_HIG);
	
	EEPROM_WriteByte(0x65,Polar4);
	EEPROM_WriteByte(0x66,Polar5);
	IWDG_ReloadCounter(); //喂狗
	
///////////////////////////////////////////////////////

	data_8bit=REG_CorrectionA_LOW2>>24;
	EEPROM_WriteByte(0x67, data_8bit);
	data_8bit=REG_CorrectionA_LOW2>>16;
	EEPROM_WriteByte(0x68, data_8bit);
	data_8bit=REG_CorrectionA_LOW2>>8;
	EEPROM_WriteByte(0x69, data_8bit);
	EEPROM_WriteByte(0x6A, REG_CorrectionA_LOW2);
	
	data_8bit=REG_CorrectionA_LOW3>>24;
	EEPROM_WriteByte(0x6B, data_8bit);
	data_8bit=REG_CorrectionA_LOW3>>16;
	EEPROM_WriteByte(0x6C, data_8bit);
	data_8bit=REG_CorrectionA_LOW3>>8;
	EEPROM_WriteByte(0x6D, data_8bit);
	EEPROM_WriteByte(0x6E, REG_CorrectionA_LOW2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=REG_CorrectionA_HIG1>>24;
	EEPROM_WriteByte(0x6F, data_8bit);
	data_8bit=REG_CorrectionA_HIG1>>16;
	EEPROM_WriteByte(0x70, data_8bit);
	data_8bit=REG_CorrectionA_HIG1>>8;
	EEPROM_WriteByte(0x71, data_8bit);
	EEPROM_WriteByte(0x72, REG_CorrectionA_HIG1);
	
	data_8bit=REG_CorrectionA_HIG2>>24;
	EEPROM_WriteByte(0x77, data_8bit);
	data_8bit=REG_CorrectionA_HIG2>>16;
	EEPROM_WriteByte(0x78, data_8bit);
	data_8bit=REG_CorrectionA_HIG2>>8;
	EEPROM_WriteByte(0x79, data_8bit);
	EEPROM_WriteByte(0x7A, REG_CorrectionA_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=SET_CorrectionA_LOW2>>24;
	EEPROM_WriteByte(0x7B, data_8bit);
	data_8bit=SET_CorrectionA_LOW2>>16;
	EEPROM_WriteByte(0x7C, data_8bit);
	data_8bit=SET_CorrectionA_LOW2>>8;
	EEPROM_WriteByte(0x7D, data_8bit);
	EEPROM_WriteByte(0x7E, SET_CorrectionA_LOW2);
	
	data_8bit=SET_CorrectionA_LOW3>>24;
	EEPROM_WriteByte(0x7F, data_8bit);
	data_8bit=SET_CorrectionA_LOW3>>16;
	EEPROM_WriteByte(0x80, data_8bit);
	data_8bit=SET_CorrectionA_LOW3>>8;
	EEPROM_WriteByte(0x81, data_8bit);
	EEPROM_WriteByte(0x82, SET_CorrectionA_LOW3);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=SET_CorrectionA_HIG1>>24;
	EEPROM_WriteByte(0x83, data_8bit);
	data_8bit=SET_CorrectionA_HIG1>>16;
	EEPROM_WriteByte(0x84, data_8bit);
	data_8bit=SET_CorrectionA_HIG1>>8;
	EEPROM_WriteByte(0x85, data_8bit);
	EEPROM_WriteByte(0x86, SET_CorrectionA_HIG1);
	
	data_8bit=SET_CorrectionA_HIG2>>24;
	EEPROM_WriteByte(0x87, data_8bit);
	data_8bit=SET_CorrectionA_HIG2>>16;
	EEPROM_WriteByte(0x88, data_8bit);
	data_8bit=SET_CorrectionA_HIG2>>8;
	EEPROM_WriteByte(0x89, data_8bit);
	EEPROM_WriteByte(0x8A, SET_CorrectionA_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=REG_ReadA_Offset_LOW2>>24;
	EEPROM_WriteByte(0x8B, data_8bit);
	data_8bit=REG_ReadA_Offset_LOW2>>16;
	EEPROM_WriteByte(0x8C, data_8bit);
	data_8bit=REG_ReadA_Offset_LOW2>>8;
	EEPROM_WriteByte(0x8D, data_8bit);
	EEPROM_WriteByte(0x8E, REG_ReadA_Offset_LOW2);
	
	data_8bit=REG_ReadA_Offset_LOW3>>24;
	EEPROM_WriteByte(0x8F, data_8bit);
	data_8bit=REG_ReadA_Offset_LOW3>>16;
	EEPROM_WriteByte(0x90, data_8bit);
	data_8bit=REG_ReadA_Offset_LOW3>>8;
	EEPROM_WriteByte(0x91, data_8bit);
	EEPROM_WriteByte(0x92, REG_ReadA_Offset_LOW3);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=REG_ReadA_Offset_HIG1>>24;
	EEPROM_WriteByte(0x93, data_8bit);
	data_8bit=REG_ReadA_Offset_HIG1>>16;
	EEPROM_WriteByte(0x94, data_8bit);
	data_8bit=REG_ReadA_Offset_HIG1>>8;
	EEPROM_WriteByte(0x95, data_8bit);
	EEPROM_WriteByte(0x96, REG_ReadA_Offset_HIG1);
	
	data_8bit=REG_ReadA_Offset_HIG2>>24;
	EEPROM_WriteByte(0x97, data_8bit);
	data_8bit=REG_ReadA_Offset_HIG2>>16;
	EEPROM_WriteByte(0x98, data_8bit);
	data_8bit=REG_ReadA_Offset_HIG2>>8;
	EEPROM_WriteByte(0x99, data_8bit);
	EEPROM_WriteByte(0x9A, REG_ReadA_Offset_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=SET_ReadA_Offset_LOW2>>24;
	EEPROM_WriteByte(0x9B, data_8bit);
	data_8bit=SET_ReadA_Offset_LOW2>>16;
	EEPROM_WriteByte(0x9C, data_8bit);
	data_8bit=SET_ReadA_Offset_LOW2>>8;
	EEPROM_WriteByte(0x9D, data_8bit);
	EEPROM_WriteByte(0x9E, SET_ReadA_Offset_LOW2);
	
	data_8bit=SET_ReadA_Offset_LOW3>>24;
	EEPROM_WriteByte(0x9F, data_8bit);
	data_8bit=SET_ReadA_Offset_LOW3>>16;
	EEPROM_WriteByte(0xA0, data_8bit);
	data_8bit=SET_ReadA_Offset_LOW3>>8;
	EEPROM_WriteByte(0xA1, data_8bit);
	EEPROM_WriteByte(0xA2, SET_ReadA_Offset_LOW3);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit=SET_ReadA_Offset_HIG1>>24;
	EEPROM_WriteByte(0xA3, data_8bit);
	data_8bit=SET_ReadA_Offset_HIG1>>16;
	EEPROM_WriteByte(0xA4, data_8bit);
	data_8bit=SET_ReadA_Offset_HIG1>>8;
	EEPROM_WriteByte(0xA5, data_8bit);
	EEPROM_WriteByte(0xA6, SET_ReadA_Offset_HIG1);
	
	data_8bit=SET_ReadA_Offset_HIG2>>24;
	EEPROM_WriteByte(0xA7, data_8bit);
	data_8bit=SET_ReadA_Offset_HIG2>>16;
	EEPROM_WriteByte(0xA8, data_8bit);
	data_8bit=SET_ReadA_Offset_HIG2>>8;
	EEPROM_WriteByte(0xA9, data_8bit);
	EEPROM_WriteByte(0xAA, SET_ReadA_Offset_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_CorrectionV_MID_HIG1 >> 24;
	EEPROM_WriteByte(0xAF, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG1 >> 16;
	EEPROM_WriteByte(0xB0, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG1 >> 8;
	EEPROM_WriteByte(0xB1, data_8bit);
	EEPROM_WriteByte(0xB2, REG_CorrectionV_MID_HIG1);
	
	data_8bit = SET_CorrectionV_MID_HIG1 >> 24;
	EEPROM_WriteByte(0xB3, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG1 >> 16;
	EEPROM_WriteByte(0xB4, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG1 >> 8;
	EEPROM_WriteByte(0xB5, data_8bit);
	EEPROM_WriteByte(0xB6, SET_CorrectionV_MID_HIG1);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_CorrectionV_MID_HIG2 >> 24;
	EEPROM_WriteByte(0xB7, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG2 >> 16;
	EEPROM_WriteByte(0xB8, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG2 >> 8;
	EEPROM_WriteByte(0xB9, data_8bit);
	EEPROM_WriteByte(0xBA, REG_CorrectionV_MID_HIG2);
	
	data_8bit = SET_CorrectionV_MID_HIG2 >> 24;
	EEPROM_WriteByte(0xBB, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG2 >> 16;
	EEPROM_WriteByte(0xBC, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG2 >> 8;
	EEPROM_WriteByte(0xBD, data_8bit);
	EEPROM_WriteByte(0xBE, SET_CorrectionV_MID_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_CorrectionV_MID_HIG3 >> 24;
	EEPROM_WriteByte(0xBF, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG3 >> 16;
	EEPROM_WriteByte(0xC0, data_8bit);
	data_8bit = REG_CorrectionV_MID_HIG3 >> 8;
	EEPROM_WriteByte(0xC1, data_8bit);
	EEPROM_WriteByte(0xC2, REG_CorrectionV_MID_HIG3);
	
	data_8bit = SET_CorrectionV_MID_HIG3 >> 24;
	EEPROM_WriteByte(0xC3, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG3 >> 16;
	EEPROM_WriteByte(0xC4, data_8bit);
	data_8bit = SET_CorrectionV_MID_HIG3 >> 8;
	EEPROM_WriteByte(0xC5, data_8bit);
	EEPROM_WriteByte(0xC6, SET_CorrectionV_MID_HIG3);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_ReadV_Offset_MID_HIG1 >> 24;
	EEPROM_WriteByte(0xC7, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG1 >> 16;
	EEPROM_WriteByte(0xC8, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG1 >> 8;
	EEPROM_WriteByte(0xC9, data_8bit);
	EEPROM_WriteByte(0xCA, REG_ReadV_Offset_MID_HIG1);
	
	data_8bit = SET_ReadV_Offset_MID_HIG1 >> 24;
	EEPROM_WriteByte(0xCB, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG1 >> 16;
	EEPROM_WriteByte(0xCC, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG1 >> 8;
	EEPROM_WriteByte(0xCD, data_8bit);
	EEPROM_WriteByte(0xCE, SET_ReadV_Offset_MID_HIG1);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_ReadV_Offset_MID_HIG2 >> 24;
	EEPROM_WriteByte(0xCF, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG2 >> 16;
	EEPROM_WriteByte(0xD0, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG2 >> 8;
	EEPROM_WriteByte(0xD1, data_8bit);
	EEPROM_WriteByte(0xD2, REG_ReadV_Offset_MID_HIG2);
	
	data_8bit = SET_ReadV_Offset_MID_HIG2 >> 24;
	EEPROM_WriteByte(0xD3, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG2 >> 16;
	EEPROM_WriteByte(0xD4, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG2 >> 8;
	EEPROM_WriteByte(0xD5, data_8bit);
	EEPROM_WriteByte(0xD6, SET_ReadV_Offset_MID_HIG2);
	IWDG_ReloadCounter(); //喂狗
	
	data_8bit = REG_ReadV_Offset_MID_HIG3 >> 24;
	EEPROM_WriteByte(0xD7, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG3 >> 16;
	EEPROM_WriteByte(0xD8, data_8bit);
	data_8bit = REG_ReadV_Offset_MID_HIG3 >> 8;
	EEPROM_WriteByte(0xD9, data_8bit);
	EEPROM_WriteByte(0xDA, REG_ReadV_Offset_MID_HIG3);
	
	data_8bit = SET_ReadV_Offset_MID_HIG3 >> 24;
	EEPROM_WriteByte(0xDB, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG3 >> 16;
	EEPROM_WriteByte(0xDC, data_8bit);
	data_8bit = SET_ReadV_Offset_MID_HIG3 >> 8;
	EEPROM_WriteByte(0xDD, data_8bit);
	EEPROM_WriteByte(0xDE, SET_ReadV_Offset_MID_HIG3);
	IWDG_ReloadCounter(); //喂狗
	
	EEPROM_WriteByte(0xAB,Polar6);
	EEPROM_WriteByte(0xAC,Polar7);
	EEPROM_WriteByte(0xAD,Polar8);
	EEPROM_WriteByte(0xAE,Polar9);
	EEPROM_WriteByte(0xDF,Polar10);
	EEPROM_WriteByte(0xE0,Polar11);
	EEPROM_WriteByte(0xE1,Polar12);
	IWDG_ReloadCounter(); //喂狗
	/************设置参数存储*******************/
//	data_8bit = MODE >> 24;
//	EEPROM_WriteByte(0x67, data_8bit);
//	data_8bit = MODE >> 16;
//	EEPROM_WriteByte(0x68, data_8bit);
//	data_8bit = MODE >> 8;
//	EEPROM_WriteByte(0x69, data_8bit);
//	EEPROM_WriteByte(0x6A, MODE);
//	
//	data_8bit = I_Gear_SW >> 24;
//	EEPROM_WriteByte(0x6B, data_8bit);
//	data_8bit = I_Gear_SW >> 16;
//	EEPROM_WriteByte(0x6C, data_8bit);
//	data_8bit = I_Gear_SW >> 8;
//	EEPROM_WriteByte(0x6D, data_8bit);
//	EEPROM_WriteByte(0x6E, I_Gear_SW);
//	
//	data_8bit = V_Gear_SW >> 24;
//	EEPROM_WriteByte(0x6F, data_8bit);
//	data_8bit = V_Gear_SW >> 16;
//	EEPROM_WriteByte(0x70, data_8bit);
//	data_8bit = V_Gear_SW >> 8;
//	EEPROM_WriteByte(0x71, data_8bit);
//	EEPROM_WriteByte(0x72, V_Gear_SW);
//	
//	data_8bit = ADDR >> 24;
//	EEPROM_WriteByte(0x73, data_8bit);
//	data_8bit = ADDR >> 16;
//	EEPROM_WriteByte(0x74, data_8bit);
//	data_8bit = ADDR >> 8;
//	EEPROM_WriteByte(0x75, data_8bit);
//	EEPROM_WriteByte(0x76, ADDR);
	
	
	
//	data_8bit = Sence_SW >> 24;
//	EEPROM_WriteByte(0x77, data_8bit);
//	data_8bit = Sence_SW >> 16;
//	EEPROM_WriteByte(0x78, data_8bit);
//	data_8bit = Sence_SW >> 8;
//	EEPROM_WriteByte(0x79, data_8bit);
//	EEPROM_WriteByte(0x7A, Sence_SW);
}
//===========================================================================//
void EEPROM_READ_Coeff(void)
{
//	ReadBytes(Calibrate.CalFlash,0,sizeof(CalPara));
/****************测量参数读取*********************/
	REG_ReadV_Offset_LOW=EEPROM_READ_Byte(0x01);
	REG_ReadV_Offset_LOW=EEPROM_READ_Byte(0x01);
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW<<8;
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW+EEPROM_READ_Byte(0x02);
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW<<8;
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW+EEPROM_READ_Byte(0x03);
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW<<8;
	REG_ReadV_Offset_LOW=REG_ReadV_Offset_LOW+EEPROM_READ_Byte(0x04);
	
	REG_ReadV_Offset_HIG=EEPROM_READ_Byte(0x05);
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG<<8;
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG+EEPROM_READ_Byte(0x06);
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG<<8;
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG+EEPROM_READ_Byte(0x07);
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG<<8;
	REG_ReadV_Offset_HIG=REG_ReadV_Offset_HIG+EEPROM_READ_Byte(0x08);
	
	REG_ReadA_Offset_LOW=EEPROM_READ_Byte(0x09);
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW<<8;
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW+EEPROM_READ_Byte(0x0A);
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW<<8;
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW+EEPROM_READ_Byte(0x0B);
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW<<8;
	REG_ReadA_Offset_LOW=REG_ReadA_Offset_LOW+EEPROM_READ_Byte(0x0C);
	
	REG_ReadA_Offset_HIG=EEPROM_READ_Byte(0x0D);
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG<<8;
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG+EEPROM_READ_Byte(0x0E);
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG<<8;
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG+EEPROM_READ_Byte(0x0F);
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG<<8;
	REG_ReadA_Offset_HIG=REG_ReadA_Offset_HIG+EEPROM_READ_Byte(0x10);
	
	REG_CorrectionV_LOW=EEPROM_READ_Byte(0x11);
	REG_CorrectionV_LOW=REG_CorrectionV_LOW<<8;
	REG_CorrectionV_LOW=REG_CorrectionV_LOW+EEPROM_READ_Byte(0x12);
	REG_CorrectionV_LOW=REG_CorrectionV_LOW<<8;
	REG_CorrectionV_LOW=REG_CorrectionV_LOW+EEPROM_READ_Byte(0x13);
	REG_CorrectionV_LOW=REG_CorrectionV_LOW<<8;
	REG_CorrectionV_LOW=REG_CorrectionV_LOW+EEPROM_READ_Byte(0x14);
	
	REG_CorrectionV_HIG=EEPROM_READ_Byte(0x15);
	REG_CorrectionV_HIG=REG_CorrectionV_HIG<<8;
	REG_CorrectionV_HIG=REG_CorrectionV_HIG+EEPROM_READ_Byte(0x16);
	REG_CorrectionV_HIG=REG_CorrectionV_HIG<<8;
	REG_CorrectionV_HIG=REG_CorrectionV_HIG+EEPROM_READ_Byte(0x17);
	REG_CorrectionV_HIG=REG_CorrectionV_HIG<<8;
	REG_CorrectionV_HIG=REG_CorrectionV_HIG+EEPROM_READ_Byte(0x18);
	
	REG_CorrectionA_LOW=EEPROM_READ_Byte(0x19);
	REG_CorrectionA_LOW=REG_CorrectionA_LOW<<8;
	REG_CorrectionA_LOW=REG_CorrectionA_LOW+EEPROM_READ_Byte(0x1A);
	REG_CorrectionA_LOW=REG_CorrectionA_LOW<<8;
	REG_CorrectionA_LOW=REG_CorrectionA_LOW+EEPROM_READ_Byte(0x1B);
	REG_CorrectionA_LOW=REG_CorrectionA_LOW<<8;
	REG_CorrectionA_LOW=REG_CorrectionA_LOW+EEPROM_READ_Byte(0x1C);
	
	REG_CorrectionA_HIG=EEPROM_READ_Byte(0x1D);
	REG_CorrectionA_HIG=REG_CorrectionA_HIG<<8;
	REG_CorrectionA_HIG=REG_CorrectionA_HIG+EEPROM_READ_Byte(0x1E);
	REG_CorrectionA_HIG=REG_CorrectionA_HIG<<8;
	REG_CorrectionA_HIG=REG_CorrectionA_HIG+EEPROM_READ_Byte(0x1F);
	REG_CorrectionA_HIG=REG_CorrectionA_HIG<<8;
	REG_CorrectionA_HIG=REG_CorrectionA_HIG+EEPROM_READ_Byte(0x20);
	
	Polar0=EEPROM_READ_Byte(0x21);
/*******************设置参数读取****************************/
	SET_ReadV_Offset_LOW=EEPROM_READ_Byte(0x22);
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW<<8;
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW+EEPROM_READ_Byte(0x23);
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW<<8;
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW+EEPROM_READ_Byte(0x24);
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW<<8;
	SET_ReadV_Offset_LOW=SET_ReadV_Offset_LOW+EEPROM_READ_Byte(0x25);
	
	SET_ReadV_Offset_HIG=EEPROM_READ_Byte(0x26);
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG<<8;
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG+EEPROM_READ_Byte(0x27);
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG<<8;
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG+EEPROM_READ_Byte(0x28);
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG<<8;
	SET_ReadV_Offset_HIG=SET_ReadV_Offset_HIG+EEPROM_READ_Byte(0x29);
	
	SET_ReadA_Offset_LOW=EEPROM_READ_Byte(0x2A);
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW<<8;
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW+EEPROM_READ_Byte(0x2B);
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW<<8;
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW+EEPROM_READ_Byte(0x2C);
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW<<8;
	SET_ReadA_Offset_LOW=SET_ReadA_Offset_LOW+EEPROM_READ_Byte(0x2D);
	
	SET_ReadA_Offset_HIG=EEPROM_READ_Byte(0x2E);
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG<<8;
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG+EEPROM_READ_Byte(0x2F);
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG<<8;
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG+EEPROM_READ_Byte(0x30);
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG<<8;
	SET_ReadA_Offset_HIG=SET_ReadA_Offset_HIG+EEPROM_READ_Byte(0x31);
	
	SET_CorrectionV_LOW=EEPROM_READ_Byte(0x32);
	SET_CorrectionV_LOW=SET_CorrectionV_LOW<<8;
	SET_CorrectionV_LOW=SET_CorrectionV_LOW+EEPROM_READ_Byte(0x33);
	SET_CorrectionV_LOW=SET_CorrectionV_LOW<<8;
	SET_CorrectionV_LOW=SET_CorrectionV_LOW+EEPROM_READ_Byte(0x34);
	SET_CorrectionV_LOW=SET_CorrectionV_LOW<<8;
	SET_CorrectionV_LOW=SET_CorrectionV_LOW+EEPROM_READ_Byte(0x35);
	
	SET_CorrectionV_HIG=EEPROM_READ_Byte(0x36);
	SET_CorrectionV_HIG=SET_CorrectionV_HIG<<8;
	SET_CorrectionV_HIG=SET_CorrectionV_HIG+EEPROM_READ_Byte(0x37);
	SET_CorrectionV_HIG=SET_CorrectionV_HIG<<8;
	SET_CorrectionV_HIG=SET_CorrectionV_HIG+EEPROM_READ_Byte(0x38);
	SET_CorrectionV_HIG=SET_CorrectionV_HIG<<8;
	SET_CorrectionV_HIG=SET_CorrectionV_HIG+EEPROM_READ_Byte(0x39);
	
	SET_CorrectionA_LOW=EEPROM_READ_Byte(0x3A);
	SET_CorrectionA_LOW=SET_CorrectionA_LOW<<8;
	SET_CorrectionA_LOW=SET_CorrectionA_LOW+EEPROM_READ_Byte(0x3B);
	SET_CorrectionA_LOW=SET_CorrectionA_LOW<<8;
	SET_CorrectionA_LOW=SET_CorrectionA_LOW+EEPROM_READ_Byte(0x3C);
	SET_CorrectionA_LOW=SET_CorrectionA_LOW<<8;
	SET_CorrectionA_LOW=SET_CorrectionA_LOW+EEPROM_READ_Byte(0x3D);
	
	
	SET_CorrectionA_HIG=EEPROM_READ_Byte(0x3E);
	SET_CorrectionA_HIG=SET_CorrectionA_HIG<<8;
	SET_CorrectionA_HIG=SET_CorrectionA_HIG+EEPROM_READ_Byte(0x3F);
	SET_CorrectionA_HIG=SET_CorrectionA_HIG<<8;
	SET_CorrectionA_HIG=SET_CorrectionA_HIG+EEPROM_READ_Byte(0x40);
	SET_CorrectionA_HIG=SET_CorrectionA_HIG<<8;
	SET_CorrectionA_HIG=SET_CorrectionA_HIG+EEPROM_READ_Byte(0x41);
	
	Polar1=EEPROM_READ_Byte(0x42);
	Polar2=EEPROM_READ_Byte(0x43);
	Polar3=EEPROM_READ_Byte(0x44);
	/**************高段测量参数读取**************************/
	REG_ReadV_Offset_MID_HIG=EEPROM_READ_Byte(0x45);
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG<<8;
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x46);
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG<<8;
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x47);
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG<<8;
	REG_ReadV_Offset_MID_HIG=REG_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x48);
	
	REG_ReadA_Offset_MID_HIG=EEPROM_READ_Byte(0x49);
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG<<8;
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x4A);
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG<<8;
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x4B);
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG<<8;
	REG_ReadA_Offset_MID_HIG=REG_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x4C);
	
	REG_CorrectionV_MID_HIG=EEPROM_READ_Byte(0x4D);
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG<<8;
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x4E);
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG<<8;
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x4F);
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG<<8;
	REG_CorrectionV_MID_HIG=REG_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x50);
	
	REG_CorrectionA_MID_HIG=EEPROM_READ_Byte(0x51);
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG<<8;
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x52);
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG<<8;
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x53);
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG<<8;
	REG_CorrectionA_MID_HIG=REG_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x54);
	
	/****************高段设置参数读取***************************/
	SET_ReadV_Offset_MID_HIG=EEPROM_READ_Byte(0x55);
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG<<8;
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x56);
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG<<8;
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x57);
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG<<8;
	SET_ReadV_Offset_MID_HIG=SET_ReadV_Offset_MID_HIG+EEPROM_READ_Byte(0x58);
	
	SET_ReadA_Offset_MID_HIG=EEPROM_READ_Byte(0x59);
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG<<8;
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x5A);
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG<<8;
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x5B);
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG<<8;
	SET_ReadA_Offset_MID_HIG=SET_ReadA_Offset_MID_HIG+EEPROM_READ_Byte(0x5C);
	
	SET_CorrectionV_MID_HIG=EEPROM_READ_Byte(0x5D);
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG<<8;
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x5E);
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG<<8;
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x5F);
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG<<8;
	SET_CorrectionV_MID_HIG=SET_CorrectionV_MID_HIG+EEPROM_READ_Byte(0x60);
	
	SET_CorrectionA_MID_HIG=EEPROM_READ_Byte(0x61);
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG<<8;
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x62);
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG<<8;
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x63);
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG<<8;
	SET_CorrectionA_MID_HIG=SET_CorrectionA_MID_HIG+EEPROM_READ_Byte(0x64);
	
	Polar4=EEPROM_READ_Byte(0x65);
	Polar5=EEPROM_READ_Byte(0x66);
	
	ADDR=EEPROM_READ_Byte(0x73);
	ADDR=ADDR<<8;
	ADDR=ADDR+EEPROM_READ_Byte(0x74);
	ADDR=ADDR<<8;
	ADDR=ADDR+EEPROM_READ_Byte(0x75);
	ADDR=ADDR<<8;
	ADDR=ADDR+EEPROM_READ_Byte(0x76);
	
	
	REG_CorrectionA_LOW2=EEPROM_READ_Byte(0x67);
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2<<8;
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2+EEPROM_READ_Byte(0x68);
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2<<8;
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2+EEPROM_READ_Byte(0x69);
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2<<8;
	REG_CorrectionA_LOW2=REG_CorrectionA_LOW2+EEPROM_READ_Byte(0x6A);
	
	REG_CorrectionA_LOW3=EEPROM_READ_Byte(0x6B);
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3<<8;
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3+EEPROM_READ_Byte(0x6C);
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3<<8;
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3+EEPROM_READ_Byte(0x6D);
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3<<8;
	REG_CorrectionA_LOW3=REG_CorrectionA_LOW3+EEPROM_READ_Byte(0x6E);
	
	REG_CorrectionA_HIG1=EEPROM_READ_Byte(0x6F);
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1<<8;
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1+EEPROM_READ_Byte(0x70);
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1<<8;
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1+EEPROM_READ_Byte(0x71);
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1<<8;
	REG_CorrectionA_HIG1=REG_CorrectionA_HIG1+EEPROM_READ_Byte(0x72);
	
	REG_CorrectionA_HIG2=EEPROM_READ_Byte(0x77);
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2<<8;
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2+EEPROM_READ_Byte(0x78);
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2<<8;
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2+EEPROM_READ_Byte(0x79);
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2<<8;
	REG_CorrectionA_HIG2=REG_CorrectionA_HIG2+EEPROM_READ_Byte(0x7A);
	
	SET_CorrectionA_LOW2=EEPROM_READ_Byte(0x7B);
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2<<8;
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2+EEPROM_READ_Byte(0x7C);
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2<<8;
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2+EEPROM_READ_Byte(0x7D);
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2<<8;
	SET_CorrectionA_LOW2=SET_CorrectionA_LOW2+EEPROM_READ_Byte(0x7E);
	
	SET_CorrectionA_LOW3=EEPROM_READ_Byte(0x7F);
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3<<8;
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3+EEPROM_READ_Byte(0x80);
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3<<8;
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3+EEPROM_READ_Byte(0x81);
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3<<8;
	SET_CorrectionA_LOW3=SET_CorrectionA_LOW3+EEPROM_READ_Byte(0x82);
	
	SET_CorrectionA_HIG1=EEPROM_READ_Byte(0x83);
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1<<8;
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1+EEPROM_READ_Byte(0x84);
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1<<8;
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1+EEPROM_READ_Byte(0x85);
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1<<8;
	SET_CorrectionA_HIG1=SET_CorrectionA_HIG1+EEPROM_READ_Byte(0x86);
	
	SET_CorrectionA_HIG2=EEPROM_READ_Byte(0x87);
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2<<8;
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2+EEPROM_READ_Byte(0x88);
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2<<8;
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2+EEPROM_READ_Byte(0x89);
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2<<8;
	SET_CorrectionA_HIG2=SET_CorrectionA_HIG2+EEPROM_READ_Byte(0x8A);
	
	REG_ReadA_Offset_LOW2=EEPROM_READ_Byte(0x8B);
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2<<8;
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x8C);
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2<<8;
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x8D);
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2<<8;
	REG_ReadA_Offset_LOW2=REG_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x8E);
	
	REG_ReadA_Offset_LOW3=EEPROM_READ_Byte(0x8F);
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3<<8;
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3+EEPROM_READ_Byte(0x90);
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3<<8;
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3+EEPROM_READ_Byte(0x91);
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3<<8;
	REG_ReadA_Offset_LOW3=REG_ReadA_Offset_LOW3+EEPROM_READ_Byte(0x92);
	
	REG_ReadA_Offset_HIG1=EEPROM_READ_Byte(0x93);
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1<<8;
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1+EEPROM_READ_Byte(0x94);
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1<<8;
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1+EEPROM_READ_Byte(0x95);
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1<<8;
	REG_ReadA_Offset_HIG1=REG_ReadA_Offset_HIG1+EEPROM_READ_Byte(0x96);
	
	REG_ReadA_Offset_HIG2=EEPROM_READ_Byte(0x97);
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2<<8;
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2+EEPROM_READ_Byte(0x98);
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2<<8;
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2+EEPROM_READ_Byte(0x99);
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2<<8;
	REG_ReadA_Offset_HIG2=REG_ReadA_Offset_HIG2+EEPROM_READ_Byte(0x9A);
	
	SET_ReadA_Offset_LOW2=EEPROM_READ_Byte(0x9B);
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2<<8;
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x9C);
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2<<8;
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x9D);
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2<<8;
	SET_ReadA_Offset_LOW2=SET_ReadA_Offset_LOW2+EEPROM_READ_Byte(0x9E);
	
	SET_ReadA_Offset_LOW3=EEPROM_READ_Byte(0x9F);
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3<<8;
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3+EEPROM_READ_Byte(0xA0);
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3<<8;
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3+EEPROM_READ_Byte(0xA1);
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3<<8;
	SET_ReadA_Offset_LOW3=SET_ReadA_Offset_LOW3+EEPROM_READ_Byte(0xA2);
	
	SET_ReadA_Offset_HIG1=EEPROM_READ_Byte(0xA3);
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1<<8;
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1+EEPROM_READ_Byte(0xA4);
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1<<8;
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1+EEPROM_READ_Byte(0xA5);
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1<<8;
	SET_ReadA_Offset_HIG1=SET_ReadA_Offset_HIG1+EEPROM_READ_Byte(0xA6);
	
	SET_ReadA_Offset_HIG2=EEPROM_READ_Byte(0xA7);
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2<<8;
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2+EEPROM_READ_Byte(0xA8);
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2<<8;
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2+EEPROM_READ_Byte(0xA9);
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2<<8;
	SET_ReadA_Offset_HIG2=SET_ReadA_Offset_HIG2+EEPROM_READ_Byte(0xAA);
	
	REG_CorrectionV_MID_HIG1=EEPROM_READ_Byte(0xAF);
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1<<8;
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB0);
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1<<8;
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB1);
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1<<8;
	REG_CorrectionV_MID_HIG1=REG_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB2);
	
	SET_CorrectionV_MID_HIG1=EEPROM_READ_Byte(0xB3);
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1<<8;
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB4);
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1<<8;
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB5);
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1<<8;
	SET_CorrectionV_MID_HIG1=SET_CorrectionV_MID_HIG1+EEPROM_READ_Byte(0xB6);
	
	REG_CorrectionV_MID_HIG2=EEPROM_READ_Byte(0xB7);
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2<<8;
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xB8);
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2<<8;
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xB9);
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2<<8;
	REG_CorrectionV_MID_HIG2=REG_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xBA);
	
	SET_CorrectionV_MID_HIG2=EEPROM_READ_Byte(0xBB);
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2<<8;
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xBC);
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2<<8;
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xBD);
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2<<8;
	SET_CorrectionV_MID_HIG2=SET_CorrectionV_MID_HIG2+EEPROM_READ_Byte(0xBE);
	
	REG_CorrectionV_MID_HIG3=EEPROM_READ_Byte(0xBF);
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3<<8;
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC0);
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3<<8;
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC1);
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3<<8;
	REG_CorrectionV_MID_HIG3=REG_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC2);
	
	SET_CorrectionV_MID_HIG3=EEPROM_READ_Byte(0xC3);
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3<<8;
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC4);
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3<<8;
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC5);
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3<<8;
	SET_CorrectionV_MID_HIG3=SET_CorrectionV_MID_HIG3+EEPROM_READ_Byte(0xC6);
	
	REG_ReadV_Offset_MID_HIG1=EEPROM_READ_Byte(0xC7);
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1<<8;
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xC8);
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1<<8;
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xC9);
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1<<8;
	REG_ReadV_Offset_MID_HIG1=REG_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xCA);
	
	SET_ReadV_Offset_MID_HIG1=EEPROM_READ_Byte(0xCB);
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1<<8;
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xCC);
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1<<8;
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xCD);
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1<<8;
	SET_ReadV_Offset_MID_HIG1=SET_ReadV_Offset_MID_HIG1+EEPROM_READ_Byte(0xCE);
	
	REG_ReadV_Offset_MID_HIG2=EEPROM_READ_Byte(0xCF);
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2<<8;
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD0);
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2<<8;
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD1);
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2<<8;
	REG_ReadV_Offset_MID_HIG2=REG_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD2);
	
	SET_ReadV_Offset_MID_HIG2=EEPROM_READ_Byte(0xD3);
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2<<8;
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD4);
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2<<8;
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD5);
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2<<8;
	SET_ReadV_Offset_MID_HIG2=SET_ReadV_Offset_MID_HIG2+EEPROM_READ_Byte(0xD6);
	
	REG_ReadV_Offset_MID_HIG3=EEPROM_READ_Byte(0xD7);
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3<<8;
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xD8);
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3<<8;
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xD9);
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3<<8;
	REG_ReadV_Offset_MID_HIG3=REG_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xDA);
	
	SET_ReadV_Offset_MID_HIG3=EEPROM_READ_Byte(0xDB);
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3<<8;
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xDC);
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3<<8;
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xDD);
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3<<8;
	SET_ReadV_Offset_MID_HIG3=SET_ReadV_Offset_MID_HIG3+EEPROM_READ_Byte(0xDE);
	
	Polar6=EEPROM_READ_Byte(0xAB);
	Polar7=EEPROM_READ_Byte(0xAC);
	Polar8=EEPROM_READ_Byte(0xAD);
	Polar9=EEPROM_READ_Byte(0xAE);
	Polar10=EEPROM_READ_Byte(0xDF);
	Polar11=EEPROM_READ_Byte(0xE0);
	Polar12=EEPROM_READ_Byte(0xE1);
//	Read_Runcont();//读运行参数
//	if(Baud_rate==9600)
//	{
//		Baud_rate=9600;
//	}
//	else if(Baud_rate==19200)
//	{
//		Baud_rate=19200;
//	}
//	else if(Baud_rate==56000)
//	{
//		Baud_rate=56000;
//	}
//	else if(Baud_rate==57600)
//	{
//		Baud_rate=57600;
//	}
//	else if(Baud_rate==115200)
//	{
//		Baud_rate=115200;
//	}
//	else if(Baud_rate==128000)
//	{
//		Baud_rate=128000;
//	}
//	else 
//	{
//		Baud_rate=9600;
//	}
}
void Write_ADDR(void)//将本机地址写入EEPROM
{
	vu8 data_8bit;
	
	data_8bit = ADDR >> 24;
	EEPROM_WriteByte(0x73, data_8bit);
	data_8bit = ADDR >> 16;
	EEPROM_WriteByte(0x74, data_8bit);
	data_8bit = ADDR >> 8;
	EEPROM_WriteByte(0x75, data_8bit);
	EEPROM_WriteByte(0x76, ADDR);
}
/************************************************************************/
void Wite_Runcont(void)//将运行参数写入EEPROM
{
	vu8 data_8bit;
	vu8 var8;
	vu8 ADDR_CONT;
	if(Flag_Save_SW ==1)
	{
    /***********************存储单个寄存器****************************/
		Flag_Save_SW=0;//清除标志位只保存一次
		ADDR_CONT=UART_Buffer_Rece[3];
		switch (ADDR_CONT)
		{
			case 13:
			{
				data_8bit = Run_Control[13] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+5, data_8bit);
				data_8bit = Run_Control[13] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+6, data_8bit);
				data_8bit = Run_Control[13] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+7, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+8, Run_Control[13]);
			}
			break;
			case 14:
			{
				data_8bit = Run_Control[14] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+9, data_8bit);
				data_8bit = Run_Control[14] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+10, data_8bit);
				data_8bit = Run_Control[14] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+11, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+12, Run_Control[14]);
			}
			break;
			case 15:
			{
				data_8bit = Run_Control[15] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+13, data_8bit);
				data_8bit = Run_Control[15] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+14, data_8bit);
				data_8bit = Run_Control[15] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+15, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+16, Run_Control[15]);
			}
			break;
			case 16:
			{
				data_8bit = Run_Control[16] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+17, data_8bit);
				data_8bit = Run_Control[16] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+18, data_8bit);
				data_8bit = Run_Control[16] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+19, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+20, Run_Control[16]);
			}
			break;
			case 17:
			{
				data_8bit = Run_Control[17] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+21, data_8bit);
				data_8bit = Run_Control[17] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+22, data_8bit);
				data_8bit = Run_Control[17] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+23, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+24, Run_Control[17]);
			}
			break;
			case 18:
			{
				data_8bit = Run_Control[18] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+25, data_8bit);
				data_8bit = Run_Control[18] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+26, data_8bit);
				data_8bit = Run_Control[18] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+27, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+28, Run_Control[18]);
			}
			break;
			case 19:
			{
				data_8bit = Run_Control[19] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+29, data_8bit);
				data_8bit = Run_Control[19] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+30, data_8bit);
				data_8bit = Run_Control[19] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+31, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+32, Run_Control[19]);
			}
			break;
			case 20:
			{
				data_8bit = Run_Control[20] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+33, data_8bit);
				data_8bit = Run_Control[20] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+34, data_8bit);
				data_8bit = Run_Control[20] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+35, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+36, Run_Control[20]);
			}
			break;
			case 21:
			{
				data_8bit = Run_Control[21] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+37, data_8bit);
				data_8bit = Run_Control[21] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+38, data_8bit);
				data_8bit = Run_Control[21] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+39, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+40, Run_Control[21]);
			}
			break;
			case 22:
			{
				data_8bit = Run_Control[22] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+41, data_8bit);
				data_8bit = Run_Control[22] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+42, data_8bit);
				data_8bit = Run_Control[22] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+43, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+44, Run_Control[22]);
			}
			break;
			case 23:
			{
				data_8bit = Run_Control[23] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+45, data_8bit);
				data_8bit = Run_Control[23] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+46, data_8bit);
				data_8bit = Run_Control[23] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+47, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+48, Run_Control[23]);
			}
			break;
			case 24:
			{
				data_8bit = Run_Control[24] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+49, data_8bit);
				data_8bit = Run_Control[24] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+50, data_8bit);
				data_8bit = Run_Control[24] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+51, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+52, Run_Control[24]);
			}
			break;
			case 25:
			{
				data_8bit = Run_Control[25] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+53, data_8bit);
				data_8bit = Run_Control[25] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+54, data_8bit);
				data_8bit = Run_Control[25] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+55, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+56, Run_Control[25]);
			}
			break;
			case 26:
			{
				data_8bit = Run_Control[26] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+57, data_8bit);
				data_8bit = Run_Control[26] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+58, data_8bit);
				data_8bit = Run_Control[26] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+59, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+60, Run_Control[26]);
			}
			break;
			case 27:
			{
				data_8bit = Run_Control[27] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+61, data_8bit);
				data_8bit = Run_Control[27] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+62, data_8bit);
				data_8bit = Run_Control[27] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+63, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+64, Run_Control[27]);
			}
			break;
			case 28:
			{
				data_8bit = Run_Control[28] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+65, data_8bit);
				data_8bit = Run_Control[28] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+66, data_8bit);
				data_8bit = Run_Control[28] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+67, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+68, Run_Control[28]);
			}
			break;
			case 29:
			{
				data_8bit = Run_Control[29] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+69, data_8bit);
				data_8bit = Run_Control[29] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+70, data_8bit);
				data_8bit = Run_Control[29] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+71, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+72, Run_Control[29]);
			}
			break;
			case 30:
			{
				data_8bit = Run_Control[30] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+73, data_8bit);
				data_8bit = Run_Control[30] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+74, data_8bit);
				data_8bit = Run_Control[30] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+75, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+76, Run_Control[30]);
			}
			break;
			case 31:
			{
				data_8bit = Run_Control[31] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+77, data_8bit);
				data_8bit = Run_Control[31] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+78, data_8bit);
				data_8bit = Run_Control[31] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+79, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+80, Run_Control[31]);
			}
			break;
			case 32:
			{
				data_8bit = Run_Control[32] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+81, data_8bit);
				data_8bit = Run_Control[32] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+82, data_8bit);
				data_8bit = Run_Control[32] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+83, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+84, Run_Control[32]);
			}
			break;
			case 33:
			{
				data_8bit = Run_Control[33] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+85, data_8bit);
				data_8bit = Run_Control[33] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+86, data_8bit);
				data_8bit = Run_Control[33] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+87, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+88, Run_Control[33]);
			}
			break;
			case 34:
			{
				data_8bit = Run_Control[34] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+89, data_8bit);
				data_8bit = Run_Control[34] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+90, data_8bit);
				data_8bit = Run_Control[34] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+91, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+92, Run_Control[34]);
			}
			break;
			case 35:
			{
				data_8bit = Run_Control[35] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+93, data_8bit);
				data_8bit = Run_Control[35] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+94, data_8bit);
				data_8bit = Run_Control[35] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+95, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+96, Run_Control[35]);
			}
			break;
			case 36:
			{
				data_8bit = Run_Control[36] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+97, data_8bit);
				data_8bit = Run_Control[36] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+98, data_8bit);
				data_8bit = Run_Control[36] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+99, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+100, Run_Control[36]);
			}
			break;
			case 37:
			{
				data_8bit = Run_Control[37] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+101, data_8bit);
				data_8bit = Run_Control[37] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+102, data_8bit);
				data_8bit = Run_Control[37] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+103, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+104, Run_Control[37]);
			}
			break;
			case 38:
			{
				data_8bit = Run_Control[38] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+105, data_8bit);
				data_8bit = Run_Control[38] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+106, data_8bit);
				data_8bit = Run_Control[38] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+107, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+108, Run_Control[38]);
			}
			break;
			case 39:
			{
				data_8bit = Run_Control[39] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+109, data_8bit);
				data_8bit = Run_Control[39] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+110, data_8bit);
				data_8bit = Run_Control[39] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+111, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+112, Run_Control[39]);
			}
			break;
			case 40:
			{
				data_8bit = Run_Control[40] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+113, data_8bit);
				data_8bit = Run_Control[40] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+114, data_8bit);
				data_8bit = Run_Control[40] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+115, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+116, Run_Control[40]);
			}
			break;
			case 41:
			{
				data_8bit = Run_Control[41] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+117, data_8bit);
				data_8bit = Run_Control[41] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+118, data_8bit);
				data_8bit = Run_Control[41] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+119, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+120, Run_Control[41]);
			}
			break;
			case 42:
			{
				data_8bit = Run_Control[42] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+121, data_8bit);
				data_8bit = Run_Control[42] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+122, data_8bit);
				data_8bit = Run_Control[42] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+123, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+124, Run_Control[42]);
			}
			break;
			case 43:
			{
				data_8bit = Run_Control[43] >> 24;
				EEPROM_WriteByte(MAX_Correct_Addr+125, data_8bit);
				data_8bit = Run_Control[43] >> 16;
				EEPROM_WriteByte(MAX_Correct_Addr+126, data_8bit);
				data_8bit = Run_Control[43] >> 8;
				EEPROM_WriteByte(MAX_Correct_Addr+127, data_8bit);
				EEPROM_WriteByte(MAX_Correct_Addr+128, Run_Control[43]);
			}
			break;
//			case 44:
//			{
//				data_8bit = Run_Control[44] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+129, data_8bit);
//				data_8bit = Run_Control[44] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+130, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+131, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+132, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 45:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+133, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+134, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+135, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+136, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 46:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+137, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+138, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+139, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+140, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 47:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+141, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+142, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+143, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+144, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 48:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+145, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+146, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+147, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+148, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 49:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+149, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+150, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+151, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+152, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
//			case 50:
//			{
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 24;
//				EEPROM_WriteByte(MAX_Correct_Addr+153, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 16;
//				EEPROM_WriteByte(MAX_Correct_Addr+154, data_8bit);
//				data_8bit = Run_Control[UART_Buffer_Rece[3]] >> 8;
//				EEPROM_WriteByte(MAX_Correct_Addr+155, data_8bit);
//				EEPROM_WriteByte(MAX_Correct_Addr+156, Run_Control[UART_Buffer_Rece[3]]);
//			}
//			break;
			default:  
			break;
			
		}
		
	}
//	else if(UART_Buffer_Rece[1] ==0x10)
//	{
//		for(var8=0;var8<UART_Buffer_Rece[6];var8++)
//		{
//			/***********************存储多个寄存器****************************/
//			data_8bit = Run_Control[UART_Buffer_Rece[3]+var8] >> 24;
//			EEPROM_WriteByte(((UART_Buffer_Rece[3]+var8)+MAX_Correct_Addr), data_8bit);
//			
//			data_8bit = Run_Control[(UART_Buffer_Rece[3]+var8)] >> 16;
//			EEPROM_WriteByte(((UART_Buffer_Rece[3]+var8)+MAX_Correct_Addr)+1, data_8bit);
//			
//			data_8bit = Run_Control[(UART_Buffer_Rece[3]+var8)] >> 8;
//			EEPROM_WriteByte(((UART_Buffer_Rece[3]+var8)+MAX_Correct_Addr)+2, data_8bit);
//			
//			EEPROM_WriteByte(((UART_Buffer_Rece[3]+var8)+MAX_Correct_Addr)+3, Run_Control[(UART_Buffer_Rece[3]+var8)]);
//		}
//	}
}


void Read_Runcont(void)
{
//	Run_Control[12]=EEPROM_READ_Byte((MAX_Correct_Addr+1));
//	Run_Control[12]=Run_Control[12]<<8;
//	Run_Control[12]=Run_Control[12]+EEPROM_READ_Byte((MAX_Correct_Addr+2));
//	Run_Control[12]=Run_Control[12]<<8;
//	Run_Control[12]=Run_Control[12]+EEPROM_READ_Byte((MAX_Correct_Addr+3));
//	Run_Control[12]=Run_Control[12]<<8;
//	Run_Control[12]=Run_Control[12]+EEPROM_READ_Byte((MAX_Correct_Addr+4));
	
	Run_Control[13]=EEPROM_READ_Byte((MAX_Correct_Addr+5));
	Run_Control[13]=Run_Control[13]<<8;
	Run_Control[13]=Run_Control[13]+EEPROM_READ_Byte((MAX_Correct_Addr+6));
	Run_Control[13]=Run_Control[13]<<8;
	Run_Control[13]=Run_Control[13]+EEPROM_READ_Byte((MAX_Correct_Addr+7));
	Run_Control[13]=Run_Control[13]<<8;
	Run_Control[13]=Run_Control[13]+EEPROM_READ_Byte((MAX_Correct_Addr+8));
	
	Run_Control[14]=EEPROM_READ_Byte((MAX_Correct_Addr+9));
	Run_Control[14]=Run_Control[14]<<8;
	Run_Control[14]=Run_Control[14]+EEPROM_READ_Byte((MAX_Correct_Addr+10));
	Run_Control[14]=Run_Control[14]<<8;
	Run_Control[14]=Run_Control[14]+EEPROM_READ_Byte((MAX_Correct_Addr+11));
	Run_Control[14]=Run_Control[14]<<8;
	Run_Control[14]=Run_Control[14]+EEPROM_READ_Byte((MAX_Correct_Addr+12));
	
	Run_Control[15]=EEPROM_READ_Byte((MAX_Correct_Addr+13));
	Run_Control[15]=Run_Control[15]<<8;
	Run_Control[15]=Run_Control[15]+EEPROM_READ_Byte((MAX_Correct_Addr+14));
	Run_Control[15]=Run_Control[15]<<8;
	Run_Control[15]=Run_Control[15]+EEPROM_READ_Byte((MAX_Correct_Addr+15));
	Run_Control[15]=Run_Control[15]<<8;
	Run_Control[15]=Run_Control[15]+EEPROM_READ_Byte((MAX_Correct_Addr+16));
	
	Run_Control[16]=EEPROM_READ_Byte((MAX_Correct_Addr+17));
	Run_Control[16]=Run_Control[16]<<8;
	Run_Control[16]=Run_Control[16]+EEPROM_READ_Byte((MAX_Correct_Addr+18));
	Run_Control[16]=Run_Control[16]<<8;
	Run_Control[16]=Run_Control[16]+EEPROM_READ_Byte((MAX_Correct_Addr+19));
	Run_Control[16]=Run_Control[16]<<8;
	Run_Control[16]=Run_Control[16]+EEPROM_READ_Byte((MAX_Correct_Addr+20));
	
	Run_Control[17]=EEPROM_READ_Byte((MAX_Correct_Addr+21));
	Run_Control[17]=Run_Control[17]<<8;
	Run_Control[17]=Run_Control[17]+EEPROM_READ_Byte((MAX_Correct_Addr+22));
	Run_Control[17]=Run_Control[17]<<8;
	Run_Control[17]=Run_Control[17]+EEPROM_READ_Byte((MAX_Correct_Addr+23));
	Run_Control[17]=Run_Control[17]<<8;
	Run_Control[17]=Run_Control[17]+EEPROM_READ_Byte((MAX_Correct_Addr+24));
	
	Run_Control[18]=EEPROM_READ_Byte((MAX_Correct_Addr+25));
	Run_Control[18]=Run_Control[18]<<8;
	Run_Control[18]=Run_Control[18]+EEPROM_READ_Byte((MAX_Correct_Addr+26));
	Run_Control[18]=Run_Control[18]<<8;
	Run_Control[18]=Run_Control[18]+EEPROM_READ_Byte((MAX_Correct_Addr+27));
	Run_Control[18]=Run_Control[18]<<8;
	Run_Control[18]=Run_Control[18]+EEPROM_READ_Byte((MAX_Correct_Addr+28));
	
	Run_Control[19]=EEPROM_READ_Byte((MAX_Correct_Addr+29));
	Run_Control[19]=Run_Control[19]<<8;
	Run_Control[19]=Run_Control[19]+EEPROM_READ_Byte((MAX_Correct_Addr+30));
	Run_Control[19]=Run_Control[19]<<8;
	Run_Control[19]=Run_Control[19]+EEPROM_READ_Byte((MAX_Correct_Addr+31));
	Run_Control[19]=Run_Control[19]<<8;
	Run_Control[19]=Run_Control[19]+EEPROM_READ_Byte((MAX_Correct_Addr+32));
	
	Run_Control[20]=EEPROM_READ_Byte((MAX_Correct_Addr+33));
	Run_Control[20]=Run_Control[20]<<8;
	Run_Control[20]=Run_Control[20]+EEPROM_READ_Byte((MAX_Correct_Addr+34));
	Run_Control[20]=Run_Control[20]<<8;
	Run_Control[20]=Run_Control[20]+EEPROM_READ_Byte((MAX_Correct_Addr+35));
	Run_Control[20]=Run_Control[20]<<8;
	Run_Control[20]=Run_Control[20]+EEPROM_READ_Byte((MAX_Correct_Addr+36));
	
	Run_Control[21]=EEPROM_READ_Byte((MAX_Correct_Addr+37));
	Run_Control[21]=Run_Control[21]<<8;
	Run_Control[21]=Run_Control[21]+EEPROM_READ_Byte((MAX_Correct_Addr+38));
	Run_Control[21]=Run_Control[21]<<8;
	Run_Control[21]=Run_Control[21]+EEPROM_READ_Byte((MAX_Correct_Addr+39));
	Run_Control[21]=Run_Control[21]<<8;
	Run_Control[21]=Run_Control[21]+EEPROM_READ_Byte((MAX_Correct_Addr+40));
	
	Run_Control[22]=EEPROM_READ_Byte((MAX_Correct_Addr+41));
	Run_Control[22]=Run_Control[22]<<8;
	Run_Control[22]=Run_Control[22]+EEPROM_READ_Byte((MAX_Correct_Addr+42));
	Run_Control[22]=Run_Control[22]<<8;
	Run_Control[22]=Run_Control[22]+EEPROM_READ_Byte((MAX_Correct_Addr+43));
	Run_Control[22]=Run_Control[22]<<8;
	Run_Control[22]=Run_Control[22]+EEPROM_READ_Byte((MAX_Correct_Addr+44));
	
	Run_Control[23]=EEPROM_READ_Byte((MAX_Correct_Addr+45));
	Run_Control[23]=Run_Control[23]<<8;
	Run_Control[23]=Run_Control[23]+EEPROM_READ_Byte((MAX_Correct_Addr+46));
	Run_Control[23]=Run_Control[23]<<8;
	Run_Control[23]=Run_Control[23]+EEPROM_READ_Byte((MAX_Correct_Addr+47));
	Run_Control[23]=Run_Control[23]<<8;
	Run_Control[23]=Run_Control[23]+EEPROM_READ_Byte((MAX_Correct_Addr+48));
	
	Run_Control[24]=EEPROM_READ_Byte((MAX_Correct_Addr+49));
	Run_Control[24]=Run_Control[24]<<8;
	Run_Control[24]=Run_Control[24]+EEPROM_READ_Byte((MAX_Correct_Addr+50));
	Run_Control[24]=Run_Control[24]<<8;
	Run_Control[24]=Run_Control[24]+EEPROM_READ_Byte((MAX_Correct_Addr+51));
	Run_Control[24]=Run_Control[24]<<8;
	Run_Control[24]=Run_Control[24]+EEPROM_READ_Byte((MAX_Correct_Addr+52));
	
	Run_Control[25]=EEPROM_READ_Byte((MAX_Correct_Addr+53));
	Run_Control[25]=Run_Control[25]<<8;
	Run_Control[25]=Run_Control[25]+EEPROM_READ_Byte((MAX_Correct_Addr+54));
	Run_Control[25]=Run_Control[25]<<8;
	Run_Control[25]=Run_Control[25]+EEPROM_READ_Byte((MAX_Correct_Addr+55));
	Run_Control[25]=Run_Control[25]<<8;
	Run_Control[25]=Run_Control[25]+EEPROM_READ_Byte((MAX_Correct_Addr+56));
	
	Run_Control[26]=EEPROM_READ_Byte((MAX_Correct_Addr+57));
	Run_Control[26]=Run_Control[26]<<8;
	Run_Control[26]=Run_Control[26]+EEPROM_READ_Byte((MAX_Correct_Addr+58));
	Run_Control[26]=Run_Control[26]<<8;
	Run_Control[26]=Run_Control[26]+EEPROM_READ_Byte((MAX_Correct_Addr+59));
	Run_Control[26]=Run_Control[26]<<8;
	Run_Control[26]=Run_Control[26]+EEPROM_READ_Byte((MAX_Correct_Addr+60));
	
	Run_Control[27]=EEPROM_READ_Byte((MAX_Correct_Addr+61));
	Run_Control[27]=Run_Control[27]<<8;
	Run_Control[27]=Run_Control[27]+EEPROM_READ_Byte((MAX_Correct_Addr+62));
	Run_Control[27]=Run_Control[27]<<8;
	Run_Control[27]=Run_Control[27]+EEPROM_READ_Byte((MAX_Correct_Addr+63));
	Run_Control[27]=Run_Control[27]<<8;
	Run_Control[27]=Run_Control[27]+EEPROM_READ_Byte((MAX_Correct_Addr+64));
	
	Run_Control[28]=EEPROM_READ_Byte((MAX_Correct_Addr+65));
	Run_Control[28]=Run_Control[28]<<8;
	Run_Control[28]=Run_Control[28]+EEPROM_READ_Byte((MAX_Correct_Addr+66));
	Run_Control[28]=Run_Control[28]<<8;
	Run_Control[28]=Run_Control[28]+EEPROM_READ_Byte((MAX_Correct_Addr+67));
	Run_Control[28]=Run_Control[28]<<8;
	Run_Control[28]=Run_Control[28]+EEPROM_READ_Byte((MAX_Correct_Addr+68));
	
	Run_Control[29]=EEPROM_READ_Byte((MAX_Correct_Addr+69));
	Run_Control[29]=Run_Control[29]<<8;
	Run_Control[29]=Run_Control[29]+EEPROM_READ_Byte((MAX_Correct_Addr+70));
	Run_Control[29]=Run_Control[29]<<8;
	Run_Control[29]=Run_Control[29]+EEPROM_READ_Byte((MAX_Correct_Addr+71));
	Run_Control[29]=Run_Control[29]<<8;
	Run_Control[29]=Run_Control[29]+EEPROM_READ_Byte((MAX_Correct_Addr+72));
	
	Run_Control[30]=EEPROM_READ_Byte((MAX_Correct_Addr+73));
	Run_Control[30]=Run_Control[30]<<8;
	Run_Control[30]=Run_Control[30]+EEPROM_READ_Byte((MAX_Correct_Addr+74));
	Run_Control[30]=Run_Control[30]<<8;
	Run_Control[30]=Run_Control[30]+EEPROM_READ_Byte((MAX_Correct_Addr+75));
	Run_Control[30]=Run_Control[30]<<8;
	Run_Control[30]=Run_Control[30]+EEPROM_READ_Byte((MAX_Correct_Addr+76));
	
	Run_Control[31]=EEPROM_READ_Byte((MAX_Correct_Addr+77));
	Run_Control[31]=Run_Control[31]<<8;
	Run_Control[31]=Run_Control[31]+EEPROM_READ_Byte((MAX_Correct_Addr+78));
	Run_Control[31]=Run_Control[31]<<8;
	Run_Control[31]=Run_Control[31]+EEPROM_READ_Byte((MAX_Correct_Addr+79));
	Run_Control[31]=Run_Control[31]<<8;
	Run_Control[31]=Run_Control[31]+EEPROM_READ_Byte((MAX_Correct_Addr+80));
	
	Run_Control[32]=EEPROM_READ_Byte((MAX_Correct_Addr+81));
	Run_Control[32]=Run_Control[32]<<8;
	Run_Control[32]=Run_Control[32]+EEPROM_READ_Byte((MAX_Correct_Addr+82));
	Run_Control[32]=Run_Control[32]<<8;
	Run_Control[32]=Run_Control[32]+EEPROM_READ_Byte((MAX_Correct_Addr+83));
	Run_Control[32]=Run_Control[32]<<8;
	Run_Control[32]=Run_Control[32]+EEPROM_READ_Byte((MAX_Correct_Addr+84));
	
	Run_Control[33]=EEPROM_READ_Byte((MAX_Correct_Addr+85));
	Run_Control[33]=Run_Control[33]<<8;
	Run_Control[33]=Run_Control[33]+EEPROM_READ_Byte((MAX_Correct_Addr+86));
	Run_Control[33]=Run_Control[33]<<8;
	Run_Control[33]=Run_Control[33]+EEPROM_READ_Byte((MAX_Correct_Addr+87));
	Run_Control[33]=Run_Control[33]<<8;
	Run_Control[33]=Run_Control[33]+EEPROM_READ_Byte((MAX_Correct_Addr+88));
	
	Run_Control[34]=EEPROM_READ_Byte((MAX_Correct_Addr+89));
	Run_Control[34]=Run_Control[34]<<8;
	Run_Control[34]=Run_Control[34]+EEPROM_READ_Byte((MAX_Correct_Addr+90));
	Run_Control[34]=Run_Control[34]<<8;
	Run_Control[34]=Run_Control[34]+EEPROM_READ_Byte((MAX_Correct_Addr+91));
	Run_Control[34]=Run_Control[34]<<8;
	Run_Control[34]=Run_Control[34]+EEPROM_READ_Byte((MAX_Correct_Addr+92));
	
	Run_Control[35]=EEPROM_READ_Byte((MAX_Correct_Addr+93));
	Run_Control[35]=Run_Control[35]<<8;
	Run_Control[35]=Run_Control[35]+EEPROM_READ_Byte((MAX_Correct_Addr+94));
	Run_Control[35]=Run_Control[35]<<8;
	Run_Control[35]=Run_Control[35]+EEPROM_READ_Byte((MAX_Correct_Addr+95));
	Run_Control[35]=Run_Control[35]<<8;
	Run_Control[35]=Run_Control[35]+EEPROM_READ_Byte((MAX_Correct_Addr+96));
	
	Run_Control[36]=EEPROM_READ_Byte((MAX_Correct_Addr+97));
	Run_Control[36]=Run_Control[36]<<8;
	Run_Control[36]=Run_Control[36]+EEPROM_READ_Byte((MAX_Correct_Addr+98));
	Run_Control[36]=Run_Control[36]<<8;
	Run_Control[36]=Run_Control[36]+EEPROM_READ_Byte((MAX_Correct_Addr+99));
	Run_Control[36]=Run_Control[36]<<8;
	Run_Control[36]=Run_Control[36]+EEPROM_READ_Byte((MAX_Correct_Addr+100));
	
	Run_Control[37]=EEPROM_READ_Byte((MAX_Correct_Addr+101));
	Run_Control[37]=Run_Control[37]<<8;
	Run_Control[37]=Run_Control[37]+EEPROM_READ_Byte((MAX_Correct_Addr+102));
	Run_Control[37]=Run_Control[37]<<8;
	Run_Control[37]=Run_Control[37]+EEPROM_READ_Byte((MAX_Correct_Addr+103));
	Run_Control[37]=Run_Control[37]<<8;
	Run_Control[37]=Run_Control[37]+EEPROM_READ_Byte((MAX_Correct_Addr+104));
	
//	for(var8=0;var8<32;var8++)
//	{
//		Run_Control[var8+12]=EEPROM_READ_Byte((MAX_Correct_Addr+1)+var8);
//		Run_Control[var8+12]=Run_Control[var8+12]<<8;
//		Run_Control[var8+12]=Run_Control[var8+12]+EEPROM_READ_Byte((MAX_Correct_Addr+2)+var8);
//		Run_Control[var8+12]=Run_Control[var8+12]<<8;
//		Run_Control[var8+12]=Run_Control[var8+12]+EEPROM_READ_Byte((MAX_Correct_Addr+3)+var8);
//		Run_Control[var8+12]=Run_Control[var8+12]<<8;
//		Run_Control[var8+12]=Run_Control[var8+12]+EEPROM_READ_Byte((MAX_Correct_Addr+4)+var8);
//	}
}
void READ_TSET(void)
{
	
	REND_TSETING=EEPROM_READ_Byte((MAX_Correct_Addr+12)+0);
	REND_TSETING=REND_TSETING<<8;
	REND_TSETING=REND_TSETING+EEPROM_READ_Byte((MAX_Correct_Addr+12)+1);
	REND_TSETING=REND_TSETING<<8;
	REND_TSETING=REND_TSETING+EEPROM_READ_Byte((MAX_Correct_Addr+12)+2);
	REND_TSETING=REND_TSETING<<8;
	REND_TSETING=REND_TSETING+EEPROM_READ_Byte((MAX_Correct_Addr+12)+3);
}
/******************* (C) COPYRIGHT 2015 AVER *****END OF FILE*************************/