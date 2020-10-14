/******************** (C) COPYRIGHT 2019 AVER********************
 * 文件名  :MODBUS.C
 * 作者   :
 * 描述    :MODBUS?????
 * 内容    :KL220A_Master
 * 硬件连接: 485
 * 修改日期:2014-12-22
********************************************************************/
#include "my_register.h" 
#include "usart.h" 
#include "modbus.h" 
#include "stm32f10x.h"
#include "flash.h"
//===================================================================//
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
} flagA,flagB,flagC,flagD,flagE;
/*************************校准参数************************************/
vu8 DAC_Flag;//DAC是否加载标志
vu32 Modify_A_READ;
vu32 Modify_C_READ;
vu32 Modify_A_ACT;
	
vu32 Modify_B_READ;
vu32 Modify_D_READ;
vu32 Modify_B_ACT;

vu16 Contr_DACVlue;//DAC加载值
vu32 Correct_Parametet[12];//校准参数
vu32 Correct_Strong[12];//校准系数
vu8 coefficient[8];
/*************************变量定义***********************************/
vu32 Run_Control[50];//运行数据寄存器
vu32 Transition_Date[10];//过渡变量
//============================函数声明===============================//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
//===========================MODBUS协议=============================//
void UART_Action(void)
{//RUT格式：
	//ADDR  命令码  读寄存器的起始地址高   读寄存器的起始地址低  读数据字个数高字节   读数据个数低字节  CRC高 CRC低
	//返回格式：ADDR 命令码 返回数据字节数  数据高  数据低 ..... CRC高  CRC低
	if ((UART_Buffer_Rece[0] == ADDR)||(UART_Buffer_Rece[0] == 0))
	{
		if (UART_Buffer_Rece[1] == (0x03))	//命令3 读数据   
		{																		 
			vu8 i;
			vu16 crc_result;
			crc_result = (UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];
			if ((crc_result == Hardware_CRC(UART_Buffer_Rece,6)) ||(crc_result == 0) )
			{
				if (UART_Buffer_Rece[3] < 0xFF)    								//如果寄存器在可读范围内
				{
					if ((UART_Buffer_Rece[3] + UART_Buffer_Rece[5]) < 0xFF)		//如果最后一个读取的寄存器地址在可读范围内
					{							
						UART_Buffer_Send[0] = ADDR;
						UART_Buffer_Send[1] = 0x03;
						UART_Buffer_Send[2] = UART_Buffer_Rece[5]*4;
						for (i=0;i<UART_Buffer_Send[2];i++)
						{
							if ((i % 4) == 0) 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 24;//高位在前
							}
							else if((i % 4) == 1)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 16;//高位在前
							}
							else if((i % 4) == 2)
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4] >> 8;//高位在前
							}
							else 
							{
								UART_Buffer_Send[3 + i] = Run_Control[UART_Buffer_Rece[3] + i / 4];			
							}															
						}
						crc_result = Hardware_CRC(UART_Buffer_Send,UART_Buffer_Send[2] + 3);
						UART_Buffer_Send[3 + UART_Buffer_Send[2]] = crc_result >> 8;
						UART_Buffer_Send[4 + UART_Buffer_Send[2]] = crc_result;
						Transmit_BUFFERsize = UART_Buffer_Send[2] + 5;
						UART_SEND_flag=1;
					}
				}
			}	
		}
	} 
//===============================写寄存器=================================
	if ((UART_Buffer_Rece[0] == ADDR) || (UART_Buffer_Rece[0] == 0))	 
	{
		vu8 var8;
		vu8 a=0;
		vu16 var16;
		vu16 crc_result;
		vu32 var32;
//=========================以下命令6 写单个寄存器===========================
		if (UART_Buffer_Rece[1] == 0X06)                                 //判断第二个字节是否为命令6
		{
			if (UART_Buffer_Rece[3] < 0xFF)							  //判断需要写的地址是否在可写范围内
			{
				crc_result = (UART_Buffer_Rece[8] << 8) + UART_Buffer_Rece[9];
				if ((crc_result == Hardware_CRC(UART_Buffer_Rece,8)) ||(crc_result == 0) )		  //检查CRC
				{
					var32 = (UART_Buffer_Rece[4] << 8) + UART_Buffer_Rece[5];	//第5 6个字节为要写入的数据
					var16=(UART_Buffer_Rece[6] << 8) + UART_Buffer_Rece[7];	//第6  7个字节为要写入的数据
					var8 = UART_Buffer_Rece[3];	        						//第3 4个字节为要写入的地址
					var32=(var32<<16)+var16;
					Run_Control[var8] = var32;			    //将数据写入指定的地址

					if (UART_Buffer_Rece[0] == ADDR)							//广播模式下不返回数据
					{
						for (a=0;a<10;a++)
						{UART_Buffer_Send[a] = UART_Buffer_Rece[a];}
						Transmit_BUFFERsize = 10;						//原样数据返回，不计算CRC
						UART_SEND_flag=1;
					}
				}
			}
		}
//=======================================以下是命令16，连写寄存器===========================================
//功能码16格式:
//     地址 命令 写入起始地址高  写入起始地址低 写入字数高  写入字数低 写入寄存器个数  数据高 数据低 ......CRC高 CRC低
//返回数据格式:
//     地址 命令 写入起始地址高  写入起始地址低  写入字节数高 写入字节数低  CRC高  CRC低 
		if (UART_Buffer_Rece[1] == 0X10)										  
		{	
			crc_result = ((UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+7]) << 8) + UART_Buffer_Rece[(UART_Buffer_Rece[6]*4)+8];
			if (crc_result == Hardware_CRC(UART_Buffer_Rece,(UART_Buffer_Rece[6]*4)+7)) 	  //检查CRC
			{												
				for (var8=0;var8<UART_Buffer_Rece[6];var8++) 
				{
					var32 = (UART_Buffer_Rece[var8*4+7] << 8) + UART_Buffer_Rece[var8*4+8];
					var16=(UART_Buffer_Rece[var8*4+9] << 8) + UART_Buffer_Rece[var8*4+10];	//这里是写入的是32位寄存器所以是*4
					var32=(var32<<16)+var16;
					Run_Control[UART_Buffer_Rece[3]+var8] = var32;			    //将数据写入指定的地址
				}

				if (UART_Buffer_Rece[0] == ADDR)					  //广播模式不返回数据
				{
					UART_Buffer_Send[0] = ADDR;
					UART_Buffer_Send[1] = 16;
					UART_Buffer_Send[2] = UART_Buffer_Rece[2];
					UART_Buffer_Send[3] = UART_Buffer_Rece[3];
					UART_Buffer_Send[4] = UART_Buffer_Rece[4];
					UART_Buffer_Send[5] = UART_Buffer_Rece[5];
					crc_result = Hardware_CRC(UART_Buffer_Send,6);	 //计算CRC码
					UART_Buffer_Send[6] = crc_result>>8;
					UART_Buffer_Send[7] = crc_result;				 
					Transmit_BUFFERsize = 8;					     //设置发送字节数长度
					UART_SEND_flag=1;
				}
			}
		}			 
	}
/*************************************以下为校准部分**************************************************************************/
	if ((UART_Buffer_Rece[0] == 0x01)&&(UART_Buffer_Rece[2] == 0xA5))	 
	{
		vu16 crc_result;
		crc_result = (UART_Buffer_Rece[8] << 8) + UART_Buffer_Rece[9];
	  if (crc_result == Hardware_CRC(UART_Buffer_Rece,8))
		{
	/*******************电压测量低档校准**************************************/	
			if(UART_Buffer_Rece[1] == 0x01)
			{
				Modify_A_READ = Vmon_value;//测量电压值
				Modify_C_READ = Contr_DACVlue;//设置电压值
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
			}
			if (UART_Buffer_Rece[1] == 0x02)			   //电压测量校准完成
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_B_READ =Vmon_value;//测量电压值
				Modify_D_READ =Contr_DACVlue;//设置电压值
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionV_LOW = var32a;
				var32a=0;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionV_LOW;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_LOW = var32b;
					Polar0 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_LOW = var32a;
					Polar0 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CV模式电压控制校准
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_LOW = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_LOW;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_LOW = var32d;
					Polar0 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_LOW = var32c;
					Polar0 &= ~0x04;
				}
				Flash_Write_all();	//参数写进FLASH
				DAC_Flag=0;
			}
			/*******************电压测量高档位校准**************************************/	
			if(UART_Buffer_Rece[1] == 0x0C)
			{
				Modify_A_READ = Vmon_value;//测量电压值
				Modify_C_READ = Contr_DACVlue;//设置电压值
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
			}
			/*******************电压测量中段校准*************************/	
			if (UART_Buffer_Rece[1] == 0x04)			   
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_B_READ =Vmon_value;//测量电压值
				Modify_D_READ =Contr_DACVlue;//设置电压值
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionV_HIG = var32a;
				var32a=0;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionV_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_HIG = var32b;
					Polar1 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_HIG = var32a;
					Polar1 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_B_ACT; //CV模式电压控制校准
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_HIG = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_HIG;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_HIG = var32d;
					Polar1 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_HIG = var32c;
					Polar1 &= ~0x04;
				}
				Flash_Write_all();	//参数写进FLASH
			}
			/*******************电压测量和控制高段校准*****************/
			if (UART_Buffer_Rece[1] == 0x05)			   //电压测量校准完成
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_A_READ=0;
				Modify_C_READ=0;
				Modify_A_ACT=0;
				Modify_A_READ =Vmon_value;//测量电压值
				Modify_C_READ =Contr_DACVlue;//设置电压值
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionV_MID_HIG = var32a;
				var32a=0;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionV_MID_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadV_Offset_MID_HIG = var32b;
					Polar4 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadV_Offset_MID_HIG = var32a;
					Polar4 &= ~0x01;
				}
	//---------------------------------------------------------------------------------------//			
				var32c = Modify_A_ACT; //CV模式电压控制校准
				var32c = var32c - Modify_B_ACT;
				var32c = var32c << 12;
				var16a=Modify_C_READ-Modify_D_READ;
				var16a=(var16a*2);
				var32c=var32c/var16a;
				SET_CorrectionV_MID_HIG = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionV_MID_HIG;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadV_Offset_MID_HIG = var32d;
					Polar4 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadV_Offset_MID_HIG = var32c;
					Polar4 &= ~0x04;
				}
				Flash_Write_all();	//参数写进FLASH
				DAC_Flag=0;
			}
			
			/******************电流测量和控制低档校准********************/	
			if (UART_Buffer_Rece[1] == 0x06)			   //CC模式电流测量校准
			{
				Modify_A_READ = Imon_value;//测量电流
				Modify_C_READ = Contr_DACVlue;//设置电流
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];//读取高段
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
			}

			if (UART_Buffer_Rece[1] == 0x07)			   //电流测量校准完成
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_LOW = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_LOW;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_LOW = var32b;
					Polar2 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_LOW = var32a;
					Polar2 &= ~0x01;					//电流测量系数和零点校准完成
				}
				
				var32c = Modify_B_ACT; //设置电流校准
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_LOW = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_LOW;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_LOW = var32d;
					Polar2 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_LOW = var32c;
					Polar2 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
	/******************电流测量和控制中段校准*********************/
			if (UART_Buffer_Rece[1] == 0x08)			  
			{
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
			}

			if (UART_Buffer_Rece[1] == 0x09)			  
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_B_READ = Imon_value;
				Modify_D_READ = Contr_DACVlue;
				Modify_B_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_B_ACT=(Modify_B_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_B_ACT;
				var32a = var32a - Modify_A_ACT;
				var32a = var32a << 12;
				var16 = Modify_B_READ - Modify_A_READ;
				var32a = var32a / var16;
				REG_CorrectionA_HIG = var32a;
				var32a = Modify_B_ACT;
				var32a = var32a << 12;
				var32b = Modify_B_READ;
				var32b = var32b * REG_CorrectionA_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_HIG = var32b;
					Polar3 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_HIG = var32a;
					Polar3 &= ~0x01;						
				}
				var32c = Modify_B_ACT; 
				var32c = var32c - Modify_A_ACT;
				var32c = var32c << 12;
				var16a=Modify_D_READ-Modify_C_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_HIG = var32c;
				var32c = Modify_B_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_HIG;
				var32d = var32d * (Modify_D_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_HIG = var32d;
					Polar3 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_HIG = var32c;
					Polar3 &= ~0x04;
				}
				Flash_Write_all ();	
			}
			
			/****************电流测量和控制高段校准*****************/
			if (UART_Buffer_Rece[1] == 0x0A)			  
			{
				vu32 var16;
				vu32 var32a;
				vu32 var32b;
				
				vu32 var16a;
				vu32 var32c;
				vu32 var32d;
				
				Modify_A_READ = Imon_value;
				Modify_C_READ = Contr_DACVlue;
				Modify_A_ACT = (UART_Buffer_Rece[3] << 8) + UART_Buffer_Rece[4];
				Modify_A_ACT=(Modify_A_ACT<<16)+(UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];//读取低段
				var32a = Modify_A_ACT;
				var32a = var32a - Modify_B_ACT;
				var32a = var32a << 12;
				var16 = Modify_A_READ - Modify_B_READ;
				var32a = var32a / var16;
				REG_CorrectionA_MID_HIG = var32a;
				var32a = Modify_A_ACT;
				var32a = var32a << 12;
				var32b = Modify_A_READ;
				var32b = var32b * REG_CorrectionA_MID_HIG;
				if (var32a < var32b)
				{
					var32b = var32b - var32a;
					REG_ReadA_Offset_MID_HIG = var32b;
					Polar5 |= 0x01;
				}
				else 
				{
					var32a = var32a - var32b;
					REG_ReadA_Offset_MID_HIG = var32a;
					Polar5 &= ~0x01;						
				}
				var32c = Modify_A_ACT; 
				var32c = var32c - Modify_B_ACT;
				var32c = var32c << 12;
				var16a=Modify_C_READ-Modify_D_READ;
				var16a=var16a*2;
				var32c=var32c/var16a;
				SET_CorrectionA_MID_HIG = var32c;
				var32c = Modify_A_ACT;
				var32c = var32c << 12;
				var32d = SET_CorrectionA_MID_HIG;
				var32d = var32d * (Modify_C_READ*2);
				if (var32c < var32d)
				{
					var32d = var32d - var32c;
					SET_ReadA_Offset_MID_HIG = var32d;
					Polar5 |= 0x04;
				}
				else 
				{
					var32c = var32c - var32d;
					SET_ReadA_Offset_MID_HIG = var32c;
					Polar5 &= ~0x04;
				}
				Flash_Write_all ();	
				DAC_Flag=0;
			}
	/********************接受DA值********************************/
			if (UART_Buffer_Rece[1] == 0x0B)//接受电压DAC值
			{
				DAC_Flag=0x01;
				Contr_DACVlue = (UART_Buffer_Rece[5] << 8) + UART_Buffer_Rece[6];
			}
		}
 }
}
//===============================AD值转换成测量值============================================//
void Transformation_ADC(void)  
{
	vu32 var32;
	vu32 var32a;
/*****************************测量电压转换*******************************************/
	if(V_Gear_SW==0)//电压低档位
	{
		var32 = Vmon_value;
		var32 = var32 * REG_CorrectionV_LOW;  //Y=KX+B 这里计算Y=KX
		if ((Polar0 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_LOW) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_LOW;
		}
		else var32 = var32 + REG_ReadV_Offset_LOW;
		var32 = var32 >> 12;
		if (var32 < 4) var32 = 0;				  //10mV以下清零
		Voltage = var32;
		var32 = 0;
		if(I_Gear_SW==0)//电流低档
		{
			Power_DATE=(Voltage/10)*(Current/100);//计算实时功率
			Power_DATE=Power_DATE/1000;//统一精确到小数点后3位
			R_DATE=(Voltage*10000/Current);//计算实时电阻  统一精确到小数点后3位
		}
		else//电流高档
		{
			Power_DATE=(Voltage/10)*(Current/10);//计算实时功率
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*1000/Current);//计算实时电阻
		}
	}
	else//电压高档位
	{
		var32 = Vmon_value;
		var32 = var32 * REG_CorrectionV_HIG;  //Y=KX+B 这里计算Y=KX
		if ((Polar1 & 0x01) == 0x01)		  //
		{
			if (var32 < REG_ReadV_Offset_HIG) 
			{
				var32 = 0;
			}
			else var32 = var32 - REG_ReadV_Offset_HIG;
		}
		else var32 = var32 + REG_ReadV_Offset_HIG;
		var32 = var32 >> 12;
		if (var32 < 4) var32 = 0;				  //10mV以下清零
		Voltage = var32;
		var32=0;
		if(Voltage>20000)
		{
			var32 = Vmon_value;
			var32 = var32 * REG_CorrectionV_MID_HIG;  //Y=KX+B 这里计算Y=KX
			if ((Polar4 & 0x01) == 0x01)		  //
			{
				if (var32 < REG_ReadV_Offset_MID_HIG) 
				{
					var32 = 0;
				}
				else var32 = var32 - REG_ReadV_Offset_MID_HIG;
			}
			else var32 = var32 + REG_ReadV_Offset_MID_HIG;
			var32 = var32 >> 12;
			if (var32 < 4) var32 = 0;				  //10mV以下清零
			Voltage = var32;
			var32 = 0;
		}
		if(I_Gear_SW==0)
		{
			Power_DATE=Voltage*(Current/100);//计算实时功率
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*100000/Current);//计算实时电阻
		}
		else
		{
			Power_DATE=Voltage*(Current/10);//计算实时功率
			Power_DATE=Power_DATE/1000;
			R_DATE=(Voltage*10000/Current);//计算实时电阻
		}
	}
/*****************************测量电流转换*******************************************/
	if(I_Gear_SW==0)//低档位
	{
		var32 = Imon_value;
		var32 = var32 * REG_CorrectionA_LOW;	   
		if ((Polar2 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_LOW) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_LOW;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_LOW;
		}	
		var32 = var32 >> 12;
		Current = var32;
		var32 = 0;
		if((Current<=1)||(onoff_ch==0)||(Voltage==0))
		{
			Current = 0;
		}
	}
	else//高档位
	{
		var32 = Imon_value;
		var32 = var32 * REG_CorrectionA_HIG;	   
		if ((Polar3 & 0x01) == 0x01)			   
		{
			if (var32 < REG_ReadA_Offset_HIG) var32 = 0;
			else var32 = var32 - REG_ReadA_Offset_HIG;
		}
		else
		{
			var32 = var32 + REG_ReadA_Offset_HIG;
		}	
		var32 = var32 >> 12;
		Current = var32;
		if(Current>20000)
		{
			var32=0;
			var32 = Imon_value;
			var32 = var32 * REG_CorrectionA_MID_HIG;	   
			if ((Polar5 & 0x01) == 0x01)			   
			{
				if (var32 < REG_ReadA_Offset_MID_HIG) var32 = 0;
				else var32 = var32 - REG_ReadA_Offset_MID_HIG;
			}
			else
			{
				var32 = var32 + REG_ReadA_Offset_MID_HIG;
			}	
			var32 = var32 >> 12;
			Current = var32;
		}
		if(onoff_ch==0||(Voltage==0))
		{
			Current=0;
		}
		var32 = 0;

	}
/**************************设置电压转换******************************************/
	if(MODE==1)//CV模式
	{
		if(SET_V_TRAN!=SET_Voltage)
		{
			TIME_1MS_OVER=0;
		}
		if(TIME_1MS_flag==1)//CV模式电压下降率调节
		{
			TIME_1MS_flag=0;
			SET_V_TRAN=SET_V_TRAN-CV_Down_Time;
			if(SET_V_TRAN<=SET_Voltage)
			{
				SET_V_TRAN=SET_Voltage;
				TIME_1MS_OVER=1;//结束爬升标志
			}
		} 
		if(V_Gear_SW==0)//低档位
		{ 
			var32 = SET_V_TRAN;
		  var32=var32<<12; 
			if ((Polar0 & 0x04) == 0)			   
			{
				if (var32 < SET_ReadV_Offset_LOW) var32 = 0;
				else var32 = var32 - SET_ReadV_Offset_LOW;
			}
			else var32 = var32 + SET_ReadV_Offset_LOW;
			var32 = var32/SET_CorrectionV_LOW;
			var32=var32>>1;
			Contr_DACVlue = var32;
			var32 = 0;
		}
		else//高档位
		{ 
			var32 = SET_V_TRAN;
			var32=var32<<12; 
			if(SET_Voltage<20000)
			{
				if ((Polar1 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_HIG;
				}
				else var32 = var32 + SET_ReadV_Offset_HIG;
				var32 = var32/SET_CorrectionV_HIG;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}
			else
			{
				if ((Polar4 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadV_Offset_MID_HIG) var32 = 0;
					else var32 = var32 - SET_ReadV_Offset_MID_HIG;
				}
				else var32 = var32 + SET_ReadV_Offset_MID_HIG;
				var32 = var32/SET_CorrectionV_MID_HIG;
				var32=var32>>1;
				Contr_DACVlue = var32;
				var32 = 0;
			}
		}
	}
/**************************设置电流转换**************************************/
	else//CC模式，CR,CP由CC模式模拟而来
	{
		if(I_Gear_SW==0)//低档
		{
			if(MODE==0)//CC模式
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_Current)
					{
						SET_I_TRAN=SET_Current;
					}
				}
				var32 = SET_I_TRAN;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			else if(MODE==2)//CR模式
			{
				if(V_Gear_SW==0)//电压低档模式时
				{
					SET_R_Current=(Voltage*1000)/SET_Resist;//电流低档模式CR模式精确到0.001
				}
				else//电压高档模式时
				{
					SET_R_Current=(Voltage*10000)/SET_Resist;//电流低档模式CR模式精确到0.001
				}
				var32 = SET_R_Current;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			else if(MODE==3)//CP模式
			{
				if(V_Gear_SW==0)
				{
					SET_P_Current=(SET_Power*100000)/(Voltage/10);//CP模式电流高档最小分辨率为0.001瓦
				}
				else
				{
					SET_P_Current=(SET_Power*100000)/Voltage;//CP模式电流高档最小分辨率为0.001瓦
				}
				var32 = SET_P_Current;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			else if(MODE==5)//LED模式 实际加载为CR模式  LED模式电压档位默认切换为高档位
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_R_Current)
					{
						SET_I_TRAN=SET_R_Current;
					}
				}
				var32 = SET_I_TRAN;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			else if(MODE==6)//短路模式
			{
				var32 = SET_S_Current;
				var32=var32<<12;   
				if ((Polar2 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_LOW) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_LOW;
				}
				else var32 = var32 + SET_ReadA_Offset_LOW;
				var32 = var32/SET_CorrectionA_LOW;
				var32=var32>>1;
			}
			Contr_DACVlue = var32;
			var32 = 0;
		}
		else//高档
		{
			if(MODE==0)//CC模式
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_Current)
					{
						SET_I_TRAN=SET_Current;
					}
				}
				var32 = SET_I_TRAN;
				if(SET_Current<50000)//分段控制，第一段
				{
					var32=var32<<12;   
					if ((Polar3 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_HIG;
					var32 = var32/SET_CorrectionA_HIG;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
				else//第二段
				{
					var32=var32<<12;   
					if ((Polar5 & 0x04) == 0)			   
					{
						if (var32 < SET_ReadA_Offset_MID_HIG) var32 = 0;
						else var32 = var32 - SET_ReadA_Offset_MID_HIG;
					}
					else var32 = var32 + SET_ReadA_Offset_MID_HIG;
					var32 = var32/SET_CorrectionA_MID_HIG;
					var32=var32>>1;
					if(SET_Current==0)
					{
						Contr_DACVlue=0;
					}
				}
			}
			else if(MODE==2)//CR模式
			{
				if(V_Gear_SW==0)
				{
					SET_R_Current=(Voltage*100)/SET_Resist;//CR模式电流高档最小分辨率为0.01欧
				}
				else
				{
					SET_R_Current=(Voltage*1000)/SET_Resist;//CR模式电流高档最小分辨率为0.01欧
				}
				var32 = SET_R_Current;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			else if(MODE==3)//CP模式
			{
				if(V_Gear_SW==0)
				{
					SET_P_Current=(SET_Power*10000)/(Voltage/10);//CP模式电流高档最小分辨率为0.01瓦
				}
				else
				{
					SET_P_Current=(SET_Power*10000)/Voltage;//CP模式电流高档最小分辨率为0.01瓦
				}
				var32 = SET_P_Current;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			else if(MODE==5)//LED模式 实际加载为CR模式  LED模式电压档位默认切换为高档位
			{
				if(TIME_1MS_flag==1)
				{
					SET_I_TRAN=SET_I_TRAN+I_Rise_Time;
					if(SET_I_TRAN>=SET_R_Current)
					{
						SET_I_TRAN=SET_R_Current;
					}
				}
				var32 = SET_I_TRAN;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			else if(MODE==6)//短路模式
			{
				var32 = SET_S_Current;
				var32=var32<<12;   
				if ((Polar3 & 0x04) == 0)			   
				{
					if (var32 < SET_ReadA_Offset_HIG) var32 = 0;
					else var32 = var32 - SET_ReadA_Offset_HIG;
				}
				else var32 = var32 + SET_ReadA_Offset_HIG;
				var32 = var32/SET_CorrectionA_HIG;
				var32=var32>>1;
			}
			Contr_DACVlue = var32;
			var32=0;
			if(SET_Current==0)
			{
				Contr_DACVlue=0;
			}
		}
	}
}
//-----------------------------CRC检测--------------------------------------------//
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count)    //CRC16
{
	vu16 CRC_Result=0xffff;
	vu8 i;
	if(count==0)
	{
		count=1;
	}
	while(count--)
	{
		CRC_Result^=*p_buffer;
		for(i=0;i<8;i++)
		{
			if(CRC_Result&1)
			{
				CRC_Result>>=1;
				CRC_Result^=0xA001;
			}
			else 
			{
				CRC_Result>>=1;
			}
		}
		p_buffer++;
	}
	return CRC_Result;
}
