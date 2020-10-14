/******************** (C) COPYRIGHT 2018 shenzhen********************
 * 名称    : _ME_SCPI.C
 * 作者    :  xq-guo
 * 描述    :
 * 软件功能:
 * 硬件外设: 通用串口SCPI协议
 * 修改日期: 2019-2-22
*******************************************************************/


#include "stm32f10x.h"
#include "me_scpi.h"
#include "my_register.h" 
#include "usart.h" 
#include "string.h" 
#include "stdlib.h" 
#include "stdio.h" 
#include "usart.h"
#include "sys_io_cfg.h"

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
vu16 Run_temp_contr[10];
vu8 Setcontr_ADDR;//设置参数存入FLASH位置
//-------------------------------------------------------------------------------
//--获取串口字符串中的设定的值 
//--将指定位置的字符串数字转化为10进制数字
//*p_buffer : 输入的数字字符串
//     fdr  : 数字在字符串中的首位置
//     b_x  : 就是高低档不同量程 放大倍数 ( 高档 1倍,  低档 10倍 )
//-------------------------------------------------------------------------------
vu32	Get_strNum( vu8 *p_buffer , vu8 fdr ,  vu8 b_x , vu32 max, vu32 min)
{		
		vu32 temp;
		
		if(b_x >10)  b_x=10; //最大10倍
		
		if( p_buffer[fdr +1] == '\n') //0\n  最大值为21.0000  11
		{
			temp =  ( (p_buffer[fdr ] - 0x30 ) )*10000 * b_x;	
		}	
		else if( p_buffer[fdr +2 ] == '\n') //00\n  最大值为21.0000 12
		{
			temp =  ( (p_buffer[ fdr  ] - 0x30 )*100000  * b_x+  (p_buffer[fdr+1  ] - 0x30 )*10000  * b_x);	
		}	
		else if( p_buffer[fdr +3 ] == '\n') //0.x\n  最大值为21.0000 13
		{
		 if( p_buffer[fdr +1 ] == '.')
		 {
				temp =  ( (p_buffer[fdr  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +2 ] - 0x30 )*1000  * b_x);	
		 }				
		}
		else if( p_buffer[fdr +4 ] == '\n') //0.xxA/00.x  最大值为21.0000 14
		{
			if( p_buffer[fdr +1 ] == '.')
			 {
					temp =  ( (p_buffer[fdr  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +2 ] - 0x30 )*1000  * b_x +  (p_buffer[fdr +3 ] - 0x30 )*100 * b_x  );
			 }	
			 else if( p_buffer[fdr +2 ] == '.')
			 {
					temp =  ( (p_buffer[fdr  ] - 0x30 )*100000 * b_x +  (p_buffer[fdr +1 ] - 0x30 )*10000  * b_x +  (p_buffer[fdr +3 ] - 0x30 )*1000 * b_x  );	
			 }			 
		}
		else if( p_buffer[fdr +5 ] == '\n') //0.xxxA 00.xx 最大值为21.0000 15
		{
			if( p_buffer[fdr +1 ] == '.')
			 {
					temp =  ( (p_buffer[fdr  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +2 ] - 0x30 )*1000  * b_x +  (p_buffer[fdr +3] - 0x30 )*100  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*10  * b_x);	
			 }
			 else if( p_buffer[fdr +2 ] == '.')
			 {
					temp =  ( (p_buffer[fdr  ] - 0x30 )*100000  * b_x+  (p_buffer[fdr +1  ] - 0x30 )*10000  * b_x +  (p_buffer[fdr +3 ] - 0x30 )*1000  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*100  * b_x);	
			 }
		}
		else if( p_buffer[fdr +6 ] == '\n') //0.xxxxA  00.xxx最大值为 21.0000 16
		{
			if( p_buffer[fdr +1 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +2 ] - 0x30 )*1000  * b_x+  (p_buffer[fdr +3 ] - 0x30 )*100  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*10  * b_x+  (p_buffer[fdr +5 ] - 0x30 ) * b_x );	
			}
			else if( p_buffer[fdr +2 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*100000  * b_x+  (p_buffer[fdr +1 ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +3 ] - 0x30 )*1000  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*100  * b_x+  (p_buffer[fdr +5 ] - 0x30 )*10  * b_x);	
			}
		}
		else if( p_buffer[fdr +7 ] == '\n') //00.xxxx  0.xxxxxA 最大值为 21.0000  17
		{
			if( p_buffer[fdr +2 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*100000  * b_x+  (p_buffer[fdr+1  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +3 ] - 0x30 )*1000  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*100  * b_x+  (p_buffer[fdr +5 ] - 0x30 )*10  * b_x+  (p_buffer[fdr +6 ] - 0x30 )  * b_x);	
			}
			else if( p_buffer[fdr +1 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*100000 +  (p_buffer[fdr+2  ] - 0x30 )*10000  +  (p_buffer[fdr +3 ] - 0x30 )*1000  +  (p_buffer[fdr +4 ] - 0x30 )*100  +  (p_buffer[fdr +5 ] - 0x30 ) *10 +  (p_buffer[fdr +6 ] - 0x30 ));	
			}
		}
		else if( p_buffer[fdr +8 ] == '\n') //00.xxxxX  0.xxxxxXA 无效值
		{
			if( p_buffer[fdr +2 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*100000  * b_x+  (p_buffer[fdr+1  ] - 0x30 )*10000  * b_x+  (p_buffer[fdr +3 ] - 0x30 )*1000  * b_x+  (p_buffer[fdr +4 ] - 0x30 )*100  * b_x+  (p_buffer[fdr +5 ] - 0x30 )*10  * b_x+  (p_buffer[fdr +6 ] - 0x30 )  * b_x);	
			}
			else if( p_buffer[fdr +1 ] == '.')
			{				 
				temp =  ( (p_buffer[fdr  ] - 0x30 )*100000 +  (p_buffer[fdr+2  ] - 0x30 )*10000  +  (p_buffer[fdr +3 ] - 0x30 )*1000  +  (p_buffer[fdr +4 ] - 0x30 )*100  +  (p_buffer[fdr +5 ] - 0x30 ) *10 +  (p_buffer[fdr +6 ] - 0x30 ));	
			}
		}
		if( temp > max)
			temp =  max;
		else if( temp < min)
			temp =  min;	
		return ( temp ); 
}			


//-------------------------------------------------------------------------------
//--获取串口字符串中的设定的值 
//--将指定位置的字符串数字转化为10进制数字
//*p_buffer : 输入的数字字符串
//     fdr  : 数字在字符串中的首位置
//     b_x  : 就是高低档不同量程 放大倍数 ( 高档 1倍,  低档 10倍 ) 用于转换电压值
//-------------------------------------------------------------------------------
vu32 Get_strNumV(vu8 *p_buffer , vu8 fdr ,  vu8 b_x , vu32 max, vu32 min)
{
	vu32 temp;
	
	if( p_buffer[ fdr + 1 ] == '\n' ) //0\n  最大值为 150.000 
	{
		temp =  ( (p_buffer[ fdr ] - 0x30 ) )*1000 *b_x;	
	}	
	else if( p_buffer[ fdr + 2 ] == '\n') //00\n  最大值为 150.000 
	{
		temp =  ( (p_buffer[fdr ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*1000 *b_x;	
	}	
	else if( p_buffer[ fdr + 3] == '\n') //000 0.0\n  最大值为 150.000 
	{
		if( p_buffer[ fdr + 1 ] ==  '.') //0.0
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*100 *b_x;	
		}
		else  //000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*1000 *b_x;	
		}
	}	
	else if( p_buffer[ fdr + 4] == '\n') //0.00 / 00.0\n  最大值为 150.000 
	{
		if( p_buffer[ fdr + 1 ] ==  '.') //0.00
		{
			temp =  ( (p_buffer[fdr ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 3 ] - 0x30 ) )*10*b_x;	
		}
		else  if( p_buffer[ fdr + 2] ==  '.')//00.0
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 3 ] - 0x30 ) )*100*b_x;	
		}
	}
	else if( p_buffer[ fdr + 5 ] == '\n') //000.0 / 00.00 / 0.000\n  最大值为 150.000 
	{
		if( p_buffer[ fdr + 1 ] ==  '.') //0.000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 3 ] - 0x30 ) )*10 *b_x+  ( (p_buffer[  fdr + 4 ] - 0x30 ) )*b_x;	
		}
		else  if( p_buffer[ fdr + 2 ] ==  '.')//00.00
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 3 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 4 ] - 0x30 ) )*10 *b_x;	
		}
		else  if( p_buffer[ fdr + 3 ] ==  '.')//000.0  
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 4 ] - 0x30 ) )*100 *b_x ;	
		}
	}
	else if( p_buffer[ fdr + 6] == '\n') //000.00 / 00.000 / 0.0000 \n  最大值为 150.000 
	{
		if( p_buffer[ fdr + 1 ] ==  '.') //0.0000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 2 ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[ fdr + 3 ] - 0x30 ) )*100 *b_x +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*10 *b_x+  ( (p_buffer[fdr + 5 ] - 0x30 ) ) *b_x;	
		}
		else  if( p_buffer[ fdr + 2] ==  '.')//00.000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*1000 *b_x+  ( (p_buffer[fdr + 3 ] - 0x30 ) )*100 *b_x +  ( (p_buffer[fdr + 4 ] - 0x30 ) )*10 *b_x+  ( (p_buffer[fdr + 5 ] - 0x30 ) ) *b_x;	
		}
		else  if( p_buffer[ fdr + 3] ==  '.')//000.00  
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 *b_x+  ( (p_buffer[fdr + 1  ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[ fdr + 2 ] - 0x30 ) )*1000 *b_x +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 5 ] - 0x30 ) )*10 *b_x;	
		}
	}
	else if( p_buffer[ fdr + 7 ] == '\n') //000.000 / 00.0000  \n  最大值为 150.000 
	{
		if( p_buffer[ fdr + 3 ] ==  '.') //000.000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 *b_x+  ( (p_buffer[ fdr + 1 ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[ fdr + 2  ] - 0x30 ) )*1000 *b_x +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 5 ] - 0x30 )*10 *b_x) +  ( (p_buffer[fdr + 6 ] - 0x30 ))*b_x;	
		}
		else  if( p_buffer[ fdr + 2] ==  '.')//00.0000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 +  ( (p_buffer[ fdr + 1  ] - 0x30 ) )*10000 +  ( (p_buffer[ fdr + 3  ] - 0x30 ) )*1000 +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*100 +  ( (p_buffer[fdr + 5  ] - 0x30 )*10 )  +  ( (p_buffer[fdr + 6 ] - 0x30 ));	
		}
	}
	else  //000.000xx. / 00.0000xx.  \n  最大值为 150.000 多发位数只取前面有效几位
	{
		if( p_buffer[ fdr + 3 ] ==  '.') //000.000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 *b_x+  ( (p_buffer[ fdr + 1 ] - 0x30 ) )*10000 *b_x+  ( (p_buffer[ fdr + 2  ] - 0x30 ) )*1000 *b_x +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*100 *b_x+  ( (p_buffer[fdr + 5 ] - 0x30 )*10 *b_x) +  ( (p_buffer[fdr + 6 ] - 0x30 ))*b_x;	
		}
		else  if( p_buffer[ fdr + 2] ==  '.')//00.0000
		{
			temp =  ( (p_buffer[fdr  ] - 0x30 ) )*100000 +  ( (p_buffer[ fdr + 1  ] - 0x30 ) )*10000 +  ( (p_buffer[ fdr + 3  ] - 0x30 ) )*1000 +  ( (p_buffer[ fdr + 4 ] - 0x30 ) )*100 +  ( (p_buffer[fdr + 5  ] - 0x30 )*10 )  +  ( (p_buffer[fdr + 6 ] - 0x30 ));	
		}
	}
	if( temp > max)
		temp =  max;
	else if( temp < min)
		temp =  min;	
	return ( temp ); 
			
}

//-A 共同命令---------------------------------------------------------------------

/*--清除寄存器--------------------------------------------------------*/
vu8 CLS( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*CLS\n" ,5)==0 )
	{
		StandEventREG=0;
		QuestEventREG=0;
		OpertEventREG=0;
		BitCropeREG=0;
		ErrorCodeREG=0;
		return 1;
	}
	else 
	 return 0;
}
/*--编辑标准事件使能寄存器--------------------------------------------------------*/
vu8 ESE( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*ESE " ,5)==0 )
	{
		StandEventREG = atoi( (const char *)&p_buffer[5] );//设定的值赋值到寄存器
		return 1;
	}
	else 
	 return 0;
}

/*--读取标准事件寄存器值-------------------------------------------------------*/
vu8 QUESE( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*ESE?\n" ,6)==0 )
	{		
		printf("%d\r\n", StandEventREG);
		return 1;	
	}
	return 0;	
}

/*--查询仪器相关信息-------------------------------------------------------*/
vu8 IDN( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*IDN?\n" ,6)==0 )
	{			
		printf("AVER Electronics,A.01.80\r\n");
		return 1;
	}
	return 0;
}

/*--OPC置位-------------------------------------------------------*/
vu8 OPC( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*OPC?\n" ,6)==0 )
	{			
		printf("%d\r\n", StandEventREG);		//标准事件的OPC位 !!
		return 1;	
	}
	return 0;	
}

/*--RST置位-------------------------------------------------------*/
vu8 RST( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*RST\n" ,5)==0 )
	{		
		return 1;//复位指令
	}
	return 0;//复位指令
}

/*--STB 读取状态位组寄存器值----------------------------------------*/
vu8 STB( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "*STB?\n" ,6)==0 )
	{		
			printf("%d\r\n", BitCropeREG);	//位组寄存器	
			return 1;	
	}
	return 0;	
}

//-B 必备命令---------------------------------------------------------------------

/*--ERROR 错误信息查询----------------------------------------*/
vu8 ERR( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:ERR?\n" ,10)==0 )
	{		
		printf("ERR...\r\n");//返回相应的错误
		return 1;//报相应的错
	}
	return 0;//
}

/*--查询负载SCPI版本号----------------------------------------*/
vu8 VERS( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:VERS?\n" ,11)==0 )
	{		
		 printf("1995.0\r\n");//返回相应的SCPI版本格式
		 return 1;
	}
	return 0;
}

/*--远端补偿功能ON/OFF----------------------------------------*/
vu8 SENS( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:SENS " ,10 )==0 )
	{		
		if( (strncasecmp( (const char *)&p_buffer[10], "ON" ,2 )==0) || (strncasecmp( (const char *)&p_buffer[10], "1" ,1 )==0) )
		{
			Sence_SW=1;  //远端补偿寄存器
			return 1;
		}
		if( (strncasecmp( (const char *)&p_buffer[10], "OFF" ,3 )==0) || (strncasecmp( (const char *)&p_buffer[10], "0" ,1 )==0) )
		{
			Sence_SW=0;  
			return 0;
		}
	}
	return 0;
}
/*--查询远端补偿功能ON/OFF----------------------------------------*/
vu8 QSENS( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:SENS?\n" ,11 )==0 )
	{			
			printf("%d\r\n",Sence_SW);	
			return 1;	
	}
	return 0;	
}

///*--蜂鸣器功能ON/OFF----------------------------------------*/
//vu8 BEER( vu8 *p_buffer )
//{
//	if( strncmp( (const char *)p_buffer, "SYST:BEEP:STAT " ,15 )==0 )
//	{		
//		if( (strncmp( (const char *)&p_buffer[15], "ON" ,2 )==0) || (strncmp( (const char *)&p_buffer[15], "1" ,1 )==0) )
//		{
//			BeeperREG=1 ;  //蜂鸣器控制
//			return 1;
//		}
//		if( (strncmp( (const char *)&p_buffer[15], "OFF" ,3 )==0) || (strncmp( (const char *)&p_buffer[15], "0" ,1 )==0) )
//		{
//			BeeperREG=0 ;  
//			return 0;
//		}
//	}
//	return 0;
//}
///*--查询蜂鸣器功能ON/OFF----------------------------------------*/
//vu8 QBEER( vu8 *p_buffer )
//{
//	if( strncmp( (const char *)p_buffer, "SYST:BEEP:STAT?\n" ,16 )==0 )
//	{			
//			printf("%d\r\n",BeeperREG);			
//			return 1;	
//	}
//	return 0;	
//}

/*--键盘锁----------------------------------------*/
vu8 LOCAL( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:LOC\n" ,9 )==0 )
	{			
			Operation_MODE =0;//键盘有用
			return 1;	
	}
	return 0;	
}
vu8 REMT( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:REM\n" ,9 )==0 )
	{			
			Operation_MODE =1;//部分键盘有用
			return 1;	
	}
	return 0;	
}
vu8 REML( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SYST:RWL\n" ,9 )==0 )
	{			
			Operation_MODE =2;//键盘无效 只能远端
			return 1;	
	}
	return 0;	
}

/*--查询寄存器组事件寄存器---------------------------------------*/
vu8 QUESTEVT( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STAT:QUES:EVEN?\n" ,16 )==0 )
	{						
			printf("%d\r\n",QuestEventREG);	
			return 1;	
	}
	return 0;	
}
/*--查询寄存器组状态寄存器---------------------------------------*/
vu8 QUESTST( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STAT:QUES:COND?\n" ,16 )==0 )
	{						
			printf("%d\r\n",BitCropeREG);			
			return 1;	
	}
	return 0;	
}

/*--读取操作状态寄存器组事件 寄存器---------------------------------------*/
vu8 OPERT( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STAT:OPER:EVEN?\n" ,16 )==0 )
	{						
			printf("%d\r\n",BitCropeREG);			
			return 1;	
	}
	return 0;	
}
/*--读取操作状态寄存器组状态寄存器---------------------------------------*/
vu8 OPERTCP( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STATus:OPERation:CONDition?\n" ,16 )==0 )
	{						
			printf("%d\r\n",BitCropeREG);			
			return 1;	
	}
	return 0;	
}
/*--读取操作状态寄存器组使能寄存器---------------------------------------*/
vu8 OPERTsT( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STAT:OPER:ENAB \n" ,16 )==0 )
	{						
			atoi( (const char *)&p_buffer[15] );	//赋值到寄存器  !!
			return 1;	
	}
	return 0;	
}
/*--读取操作状态寄存器组使能寄存器-值--------------------------------------*/
vu8 REOPERTCP( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "STATus:OPERation:ENABle?\n" ,25 )==0 )
	{						
			UART_Buffer_Send[0] = BitCropeREG;// 要改！！
			UART_Buffer_Send[1] = 0x0d;
			UART_Buffer_Send[2] = 0x0a;
			Transmit_BUFFERsize = 3;
			UART_SEND_flag=1;			
			return 1;	
	}
	return 0;	
}




//-C 输入设置命令---------------------------------------------------------------------
/*--开启和关闭----------------------------------------*/
vu8 ONOFF( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "INP " ,4 )==0 )
	{			
		if( ( strncasecmp( (const char *)&p_buffer[4], "1" ,1 )==0 ) || ( strncasecmp( (const char *)&p_buffer[4], "ON" ,2 )==0 ))
		{
			onoff_ch =1;//ON
		}
		if( ( strncasecmp( (const char *)&p_buffer[4], "0" ,1 )==0 ) || ( strncasecmp( (const char *)&p_buffer[4], "OFF" ,3 )==0 ))
		{
			onoff_ch =0;//OFF
		}	
		return 1;	
	}
	return 0;	
}
/*--查询开启和关闭----------------------------------------*/
vu8 QUONOFF( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "INP?\n" ,5 )==0 )
	{			
		printf("%d\r\n",onoff_ch);		
		return 1;	
	}
	return 0;	
}
/*--输入短路设置----------------------------------------*/
vu8 ShortONOFF( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "INP:SHOR " ,9 )==0 )
	{			
		if( ( strncasecmp( (const char *)&p_buffer[9], "1" ,1 )==0 ) || ( strncasecmp( (const char *)&p_buffer[9], "ON" ,2 )==0 ))
		{
			MODE =6;//开启短路模式
			ShortENREG=1;
		}
		if( ( strncasecmp( (const char *)&p_buffer[9], "0" ,1 )==0 ) || ( strncasecmp( (const char *)&p_buffer[9], "OFF" ,3 )==0 ))
		{
			MODE =0;//关闭短路模式
			ShortENREG=0;
		}	
		return 1;	
	}
	return 0;	
}
/*--查询输入短路开启和关闭----------------------------------------*/
vu8 QUShortONOFF( vu8 *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "INP:SHOR?\n" ,10 )==0 )
	{			
		printf("%d\r\n",ShortENREG);//
		return 1;	
	}
	return 0;	
}
/*--设置电流档位----------------------------------------*/
vu8 SET_CURR_Level( vu8  *p_buffer )
{
  vu16 temp;
 
	if( strncasecmp( (const char *)p_buffer, "CURR:RANGE " ,11 )==0 )
	{		
		if( strncasecmp( (const char *)&p_buffer[11], "MAX" ,3 )==0 )//高档
		{
			I_Gear_SW=1; 		
		}
		else if( strncasecmp( (const char *)&p_buffer[11], "MIN" ,3 )==0 )//低档档
		{
			I_Gear_SW=0; 	
		}
		else //当参数落在低档，就用低档，落在高档 就用高档
		{
			if( p_buffer[12 ] == '.') //0.xxxxA
			{
				temp =  ( (p_buffer[11 ] - 0x30 )*10000 +  (p_buffer[13 ] - 0x30 )*1000  +  (p_buffer[14 ] - 0x30 )*100);
				if( temp <= MAX_CURRENT) //低档
				{
					I_Gear_SW=0; 
				}
				else 
				{
					I_Gear_SW=1; 	
				}
			}
			else if( p_buffer[13 ] == '.') //00.xxxA
			{
				temp =  ( (p_buffer[11 ] - 0x30 )*10 +  (p_buffer[12 ] - 0x30 ) );
				if( temp <= ( MAX_CURRENT /10000)) //低档
				{
					I_Gear_SW=0; 
				}
				else 
				{
					I_Gear_SW=1;	
				}
			}
			else if(p_buffer[12 ] == '\n') //0\n
			{
				temp =  ( (p_buffer[11 ] - 0x30 )  );
				if( temp <= ( MAX_CURRENT /10000)) //低档
				{
					I_Gear_SW=0; 
				}
				else 
				{
					I_Gear_SW=1; 	
				}
			}
			else  //00xxxA
			{
				temp =  ( (p_buffer[11 ] - 0x30 )*10 +  (p_buffer[12 ] - 0x30 ) );
				if( temp <= ( MAX_CURRENT /10000)) //低档
				{
					I_Gear_SW=0; 
				}
				else 
				{
					I_Gear_SW=1; 	
				}
			}			
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=14;//存储的位置	
	}
	return 0;	
}

/*--查询电流档位----------------------------------------*/
vu8 QU_CURR_Level( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "CURR:RANGE?\n" ,12 )==0 )
	{
	  if(I_Gear_SW==0)//高档
		{
			printf("20.0000\r\n");
		}
		else 
		{
			printf("2.00000\r\n");
		}
		return 1;
	}
	return 0;		
}


/*--设置电压档位----------------------------------------*/
vu8 SET_VOLT_Level( vu8  *p_buffer )
{
  vu32 temp;
 
	if( strncasecmp( (const char *)p_buffer, "SOUR:VOLT:RANGE " ,16 )==0 )
	{		
		if( strncasecmp( (const char *)&p_buffer[16], "MAX" ,3 )==0 )//高档
		{
			V_Gear_SW=1; 		
		}
		else if( strncasecmp( (const char *)&p_buffer[16], "MIN" ,3 )==0 )//低档档
		{
			V_Gear_SW=0;
		}
		else //当参数落在低档，就用低档，落在高档 就用高档
		{
			if( p_buffer[17 ] == '.') //0.xxxxV
			{
				temp =  ( (p_buffer[16 ] - 0x30 )  );
				if( temp <= MAX_VOLOTE/10000) //低档
				{
					V_Gear_SW=0;
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}
			else if( p_buffer[18 ] == '.') //00.xxxxV
			{
				temp =  ( (p_buffer[16 ] - 0x30 )*10 +  (p_buffer[17 ] - 0x30 ));
				if( temp <= MAX_VOLOTE/10000) //低档
				{
					V_Gear_SW=0;
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}
			else if( p_buffer[19 ] == '.') //000.xxxV
			{
				temp =  ( (p_buffer[16 ] - 0x30 )*100 +  (p_buffer[17 ] - 0x30 )*10  +  (p_buffer[18 ] - 0x30 ) );
				if( temp <= MAX_VOLOTE/1000) //低档
				{
					V_Gear_SW=0; 
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}	
			else if(p_buffer[17 ] == '\n') //0\n
			{
				temp =  ( (p_buffer[16 ] - 0x30 )  );
				if( temp <= ( MAX_VOLOTE /10000)) //低档
				{
					V_Gear_SW=0; 
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}
			else if(p_buffer[18 ] == '\n') //00\n
			{
				temp =  ( (p_buffer[16 ] - 0x30 )*10 + (p_buffer[17 ] - 0x30 ) );
				if( temp <= ( MAX_VOLOTE /10000)) //低档
				{
					V_Gear_SW=0; 
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}
			else  //000xxxA\n
			{
				temp =  ( (p_buffer[16 ] - 0x30 )*100 +  (p_buffer[17 ] - 0x30 )*10 +  (p_buffer[18 ] - 0x30 ) );
				if( temp <= ( MAX_VOLOTE /1000)) //低档
				{
					V_Gear_SW=0; 
				}
				else 
				{
					V_Gear_SW=1; 	
				}
			}	
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=15;//存储的位置	
	}
	return 0;	
}

/*--查询电压档位----------------------------------------*/
vu8 QU_VOLT_Level( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "SOUR:VOLT:RANGE?\n" ,17 )==0 )
	{
		if(V_Gear_SW==0)//高档
		{
			printf("150.000\r\n");
		}
		else 
		{
			printf("15.0000\r\n");
		}
		return 1;
	}
	return 0;		
}
/*--设置相同的电流上升下降率---------------------------------------*/
vu8 SET_CURR_SLEW( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "CURR:SLEW " ,10 )==0 )
	{
		vu32 temp;
 	
			if( strncasecmp( (const char *)&p_buffer[10], "MAX" ,3 )==0 )//最大上升率
			{
				I_Rise_Time = MAX_CURRENTUP; 	//电流爬升率
				I_Down_Time = MAX_CURRENTDOWN; 	//电流下降率
			}
			else if( strncasecmp( (const char *)&p_buffer[10], "MIN" ,3 )==0 )//最小上升率
			{
				I_Rise_Time = MIN_CURRENTUP; 	
				I_Down_Time = MIN_CURRENTDOWN; 
			}
			else 
			{
				if( p_buffer[11 ] == '\n') //0xxxxA  最大值为3.0000
				{
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 );
					if( temp> MAX_CURRENTUP )
					  temp =  MAX_CURRENTUP;
					if( temp < MIN_CURRENTUP )	
						temp = MIN_CURRENTUP;
						
					I_Rise_Time = temp; 	
				  I_Down_Time = temp; 
				}	
				else if( p_buffer[13 ] == '\n') //0.xA  最大值为3.0000
				{
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 );
					if( temp> MAX_CURRENTUP )
					  temp =  MAX_CURRENTUP;
					if( temp < MIN_CURRENTUP )	
						temp = MIN_CURRENTUP;
						
					I_Rise_Time = temp; 	
				  I_Down_Time = temp; 
				}	
				else if( p_buffer[14 ] == '\n') //0.xxA  最大值为3.0000
				{
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100);
					if( temp> MAX_CURRENTUP )
					  temp =  MAX_CURRENTUP;
					if( temp < MIN_CURRENTUP )	
						temp = MIN_CURRENTUP;
						
					I_Rise_Time = temp; 	
				  I_Down_Time = temp; 
				}
				else if( p_buffer[15 ] == '\n') //0.xxxA  最大值为3.0000
				{
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10);
					if( temp> MAX_CURRENTUP )
					  temp =  MAX_CURRENTUP;
					if( temp < MIN_CURRENTUP )	
						temp = MIN_CURRENTUP;
					I_Rise_Time = temp; 	
				  I_Down_Time = temp; 
				}
				else if( p_buffer[16 ] == '\n') //0.xxxxA  最大值为3.0000
				{
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10 +  (p_buffer[15 ] - 0x30 ));
					if( temp> MAX_CURRENTUP )
					  temp =  MAX_CURRENTUP;
					if( temp < MIN_CURRENTUP )	
						temp = MIN_CURRENTUP;
						
					I_Rise_Time = temp; 	
				  I_Down_Time = temp; 
				}
			}
			Flag_Save_SW=1;//开启FLASH存储
			Setcontr_ADDR=31;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "CURR:SLEW?\n" ,11 )==0 )//查询上升下降率
	{
		printf("%.4f,%.4f\r\n" , ((float)I_Rise_Time )/10000, ((float)I_Down_Time )/10000  );		
		return 1;
	}
	return 0;
}

/*--设置电流上升率---------------------------------------*/
vu8 SET_CURR_SLEW_RISE( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "CURR:SLEW:RISE " ,15 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[15], "MAX" ,3 )==0 )//最大上升率
		{
			I_Rise_Time = MAX_CURRENTUP; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[15], "MIN" ,3 )==0 )//最小上升率
		{
			I_Rise_Time = MIN_CURRENTUP; 	
		}
		else 
		{			
			if( p_buffer[16 ] == '\n') //0xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 );
				if( temp> MAX_CURRENTUP )
					temp =  MAX_CURRENTUP;
					
				I_Rise_Time = temp; 	
			}	
			else if( p_buffer[18 ] == '\n') //0.xA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 );
				if( temp> MAX_CURRENTUP )
					temp =  MAX_CURRENTUP;
					
				I_Rise_Time = temp; 	
			}	
			else if( p_buffer[19 ] == '\n') //0.xxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100);
				if( temp> MAX_CURRENTUP )
					temp =  MAX_CURRENTUP;
					
				I_Rise_Time = temp; 	
			}
			else if( p_buffer[20 ] == '\n') //0.xxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10);
				if( temp> MAX_CURRENTUP )
					temp =  MAX_CURRENTUP;

				I_Rise_Time = temp; 	
			}
			else if( p_buffer[21 ] == '\n') //0.xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10 +  (p_buffer[20 ] - 0x30 ));
				if( temp> MAX_CURRENTUP )
					temp =  MAX_CURRENTUP;
					
				I_Rise_Time = temp; 	
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=31;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "CURR:SLEW:RISE?\n" ,16 )==0 )//查询上升下降率
	{
		printf("%.4f\r\n" , ((float)I_Rise_Time )/10000);		
		return 1;
	}
	return 0;
}

/*--设置电流下降率---------------------------------------*/
vu8 SET_CURR_SLEW_FALL( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "CURR:SLEW:FALL " ,15 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[15], "MAX" ,3 )==0 )//最大上升率
		{
			I_Down_Time = MAX_CURRENTDOWN; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[15], "MIN" ,3 )==0 )//最小上升率
		{
			I_Down_Time = MIN_CURRENTDOWN; 	
		}
		else 
		{			
			if( p_buffer[16 ] == '\n') //0xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 );
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;	
				I_Down_Time = temp; 	
			}	
			else if( p_buffer[18 ] == '\n') //0.xA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 );
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				I_Down_Time = temp; 	
			}	
			else if( p_buffer[19 ] == '\n') //0.xxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100);
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				I_Down_Time = temp; 	
			}
			else if( p_buffer[20 ] == '\n') //0.xxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10);
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				I_Down_Time = temp; 	
			}
			else if( p_buffer[21 ] == '\n') //0.xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10 +  (p_buffer[20 ] - 0x30 ));
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;	
				I_Down_Time = temp; 	
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=32;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "CURR:SLEW:FALL?\n" ,16 )==0 )//查询下降率
	{
		printf("%.4f\r\n" , ((float)I_Down_Time )/10000);		
		return 1;
	}
	return 0;
}

/*----设定电压下降率--------------------------------------*/
vu8 SET_VOLT_SLEW_FALL( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "VOLT:SLEW:FALL " ,15 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[15], "MAX" ,3 )==0 )//最大上升率
		{
			CV_Down_Time = 5000; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[15], "MIN" ,3 )==0 )//最小上升率
		{
			CV_Down_Time = 10; 	
		}
		else 
		{			
			if( p_buffer[16 ] == '\n') //0xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 );
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;	
				CV_Down_Time = temp; 	
			}	
			else if( p_buffer[18 ] == '\n') //0.xA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 );
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				CV_Down_Time = temp; 	
			}	
			else if( p_buffer[19 ] == '\n') //0.xxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100);
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				CV_Down_Time = temp; 	
			}
			else if( p_buffer[20 ] == '\n') //0.xxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10);
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;
				CV_Down_Time = temp; 	
			}
			else if( p_buffer[21 ] == '\n') //0.xxxxA  最大值为3.0000
			{
				temp =  ( (p_buffer[15 ] - 0x30 )*10000 +  (p_buffer[17 ] - 0x30 )*1000 +  (p_buffer[18 ] - 0x30 )*100 +  (p_buffer[19 ] - 0x30 )*10 +  (p_buffer[20 ] - 0x30 ));
				if( temp> MAX_CURRENTDOWN )
					temp =  MAX_CURRENTDOWN;
				if( temp < MIN_CURRENTDOWN )	
						temp = MIN_CURRENTDOWN;	
				CV_Down_Time = temp; 	
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=33;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "VOLT:SLEW:FALL?\n" ,16 )==0 )//查询下降率
	{
		printf("%.4f\r\n" , ((float)CV_Down_Time )/10000);		
		return 1;
	}
	return 0;
}
/*--设置电流保护值---------------------------------------*/
vu8 SET_CURR_PROT( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "CURR:PROT " ,10 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[10], "MAX" ,3 )==0 )//最大电流
		{
			MAX_I = MAX_CURRENTPOOTE; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[10], "MIN" ,3 )==0 )//最小电流
		{
			MAX_I = 0; 
		}
		else 
		{			
			if( p_buffer[11 ] == '\n') //0\n  最大值为21.0000
			{
				temp =  ( (p_buffer[10 ] - 0x30 ) )*10000;
				if( temp> MAX_CURRENTPOOTE )
					temp =  MAX_CURRENTPOOTE;

				MAX_I = temp; 	
			}	
			else if( p_buffer[12 ] == '\n') //00\n  最大值为21.0000
			{
				temp =  ( (p_buffer[10 ] - 0x30 )*100000 +  (p_buffer[11 ] - 0x30 )*10000 );
				if( temp> MAX_CURRENTPOOTE )
					temp =  MAX_CURRENTPOOTE;

				MAX_I = temp; 	
			}	
			else if( p_buffer[13 ] == '\n') //0.x\n  最大值为21.0000
			{
			 if( p_buffer[11 ] == '.')
			 {
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 );
					
					if( temp> MAX_CURRENTPOOTE )
					temp =  MAX_CURRENTPOOTE;
					
					MAX_I = temp; 	
			 }				
			}
			else if( p_buffer[14 ] == '\n') //0.xxA/00.x  最大值为21.0000
			{
				if( p_buffer[11 ] == '.')
				 {
						temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000  +  (p_buffer[13 ] - 0x30 )*100  );
						
						if( temp> MAX_CURRENTPOOTE )
						temp =  MAX_CURRENTPOOTE;
						
						MAX_I = temp; 	
				 }	
				 else if( p_buffer[12 ] == '.')
				 {
						temp =  ( (p_buffer[10 ] - 0x30 )*100000 +  (p_buffer[11 ] - 0x30 )*10000  +  (p_buffer[13 ] - 0x30 )*1000  );
						
						if( temp> MAX_CURRENTPOOTE )
						temp =  MAX_CURRENTPOOTE;
						
						MAX_I = temp; 	
				 }	
				 
			}
			else if( p_buffer[15 ] == '\n') //0.xxxA 00.xx 最大值为21.0000
			{
				if( p_buffer[11 ] == '.')
				 {
						temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000  +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10 );
						
						if( temp> MAX_CURRENTPOOTE )
						temp =  MAX_CURRENTPOOTE;
						
						MAX_I = temp; 	
				 }
				 else if( p_buffer[12 ] == '.')
				 {
						temp =  ( (p_buffer[10 ] - 0x30 )*100000 +  (p_buffer[11 ] - 0x30 )*10000  +  (p_buffer[13 ] - 0x30 )*1000 +  (p_buffer[14 ] - 0x30 )*100 );
						
						if( temp> MAX_CURRENTPOOTE )
						temp =  MAX_CURRENTPOOTE;
						
						MAX_I = temp; 	
				 }
			}
			else if( p_buffer[16 ] == '\n') //0.xxxxA  00.xxx最大值为 21.0000
			{
				if( p_buffer[11 ] == '.')
				{				 
					temp =  ( (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[12 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10 +  (p_buffer[15 ] - 0x30 ));
					if( temp> MAX_CURRENTPOOTE/10 )
						temp =  MAX_CURRENTPOOTE;
						
					MAX_I = temp; 	
				}
				else if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[10 ] - 0x30 )*100000 +  (p_buffer[11 ] - 0x30 )*10000 +  (p_buffer[13 ] - 0x30 )*1000 +  (p_buffer[14 ] - 0x30 )*100 +  (p_buffer[15 ] - 0x30 )*10);
					if( temp> MAX_CURRENTPOOTE)
						temp =  MAX_CURRENTPOOTE;
						
					MAX_I = temp; 	
				}
			}
			else if( p_buffer[17 ] == '\n') //00.xxxx最大值为 21.0000
			{
				if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[10 ] - 0x30 )*100000 +  (p_buffer[11 ] - 0x30 )*10000 +  (p_buffer[13 ] - 0x30 )*1000 +  (p_buffer[14 ] - 0x30 )*100 +  (p_buffer[15 ] - 0x30 )*10 +  (p_buffer[16 ] - 0x30 ));
					if( temp> MAX_CURRENTPOOTE)
						temp =  MAX_CURRENTPOOTE;
						
					MAX_I = temp; 	
				}
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=28;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "CURR:PROT?\n" ,11 )==0 )//查询电流保护值
	{
		if(MAX_I< 100000)//字符前加0显示
		{
			printf(" %.4f\r\n" , ((float)MAX_I )/10000);	
		}
		else
		{	
			printf("%.4f\r\n" , ((float)MAX_I )/10000);		
		}
		return 1;
	}
	return 0;
}
/*--设置功率保护值---------------------------------------*/
vu8 SET_POW_PROT( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "POW:PROT " ,9 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[9], "MAX" ,3 )==0 )//最大功率
		{
			MAX_P = MAX_POWERPOOTE; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[9], "MIN" ,3 )==0 )//最小功率
		{
			MAX_P = 0; 
		}
		else 
		{			
			if( p_buffer[10 ] == '\n') //0\n  最大值为150.00
			{
				temp =  ( (p_buffer[9 ] - 0x30 ) )*100;
				if( temp> MAX_POWERPOOTE )
					temp =  MAX_POWERPOOTE;

				MAX_P = temp; 	
			}	
			else if( p_buffer[11 ] == '\n') //00\n  最大值为150.00
			{
				temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100 );
				if( temp> MAX_POWERPOOTE )
					temp =  MAX_POWERPOOTE;

				MAX_P = temp; 	
			}	
			else if( p_buffer[12 ] == '\n') //0.x\n 000\n  最大值为150.00
			{
				 if( p_buffer[10 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*100 +  (p_buffer[11 ] - 0x30 )*10 );
						
						if( temp> MAX_POWERPOOTE )
						temp =  MAX_POWERPOOTE;
						
						MAX_P = temp; 	
				 }
				 else //000\n
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000  +  (p_buffer[11 ] - 0x30 )*100);
						
						if( temp> MAX_POWERPOOTE )
						temp =  MAX_POWERPOOTE;
						
						MAX_P = temp; 	
				 }
			}
			else if( p_buffer[13 ] == '\n') //0.xxA/00.x  最大值为150.00
			{
				if( p_buffer[10 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*100 +  (p_buffer[11 ] - 0x30 )*10  +  (p_buffer[12 ] - 0x30 )  );
						
						if( temp> MAX_POWERPOOTE )
						temp =  MAX_POWERPOOTE;
						
						MAX_P = temp; 	
				 }	
				 else if( p_buffer[11 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100  +  (p_buffer[12 ] - 0x30 )  );
						
						if( temp> MAX_POWERPOOTE )
						temp =  MAX_POWERPOOTE;
						
						MAX_P = temp; 	
				 }					 
			}
			else if( p_buffer[14 ] == '\n') //00.xx\n 000.x\n 最大值为 150.00
			{
				if( p_buffer[11 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100 +  (p_buffer[12 ] - 0x30 )*10 +  (p_buffer[13 ] - 0x30 ) );
					if( temp> MAX_CURRENTPOOTE)
						temp =  MAX_CURRENTPOOTE;
						
					MAX_P = temp; 	
				}
				else if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 );
					if( temp> MAX_CURRENTPOOTE)
						temp =  MAX_CURRENTPOOTE;
						
					MAX_P = temp; 	
				}
			}
			else if( p_buffer[15 ] == '\n') //000.xx\n 最大值为 150.00
			{
				if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 +  (p_buffer[14 ] - 0x30 ) );
					if( temp> MAX_CURRENTPOOTE)
						temp =  MAX_CURRENTPOOTE;
						
					MAX_P = temp; 	
				}
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=26;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "POW:PROT?\n" ,10 )==0 )//查询功率保护值
	{
		if(MAX_P< 1000)//字符前加0显示
		{
			printf("   %.2f\r\n" , ((float)MAX_P )/100);	
		}
		else if(MAX_P>999 && MAX_P < 10000)
		{	
			printf("  %.2f\r\n" , ((float)MAX_P )/100);		
		}
		else
		{	
			printf(" %.2f\r\n" , ((float)MAX_P )/100);		
		}
		return 1;
	}
	return 0;
}

/*--设置负载开始带载电压值---------------------------------------*/
vu8 SET_LoadST_VOLT( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "VOLT:ON " ,8 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[8], "MAX" ,3 )==0 )//最大带载电压 120.00
		{
			VON_Voltage = MAX_LoadVOLOTE; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[8], "MIN" ,3 )==0 )//最小带载电压 0
		{
			VON_Voltage = 0; 
		}
		else 
		{			
			if( p_buffer[9 ] == '\n') //0\n  最大值为120.000
			{
				temp =  ( (p_buffer[8 ] - 0x30 ) )*1000;
				if( temp> MAX_LoadVOLOTE )
					temp =  MAX_LoadVOLOTE;

				VON_Voltage = temp; 	
			}	
			else if( p_buffer[10 ] == '\n') //00\n  最大值为120.000
			{
				temp =  ( (p_buffer[8 ] - 0x30 )*10000 +  (p_buffer[9 ] - 0x30 )*1000 );
				if( temp> MAX_LoadVOLOTE )
					temp =  MAX_LoadVOLOTE;

				VON_Voltage = temp; 	
			}	
			else if( p_buffer[11 ] == '\n') //0.x\n 000\n  最大值为120.000
			{
				 if( p_buffer[9 ] == '.')
				 {
						temp =  ( (p_buffer[8 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100 );
						
						if( temp> MAX_LoadVOLOTE )
						temp =  MAX_LoadVOLOTE;
						
						VON_Voltage = temp; 	
				 }
				 else //000\n
				 {
						temp =  ( (p_buffer[8 ] - 0x30 )*100000 +  (p_buffer[9 ] - 0x30 )*10000  +  (p_buffer[10 ] - 0x30 )*1000);
						
						if( temp> MAX_LoadVOLOTE )
						temp =  MAX_LoadVOLOTE;
						
						VON_Voltage = temp; 	
				 }
			}
			else if( p_buffer[12 ] == '\n') //0.xxA/00.x  最大值为 120.000
			{
				if( p_buffer[9 ] == '.')
				 {
						temp =  ( (p_buffer[8 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100  +  (p_buffer[11 ] - 0x30 ) *10 );
						
						if( temp> MAX_LoadVOLOTE )
						temp =  MAX_LoadVOLOTE;
						
						VON_Voltage = temp; 	
				 }	
				 else if( p_buffer[10 ] == '.')
				 {
						temp =  ( (p_buffer[8 ] - 0x30 )*10000 +  (p_buffer[9 ] - 0x30 )*1000  +  (p_buffer[11 ] - 0x30 ) *100 );
						
						if( temp> MAX_LoadVOLOTE )
						temp =  MAX_LoadVOLOTE;
						
						VON_Voltage = temp; 	
				 }					 
			}
			else if( p_buffer[13 ] == '\n') // 0.xxx\n  00.xx\n   000.x\n  最大值为 120.000
			{
				if( p_buffer[9 ] == '.') //0.xxx\n 
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*1000 +  (p_buffer[10 ] - 0x30 )*100 +  (p_buffer[11 ] - 0x30 )*10 +  (p_buffer[12 ] - 0x30 ) );
					if( temp> MAX_LoadVOLOTE)
					temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}
				else if( p_buffer[10 ] == '.')//00.xx\n 
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*10000 +  (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 +  (p_buffer[12 ] - 0x30 )*10 );
					if( temp> MAX_LoadVOLOTE)
						temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}			
				else if( p_buffer[11 ] == '.')//000.x\n
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*100000 +  (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[12 ] - 0x30 )*100 );
					if( temp> MAX_LoadVOLOTE)
						temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}
			}
			else if( p_buffer[14 ] == '\n') //000.xx\n  00.xxx\n 最大值为 120.000
			{
				if( p_buffer[11 ] == '.')
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*100000 +  (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[12 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 );
					if( temp> MAX_LoadVOLOTE)
					temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}
				else if( p_buffer[10 ] == '.')
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*10000 +  (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 +  (p_buffer[12 ] - 0x30 )*10 +  (p_buffer[13 ] - 0x30 ) );
					if( temp> MAX_LoadVOLOTE)
					temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}
			}
			else if( p_buffer[15 ] == '\n') //000.xxx\n 最大值为 120.000
			{
				if( p_buffer[11 ] == '.')
				{				 
					temp =  ( (p_buffer[8 ] - 0x30 )*100000 +  (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[12 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 +  (p_buffer[14 ] - 0x30 ) );
					if( temp> MAX_LoadVOLOTE)
						temp =  MAX_LoadVOLOTE;
						
					VON_Voltage = temp; 	
				}
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=29;//存储的位置	
		return 1;
	}
	else  if( strncasecmp( (const char *)p_buffer, "VOLT:ON?\n" ,9 )==0 )//查询开始带载电压值
	{
		if(VON_Voltage< 10000)//字符前加0显示 120.000 1.000  12.000
		{
			printf("  %.3f\r\n" , ((float)VON_Voltage )/1000);
		}
		else if( VON_Voltage > 9999  && VON_Voltage < 100000) //字符前加0显示 120.000 
		{
			printf(" %.3f\r\n" , ((float)VON_Voltage )/1000);
		}
		else
		{	
			printf("%.3f\r\n" , ((float)VON_Voltage )/1000);		
		}
		return 1;
	}
	return 0;
}
/*--设置负载卸载电压值---------------------------------------*/
vu8 SET_FreeLoad_VOLT( vu8  *p_buffer )
{
	vu32 temp;
	if( strncasecmp( (const char *)p_buffer, "VOLT:OFF " ,9 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[9], "MAX" ,3 )==0 )//最大带载电压
		{
			VOFF_Voltage = MAX_FreeLoadVOLOTE; 	
		}
		else if( strncasecmp( (const char *)&p_buffer[9], "MIN" ,3 )==0 )//最小带载电压 120.00
		{
			VOFF_Voltage = 0; 
		}
		else 
		{			
			if( p_buffer[10 ] == '\n') //0\n  最大值为120.000
			{
				temp =  ( (p_buffer[9 ] - 0x30 ) )*1000;
				if( temp> MAX_FreeLoadVOLOTE )
					temp =  MAX_FreeLoadVOLOTE;

				VOFF_Voltage = temp; 	
			}	
			else if( p_buffer[11 ] == '\n') //00\n  最大值为120.000
			{
				temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 );
				if( temp> MAX_FreeLoadVOLOTE )
					temp =  MAX_FreeLoadVOLOTE;

				VOFF_Voltage = temp; 	
			}	
			else if( p_buffer[12 ] == '\n') //0.x\n 000\n  最大值为120.000
			{
				 if( p_buffer[10 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 );
						
						if( temp> MAX_FreeLoadVOLOTE )
						temp =  MAX_FreeLoadVOLOTE;
						
						VOFF_Voltage = temp; 	
				 }
				 else //000\n
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*100000 +  (p_buffer[10 ] - 0x30 )*10000  +  (p_buffer[11 ] - 0x30 )*1000);
						
						if( temp> MAX_FreeLoadVOLOTE )
						temp =  MAX_FreeLoadVOLOTE;
						
						VOFF_Voltage = temp; 	
				 }
			}
			else if( p_buffer[13 ] == '\n') //0.xxA/00.x  最大值为 120.000
			{
				if( p_buffer[10 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100  +  (p_buffer[12 ] - 0x30 ) *10 );
						
						if( temp> MAX_FreeLoadVOLOTE )
						temp =  MAX_FreeLoadVOLOTE;
						
						VOFF_Voltage = temp; 	
				 }	
				 else if( p_buffer[11 ] == '.')
				 {
						temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000  +  (p_buffer[12 ] - 0x30 ) *100 );
						
						if( temp> MAX_FreeLoadVOLOTE )
						temp =  MAX_FreeLoadVOLOTE;
						
						VOFF_Voltage = temp; 	
				 }					 
			}
			else if( p_buffer[14 ] == '\n') // 0.xxx\n  00.xx\n   000.x\n  最大值为 120.000
			{
				if( p_buffer[10 ] == '.') //0.xxx\n 
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*1000 +  (p_buffer[11 ] - 0x30 )*100 +  (p_buffer[12 ] - 0x30 )*10 +  (p_buffer[13 ] - 0x30 ) );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}
				else if( p_buffer[11 ] == '.')//00.xx\n 
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[12 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}			
				else if( p_buffer[12 ] == '.')//000.x\n
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*100000 +  (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[11 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}
			}
			else if( p_buffer[15 ] == '\n') //000.xx\n  00.xxx\n 最大值为 120.000
			{
				if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*100000 +  (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[11 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10 );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}
				else if( p_buffer[11 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*10000 +  (p_buffer[10 ] - 0x30 )*1000 +  (p_buffer[12 ] - 0x30 )*100 +  (p_buffer[13 ] - 0x30 )*10 +  (p_buffer[14 ] - 0x30 ) );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}
			}
			else if( p_buffer[16 ] == '\n') //000.xxx\n 最大值为 120.000
			{
				if( p_buffer[12 ] == '.')
				{				 
					temp =  ( (p_buffer[9 ] - 0x30 )*100000 +  (p_buffer[10 ] - 0x30 )*10000 +  (p_buffer[11 ] - 0x30 )*1000 +  (p_buffer[13 ] - 0x30 )*100 +  (p_buffer[14 ] - 0x30 )*10 +  (p_buffer[15 ] - 0x30 ) );
					if( temp> MAX_FreeLoadVOLOTE)
						temp =  MAX_FreeLoadVOLOTE;
						
					VOFF_Voltage = temp; 	
				}
			}
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=30;//存储的位置	
		return 1;
	}
	else  if( strncasecmp( (const char *)p_buffer, "VOLT:OFF?\n" ,10 )==0 )//查询开始卸载电压值
	{
		if(VOFF_Voltage< 10000)//字符前加0显示 120.000 1.000  12.000
		{
			printf("  %.3f\r\n" , ((float)VOFF_Voltage )/1000);
		}
		else if( VOFF_Voltage > 9999  && VOFF_Voltage < 100000) //字符前加0显示 120.000 
		{
			printf(" %.3f\r\n" , ((float)VOFF_Voltage )/1000);
		}
		else
		{	
			printf("%.3f\r\n" , ((float)VOFF_Voltage )/1000);		
		}
		return 1;
	}
	return 0;
}

/*--设置负载工作模式---------------------------------------*/


vu8 SET_Load_MODE( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "MODE " ,5 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[5], "CURR" ,4 )==0 )
		{
			MODE = CURR_MODE ;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "VOLT" ,4 )==0 )
		{
			MODE = VOLT_MODE ;
		} 
		else if( strncasecmp( (const char *)&p_buffer[5], "POW" ,3 )==0 )
		{
			MODE = POW_MODE ;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "RES" ,3 )==0 )
		{
			MODE = RES_MODE ;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "DYN" ,3 )==0 )
		{
			MODE = DYN_MODE ;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "LED" ,3 )==0 )
		{
			MODE = LED_MODE ;
		}
		Flag_Save_SW=1;//开启FLASH存储
		Setcontr_ADDR=13;//存储的位置	
	}
	else  if( strncasecmp( (const char *)p_buffer, "MODE?\n" ,6 )==0 )//查询负载模式
	{
		if( MODE == CURR_MODE )//
		{
			printf("CURR\r\n" );
		}
		else if( MODE == VOLT_MODE )//
		{
			printf("VOLT\r\n" );
		}
		else if( MODE == POW_MODE )//
		{
			printf("POW\r\n" );
		}
		else if( MODE == RES_MODE )//
		{
			printf("RES\r\n" );
		}
		else if( MODE == DYN_MODE )//
		{
			printf("DYN\r\n" );
		}
		else if( MODE == LED_MODE )//
		{
			printf("LED\r\n" );
		}
	}
	return 1;
}	

/*--设置负载CC电流---------------------------------------*/
vu8 SET_CC_CURR( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "CURR " ,5 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[5], "MAX" ,3 )==0 )//最大带载电流 20A
		{
				SET_Current = MAX_SET_CURRENT;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=20;//存储的位置	
				return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "MIN" ,3 )==0 )//最小带载电流 0A
		{
			  SET_Current = 0;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=20;//存储的位置	
				return 1;				
		}
		else 
		{	
			if( I_Gear_SW == HIGH_Level)
			{		
				SET_Current = Get_strNum( p_buffer , 5 , 1 , MAX_SET_CURRENT , 0); //将指定位置的字符串数字转化为10进制数字
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=20;//存储的位置	
			}
			else 
			{
				SET_Current = Get_strNum( p_buffer , 5 ,  10, MAX_SET_CURRENT , 0); //将指定位置的字符串数字转化为10进制数字
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=20;//存储的位置	
			}
			return 1;
		}
	}
	else  if( strncasecmp( (const char *)p_buffer, "CURR?\n" ,6 )==0 )//查询当前设置电流
	{
		if( I_Gear_SW == HIGH_Level)
		{
			if( (SET_Current /10000 ) <10)
				printf(" %.4f\r\n" , ((float)SET_Current )/10000);//20.0000
			else
				printf("%.4f\r\n" , ((float)SET_Current )/10000);//20.0000
		}
		else
		{
			printf("%.5f\r\n" , ((float)SET_Current )/100000);//2.00000
		}
		return 1;
	}
	return 0;
}	

/*--设置负载CV电压---------------------------------------*/
vu8 SET_CV_VOLT( vu8  *p_buffer )
{
	vu32 temp;
	
	if( strncasecmp( (const char *)p_buffer, "VOLT " ,5 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[5], "MAX" ,3 )==0 )//
		{
				SET_Voltage = MAX_SET_VOLT;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=19;//存储的位置	
				return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[5], "MIN" ,3 )==0 )//
		{
			  SET_Voltage = 0;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=19;//存储的位置	
				return 1;				
		}
		else 
		{	
			if( V_Gear_SW == HIGH_Level)
			{
				SET_Voltage = Get_strNumV( p_buffer , 5 ,  1 , MAX_SET_VOLT, 0);
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=19;//存储的位置	
			}
			else 
			{
				SET_Voltage = Get_strNumV( p_buffer , 5 ,  10 , MAX_SET_VOLT, 0);
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=19;//存储的位置	
			}
			return 1;
		}
	}
	else  if( strncasecmp( (const char *)p_buffer, "VOLT?\n" ,6 )==0 )//查询当前设置电压
	{
		if( V_Gear_SW == HIGH_Level)
		{
			if( (SET_Voltage /1000 ) <10) //150.000 15.000 1.000
				printf("  %.3f\r\n" , ((float)SET_Voltage )/1000);//
			else if( (SET_Voltage /1000 ) <100) //150.000 15.000 1.000
				printf(" %.3f\r\n" , ((float)SET_Voltage )/1000);//
			else
				printf("%.3f\r\n" , ((float)SET_Voltage )/1000);//
		}
		else
		{
			if( (SET_Voltage /10000 ) <10)
				printf(" %.4f\r\n" , ((float)SET_Voltage )/10000);//15.0000
			else 
				printf("%.4f\r\n" , ((float)SET_Voltage )/10000);//15.0000
		}
		return 1;
	}
	return 0;
}	

/*--设置负载功率---------------------------------------*/
vu8 SET_POW_VAL( vu8  *p_buffer )
{
	vu32 temp;
	
	if( strncasecmp( (const char *)p_buffer, "POW " ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "MAX" ,3 )==0 )//
		{
				SET_Power = MAX_SET_POW;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=22;//存储的位置	
				return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "MIN" ,3 )==0 )//
		{
			  SET_Power = 0;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=22;//存储的位置	
				return 1;				
		}
		else 
		{	
			SET_Power = Get_strNumV( p_buffer , 4 ,  1 , MAX_SET_POW, 0);
			Flag_Save_SW=1;//开启FLASH存储
			Setcontr_ADDR=22;//存储的位置	
			return 1;
		}
	}
	else  if( strncasecmp( (const char *)p_buffer, "POW?\n" ,5 )==0 )//查询当前设置功率
	{
			if( (SET_Power /1000 ) <10) //150.000 15.000 1.000
				printf("  %.3f\r\n" , ((float)SET_Power )/1000);//
			else if( (SET_Power /1000 ) <100) //150.000 15.000 1.000
				printf(" %.3f\r\n" , ((float)SET_Power )/1000);//
			else
				printf("%.3f\r\n" , ((float)SET_Power )/1000);//

		return 1;
	}
	return 0;
}

/*--设置负载电阻---------------------------------------*/
vu8 SET_RES_VAL( vu8  *p_buffer )
{
	float temp=0;
	
	if( strncasecmp( (const char *)p_buffer, "RES " ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "MAX" ,3 )==0 )//
		{
				SET_Resist = MAX_SET_RES;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=21;//存储的位置	
				return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "MIN" ,3 )==0 )//
		{
			  SET_Resist = 0;
				Flag_Save_SW=1;//开启FLASH存储
				Setcontr_ADDR=21;//存储的位置	
				return 1;				
		}
		else 
		{	
			if( p_buffer[ 5 ] == '\n' ) //0\n  最大值为 50000.0
			{
				temp = (p_buffer[ 4 ] - 0x30 )  ;	
			}	
			else if( p_buffer[ 6 ] == '\n' ) //00\n  最大值为 50000.0
			{
				temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10 + ( (p_buffer[ 5 ] - 0x30 ) );	
			}	
			else if( p_buffer[ 7 ] == '\n' ) //000 /0.0 \n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10 + ( p_buffer[ 6 ] - 0x30 );
					temp =  ( temp /10.0 );
				}
				else
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100 + ( (p_buffer[ 5 ] - 0x30 ) )*10 + ( (p_buffer[ 6 ] - 0x30 ) );	
				}
			}	
			else if( p_buffer[ 8 ] == '\n' ) //0000 /0.00 00.0 \n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')//0.00
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100 + ( (p_buffer[ 6 ] - 0x30 ) )*10 + ( (p_buffer[ 7 ] - 0x30 ) ) ;
					temp = temp/100.0;
				}
				else if(p_buffer[ 6 ] == '.')//00.0
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100 + ( (p_buffer[ 5 ] - 0x30 ) )*10 + ( (p_buffer[ 7 ] - 0x30 ) ) ;	
					temp = temp/10.0;
				}
				else //0000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*1000 + ( (p_buffer[ 5 ] - 0x30 ) )*100 + ( (p_buffer[ 6 ] - 0x30 ) )*10 + ( (p_buffer[ 7 ] - 0x30 ) );	
				}				
			}	
			else if( p_buffer[ 9 ] == '\n' ) //00000 /0.000 00.00 000.0 \n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')//0.000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10+ ( (p_buffer[ 8 ] - 0x30 ) );	
				  temp = temp/1000.0;
				}
				else if(p_buffer[ 6 ] == '.')//00.00
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*1000 + ( (p_buffer[ 5 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10 + ( (p_buffer[ 8 ] - 0x30 ) );	
				  temp = temp/100.0;
				}
				else if(p_buffer[ 7 ] == '.')//000.0
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*1000 + ( (p_buffer[ 5 ] - 0x30 ) )*100 + ( (p_buffer[ 6 ] - 0x30 ) )*10 + ( (p_buffer[ 8 ] - 0x30 ) );	
					temp = temp/10.0;
				}
				else //00000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10 + ( (p_buffer[ 8 ] - 0x30 ) );	
				}			
			}
			else if( p_buffer[ 10 ] == '\n' ) //00000x /0.0000 00.000 000.00  0000.0\n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')//0.0000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 8 ] - 0x30 ) )*10 + ( (p_buffer[ 9 ] - 0x30 ) );	
					temp = temp/10000.0;
				}
				else if(p_buffer[ 6 ] == '.')//00.000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 8 ] - 0x30 ) )*10 + ( (p_buffer[ 9 ] - 0x30 ) );	
					temp = temp/1000.0;
				}
				else if(p_buffer[ 7 ] == '.')//000.00
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 8 ] - 0x30 ) )*10 + ( (p_buffer[ 9 ] - 0x30 ) );	
					temp = temp/100.0;
				}
				else if(p_buffer[ 8 ] == '.')//0000.0
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10 + ( (p_buffer[ 9 ] - 0x30 ) );	
					temp = temp/10.0;
				}
				else //00000x
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10 + ( (p_buffer[ 8 ] - 0x30 ) );	
				}			
			}
			else if( p_buffer[ 11 ] == '\n' ) //0.00000 00.0000 000.000  0000.00 00000.0\n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')//0.00000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 6 ] - 0x30 ) )*10000 + ( (p_buffer[ 7 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/100000.0;
				}
				else if(p_buffer[ 6 ] == '.')//00.0000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 7 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/10000.0;
				}
				else if(p_buffer[ 7 ] == '.')//000.000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/1000.0;
				}
				else if(p_buffer[ 8 ] == '.')//0000.00
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/100.0;
				}
				else if(p_buffer[ 9 ] == '.')//00000.0
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 8 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/10.0;
				}	
				else //00000x
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*10000 + ( (p_buffer[ 5 ] - 0x30 ) )*1000 + ( (p_buffer[ 6 ] - 0x30 ) )*100 + ( (p_buffer[ 7 ] - 0x30 ) )*10 + ( (p_buffer[ 8 ] - 0x30 ) );	
				}
			}
			else  //0.00000x 00.0000x 000.000x  0000.00x 00000.0x\n  最大值为 50000.0
			{
				if(p_buffer[ 5 ] == '.')//0.00000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 6 ] - 0x30 ) )*10000 + ( (p_buffer[ 7 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/100000.0;
				}
				else if(p_buffer[ 6 ] == '.')//00.0000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 7 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/10000.0;
				}
				else if(p_buffer[ 7 ] == '.')//000.000
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 8 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/1000.0;
				}
				else if(p_buffer[ 8 ] == '.')//0000.00
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 9 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/100.0;
				}
				else if(p_buffer[ 9 ] == '.')//00000.0
				{
					temp =  ( (p_buffer[ 4 ] - 0x30 ) )*100000 + ( (p_buffer[ 5 ] - 0x30 ) )*10000 + ( (p_buffer[ 6 ] - 0x30 ) )*1000 + ( (p_buffer[ 7 ] - 0x30 ) )*100 + ( (p_buffer[ 8 ] - 0x30 ) )*10 + ( (p_buffer[ 10 ] - 0x30 ) );	
					temp = temp/10.0;
				}			
			}
			SET_Resist  = temp*100;
			if(SET_Resist > MAX_SET_RES)
			SET_Resist = MAX_SET_RES;
		  Flag_Save_SW=1;//开启FLASH存储
			Setcontr_ADDR=21;//存储的位置	
			return 1;
		}
	}
	else  if( strncasecmp( (const char *)p_buffer, "RES?\n" ,5 )==0 ) //查询当前设置电阻
	{
		if( SET_Resist <10 ) 
			printf("%.5f\r\n" , ((float)SET_Resist/100 ));
		else if(  SET_Resist <100 ) 
			printf("%.4f\r\n" , ((float)SET_Resist/100 ));
		else if(  SET_Resist <1000 ) 
			printf("%.3f\r\n" , ((float)SET_Resist/100 ));
		else if(  SET_Resist <10000 ) 
			printf("%.2f\r\n" , ((float)SET_Resist/100 ));
		else if(  SET_Resist <100000 ) 
			printf("%.1f\r\n" , ((float)SET_Resist/100 ));	
		return 1;		
	}
	return 0;
}


/*--测量命令-------------------------------------------*/
vu8 MEAS_CMD( vu8  *p_buffer )
{
	if( strncasecmp( (const char *)p_buffer, "MEAS:" ,5 )==0 )
	{	
		if( p_buffer[ 9 ] == '\n' )//res\n
		{
			if( strncasecmp( (const char *)&p_buffer[5],  "RES?" ,4 )==0 )//
			{
				if( R_DATE <10 ) 
					printf("%.5f\r\n" , (((float)R_DATE)/1000 ));
				else if(  R_DATE <100 ) 
					printf("%.4f\r\n" , (((float)R_DATE)/1000 ));
				else if(  R_DATE <1000 ) 
					printf("%.3f\r\n" , (((float)R_DATE)/1000 ));
				else if(  R_DATE <10000 ) 
					printf("%.2f\r\n" , (((float)R_DATE)/1000 ));
				else if(  R_DATE <100000 ) 
					printf("%.1f\r\n" , (((float)R_DATE)/1000 ));	
				return 1;
			}
		}
		else if( p_buffer[ 10 ] == '\n' )
		{
			if( strncasecmp( (const char *)&p_buffer[5],  "VOLT?" ,5 )==0 )//VOLT CURR Power
			{
				if( V_Gear_SW == HIGH_Level)
				{
					if( (Voltage /1000 ) <10) //150.000 15.000 1.000
						printf("  %.3f\r\n" , ((float)Voltage )/1000);//
					else if( (Voltage /1000 ) <100) //150.000 15.000 1.000
						printf(" %.3f\r\n" , ((float)Voltage )/1000);//
					else
						printf("%.3f\r\n" , ((float)Voltage )/1000);//
					return 1;
				}
				else
				{
					if( (Voltage /10000 ) <10)
						printf(" %.4f\r\n" , ((float)Voltage )/10000);//15.0000
					else 
						printf("%.4f\r\n" , ((float)Voltage )/10000);//15.0000
					return 1;	
				}				
			}	
			else if( strncasecmp( (const char *)&p_buffer[5],  "CURR?" ,5 )==0 )
			{
				if( I_Gear_SW == HIGH_Level)
				{
					if( (Current /10000 ) <10)
						printf(" %.4f\r\n" , ((float)Current )/10000);//20.0000
					else
						printf("%.4f\r\n" , ((float)Current )/10000);//20.0000
					return 1;	
				}
				else
				{
					printf("%.5f\r\n" , ((float)Current )/100000);//2.00000
					return 1;
				}
			}
		}
		else if( p_buffer[ 11 ] == '\n' )
		{
			if( strncasecmp( (const char *)&p_buffer[5],  "POWer?" ,6 )==0 )
			{
				if( ( V_Gear_SW == HIGH_Level) &&  ( I_Gear_SW == HIGH_Level))
				{
					printf("   %.2f\r\n" , ((float)Power_DATE )/1000);
				}
				else if( ( V_Gear_SW == LOW_Level) &&  ( I_Gear_SW == LOW_Level))
				{
					printf(" %.4f\r\n" , ((float)Power_DATE )/10000);
				}
				else 
				{
					printf("  %.3f\r\n" , ((float)Power_DATE )/1000);
				}
			}
		}
		else if( p_buffer[ 14 ] == '\n' )//纹波测量
		{
			if( strncasecmp( (const char *)&p_buffer[5],  "VOLT:" ,5 )==0 )
			{
				if( strncasecmp( (const char *)&p_buffer[10],  "MAX?" ,4 )==0 )
				{
					if( V_Gear_SW == HIGH_Level )//高档0.00 低档 0.000
					{
						printf("  %.3f\r\n" , ((float)MES_VOLT_MAX )/1000);
					}
					else 
					{
						printf(" %.4f\r\n" , ((float)MES_VOLT_MAX )/10000);
					}
				}
				else if( strncasecmp( (const char *)&p_buffer[10],  "MIN?" ,4 )==0 )
				{
					if( V_Gear_SW == HIGH_Level )
					{
						printf("  %.3f\r\n" , ((float)MES_VOLT_MIN )/1000);
					}
					else 
					{
						printf(" %.4f\r\n" , ((float)MES_VOLT_MIN )/10000);
					}
				}
				else if( strncasecmp( (const char *)&p_buffer[10],  "PTP?" ,4 )==0 )
				{
					if( V_Gear_SW == HIGH_Level )
					{
						printf("  %.3f\r\n" , ((float)MES_VOLT_PTP )/1000);
					}
					else 
					{
						printf(" %.4f\r\n" , ((float)MES_VOLT_PTP )/10000);
					}
				}
				
			}
			else if( strncasecmp( (const char *)&p_buffer[5],  "CURR:" ,5 )==0 )
			{
				if( strncasecmp( (const char *)&p_buffer[10],  "MAX?" ,4 )==0 )
				{
					if( I_Gear_SW == HIGH_Level )//高档0.00 低档 0.000
					{
						printf("  %.4f\r\n" , ((float)MES_CURR_MAX )/10000);
					}
					else 
					{
						printf(" %.5f\r\n" , ((float)MES_CURR_MAX )/100000);
					}
				}
				else if( strncasecmp( (const char *)&p_buffer[10],  "MIN?" ,4 )==0 )
				{
					if( I_Gear_SW == HIGH_Level )
					{
						printf("  %.4f\r\n" , ((float)MES_CURR_MIN )/10000);
					}
					else 
					{
						printf(" %.5f\r\n" , ((float)MES_CURR_MIN )/100000);
					}
				}
				else if( strncasecmp( (const char *)&p_buffer[10],  "PTP?" ,4 )==0 )
				{
					if( I_Gear_SW == HIGH_Level )
					{
						printf("  %.4f\r\n" , ((float)MES_CURR_PTP )/10000);
					}
					else 
					{
						printf(" %.5f\r\n" , ((float)MES_CURR_PTP )/100000);
					}
				}
				
			}
		}
		else if( p_buffer[ 23 ] == '\n' )//V C P?
		{
			if( strncasecmp( (const char *)&p_buffer[5],  "VOLT?" , 5 )==0 )
			{
				if( strncasecmp( (const char *)&p_buffer[11],  "CURR?" , 5 )==0 )
				{
					if( strncasecmp( (const char *)&p_buffer[17],  "POWer?" , 6 )==0 )
					{
						if( V_Gear_SW == HIGH_Level)
						{
							if( (Voltage /1000 ) <10) //150.000 15.000 1.000
								printf("  %.3f;" , ((float)Voltage )/1000);//
							else if( (Voltage /1000 ) <100) //150.000 15.000 1.000
								printf(" %.3f;" , ((float)Voltage )/1000);//
							else
								printf("%.3f;" , ((float)Voltage )/1000);//
						}
						else
						{
							if( (Voltage /10000 ) <10)
								printf(" %.4f;" , ((float)Voltage )/10000);//15.0000
							else 
								printf("%.4f;" , ((float)Voltage )/10000);//15.0000
						}	
						//curr
						if( I_Gear_SW == HIGH_Level)
						{
							if( (Current /10000 ) <10)
								printf(" %.4f;" , ((float)Current )/10000);//20.0000
							else
								printf("%.4f;" , ((float)Current )/10000);//20.0000
						}
						else
						{
							printf("%.5f;" , ((float)Current )/100000);//2.00000
						}
						//power
						if( ( V_Gear_SW == HIGH_Level) &&  ( I_Gear_SW == HIGH_Level))
						{
							printf("   %.2f\r\n" , ((float)Power_DATE )/100);
						}
						else if( ( V_Gear_SW == LOW_Level) &&  ( I_Gear_SW == LOW_Level))
						{
							printf(" %.4f\r\n" , ((float)Power_DATE )/10000);
						}
						else 
						{
							printf("  %.3f\r\n" , ((float)Power_DATE )/1000);
						}
						return 1;
					}
				}
			}
		}
		return 1;
	}	
	return 0;
}

/*--设置动态模式参数-------------------------------------------*/
vu8 DYN_SETHIG( vu8  *p_buffer )//设置动态高准位拉载电流s
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{	
		if( strncasecmp( (const char *)&p_buffer[4], "HIGH " ,5 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[9], "MAX" ,3 )==0 )
			{
				SET_HIG_DYNCUR =DYN_HIGH_MAX;
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "MIN" ,3 )==0 )
			{
				SET_HIG_DYNCUR =DYN_HIGH_MIN;
			}
			else 
				SET_HIG_DYNCUR = Get_strNum( p_buffer , 9 ,  1 , DYN_HIGH_MAX, DYN_HIGH_MIN );
			return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "HIGH?" ,5 )==0 )
		{
			if( SET_HIG_DYNCUR < 100000)
			{
				 printf(" %.4f\r\n" , ((float)SET_HIG_DYNCUR )/10000);//
			}
			else 
			{
				printf("%.4f\r\n" , ((float)SET_HIG_DYNCUR )/10000);//
			}
			return 1;
		}
	}
	return 0;
}

vu8 DYN_SETLOW( vu8  *p_buffer )//设置动态低准位拉载电流
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{	
		if( strncasecmp( (const char *)&p_buffer[4], "LOW " ,4 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[8], "MAX" ,3 )==0 )
			{
				SET_LOW_DYNCUR =DYN_LOW_MAX;
			}
			else if( strncasecmp( (const char *)&p_buffer[8], "MIN" ,3 )==0 )
			{
				SET_LOW_DYNCUR =DYN_LOW_MIN;
			}
			else 
				SET_LOW_DYNCUR = Get_strNum( p_buffer , 8 ,  1 , DYN_LOW_MAX, DYN_LOW_MIN );
			return 1;
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "LOW?" ,4 )==0 )
		{
			if( SET_LOW_DYNCUR < 100000)
			{
				 printf(" %.4f\r\n" , ((float)SET_LOW_DYNCUR )/10000);//
			}
			else 
			{
				printf("%.4f\r\n" , ((float)SET_LOW_DYNCUR )/10000);//
			}
			return 1;
		}
	}
	return 0;
}

vu8 DYN_HIG_DELTime( vu8  *p_buffer ) //动态高准位拉载持续时间 us/
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "HIGH:" ,5 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[9], "DWELL " ,6 )==0 )
			{
				if( strncasecmp( (const char *)&p_buffer[15], "MAX\n" ,4 )==0 )
				{
					SET_HIG_DYNCURTIM =DYN_HIGH_DELLMAX;
				}
				else if( strncasecmp( (const char *)&p_buffer[15], "MIN\n" ,4 )==0 )
				{
					SET_HIG_DYNCURTIM =DYN_HIGH_DELLMIN;
				}
				else 
				{
					if( p_buffer[ 16 ] == '\n' )//15开始
					{
						SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000  ;//0\n
						return 1;
					}
					else if( p_buffer[ 17 ] == '\n' )//00\n
					{
						SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 ;
						return 1;
					}
					else if( p_buffer[ 18 ] == '\n' )//0.0\n
					{
						SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 ;
						return 1;
					}
					else if( p_buffer[ 19 ] == '\n' )//0.00 00.0\n
					{
						if( p_buffer[ 16 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 ;
							return 1;
						}
						else if( p_buffer[ 17 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 + ( p_buffer[ 18 ] - 0x30 ) *100000 ;
							return 1;
						}
					}
					else if( p_buffer[ 20 ] == '\n' )//0.000 00.00\n
					{
						if( p_buffer[ 16 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 ;
							return 1;
						}
						else if( p_buffer[ 17 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 + ( p_buffer[ 18 ] - 0x30 ) *100000 + ( p_buffer[ 19 ] - 0x30 ) *10000 ;
							return 1;
						}
					}
					else if( p_buffer[ 21 ] == '\n' )//0.0000 00.000\n
					{
						if( p_buffer[ 16 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 + ( p_buffer[ 20 ] - 0x30 ) *100 ;
							return 1;
						}
						else if( p_buffer[ 17 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 + ( p_buffer[ 18 ] - 0x30 ) *100000 + ( p_buffer[ 19 ] - 0x30 ) *10000 + ( p_buffer[ 20 ] - 0x30 ) *1000;
							return 1;
						}
					}
					else if( p_buffer[ 22 ] == '\n' )//0.00000 00.0000\n
					{
						if( p_buffer[ 16 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 + ( p_buffer[ 20 ] - 0x30 ) *100 + ( p_buffer[ 21 ] - 0x30 ) *10 ;
							return 1;
						}
						else if( p_buffer[ 17 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 + ( p_buffer[ 18 ] - 0x30 ) *100000 + ( p_buffer[ 19 ] - 0x30 ) *10000 + ( p_buffer[ 20 ] - 0x30 ) *1000 + ( p_buffer[ 21 ] - 0x30 ) *100;
							return 1;
						}
					}
					else if( p_buffer[ 23 ] == '\n' )//0.000000  00.00000\n
					{
						if( p_buffer[ 16 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 + ( p_buffer[ 20 ] - 0x30 ) *100 + ( p_buffer[ 21 ] - 0x30 ) *10 + ( p_buffer[ 22 ] - 0x30 );
							return 1;
						}
						else if( p_buffer[ 17 ] == '.' )
						{
							SET_HIG_DYNCURTIM =  ( p_buffer[ 15 ] - 0x30 ) *10000000 + ( p_buffer[ 16 ] - 0x30 ) *1000000 + ( p_buffer[ 18 ] - 0x30 ) *100000 + ( p_buffer[ 19 ] - 0x30 ) *10000 + ( p_buffer[ 20 ] - 0x30 ) *1000 + ( p_buffer[ 21 ] - 0x30 ) *100 + ( p_buffer[ 22 ] - 0x30 )*10;
							return 1;
						}
					}
				}
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "DWELL?\n" ,7 )==0 )
			{
				if( SET_HIG_DYNCURTIM <10000000)
				{
					 printf("%.6f\r\n" , ((float)SET_HIG_DYNCURTIM )/1000000);//
				}
				else if( SET_HIG_DYNCURTIM)
				{
					 printf("%.5f\r\n" , ((float)SET_HIG_DYNCURTIM )/1000000);//
				}
				return 1;
			}
		}
		return 1;
	}
	return 0;
}


vu8 DYN_LOW_DELTime( vu8  *p_buffer ) //动态低准位拉载持续时间 us/
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "LOW:" ,4 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[8], "DWELL " ,6 )==0 )
			{
				if( strncasecmp( (const char *)&p_buffer[14], "MAX\n" ,4 )==0 )
				{
					SET_LOW_DYNCURTIM =DYN_LOW_DELLMAX;
				}
				else if( strncasecmp( (const char *)&p_buffer[14], "MIN\n" ,4 )==0 )
				{
					SET_LOW_DYNCURTIM =DYN_LOW_DELLMIN;
				}
				else 
				{
					if( p_buffer[ 15 ] == '\n' )//14开始
					{
						SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000  ;//0\n
						return 1;
					}
					else if( p_buffer[ 16 ] == '\n' )//00\n
					{
						SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 ;
						return 1;
					}
					else if( p_buffer[ 17 ] == '\n' )//0.0\n
					{
						SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 ;
						return 1;
					}
					else if( p_buffer[ 18 ] == '\n' )//0.00 00.0\n
					{
						if( p_buffer[ 15 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 + ( p_buffer[ 17 ] - 0x30 ) *10000 ;
							return 1;
						}
						else if( p_buffer[ 16 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 ;
							return 1;
						}
					}
					else if( p_buffer[ 19 ] == '\n' )//0.000 00.00\n
					{
						if( p_buffer[ 15 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 + ( p_buffer[ 17 ] - 0x30 ) *10000 + ( p_buffer[ 18 ] - 0x30 ) *1000 ;
							return 1;
						}
						else if( p_buffer[ 16 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 ;
							return 1;
						}
					}
					else if( p_buffer[ 20 ] == '\n' )//0.0000 00.000\n
					{
						if( p_buffer[ 15 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 + ( p_buffer[ 17 ] - 0x30 ) *10000 + ( p_buffer[ 18 ] - 0x30 ) *1000 + ( p_buffer[ 19 ] - 0x30 ) *100 ;
							return 1;
						}
						else if( p_buffer[ 16 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000;
							return 1;
						}
					}
					else if( p_buffer[ 21 ] == '\n' )//0.00000 00.0000\n
					{
						if( p_buffer[ 15 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 + ( p_buffer[ 17 ] - 0x30 ) *10000 + ( p_buffer[ 18 ] - 0x30 ) *1000 + ( p_buffer[ 19 ] - 0x30 ) *100 + ( p_buffer[ 20 ] - 0x30 ) *10 ;
							return 1;
						}
						else if( p_buffer[ 16 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 + ( p_buffer[ 20 ] - 0x30 ) *100;
							return 1;
						}
					}
					else if( p_buffer[ 22 ] == '\n' )//0.000000  00.00000\n
					{
						if( p_buffer[ 15 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *1000000 + ( p_buffer[ 16 ] - 0x30 ) *100000 + ( p_buffer[ 17 ] - 0x30 ) *10000 + ( p_buffer[ 18 ] - 0x30 ) *1000 + ( p_buffer[ 19 ] - 0x30 ) *100 + ( p_buffer[ 20 ] - 0x30 ) *10 + ( p_buffer[ 21 ] - 0x30 );
							return 1;
						}
						else if( p_buffer[ 16 ] == '.' )
						{
							SET_LOW_DYNCURTIM =  ( p_buffer[ 14 ] - 0x30 ) *10000000 + ( p_buffer[ 15 ] - 0x30 ) *1000000 + ( p_buffer[ 17 ] - 0x30 ) *100000 + ( p_buffer[ 18 ] - 0x30 ) *10000 + ( p_buffer[ 19 ] - 0x30 ) *1000 + ( p_buffer[ 20 ] - 0x30 ) *100 + ( p_buffer[ 21 ] - 0x30 )*10;
							return 1;
						}
					}
				}
			}
			else if( strncasecmp( (const char *)&p_buffer[8], "DWELL?\n" ,7 )==0 )
			{
				if( SET_LOW_DYNCURTIM <10000000)
				{
					 printf("%.6f\r\n" , ((float)SET_LOW_DYNCURTIM )/1000000);//
				}
				else if( SET_LOW_DYNCURTIM)
				{
					 printf("%.5f\r\n" , ((float)SET_LOW_DYNCURTIM )/1000000);//
				}
				return 1;
			}
		}
		return 1;
	}
	return 0;
}

//通用电流上升下降率字符串转数字---------------------------------
// frnum 数字的首位置
// max   最大值
// min   最小值

vu16 DYN_SL_VI(vu8  *p_buffer , vu8 frnum, vu16 max , vu16 min )
{
	vu16 temp=0;
	
	if( strncasecmp( (const char *)&p_buffer[frnum ], "MAX\n" ,4 )==0 )
	{
		temp =max;
	}
	else if( strncasecmp( (const char *)&p_buffer[frnum ], "MIN\n" ,4 )==0 )
	{
		temp =min;
	}
	else 
	{
		if( p_buffer[ frnum +1 ] == '\n' )//10开始
		{
			temp =  ( p_buffer[ frnum ] - 0x30 ) *10000  ;//0\n
		}
		else if( p_buffer[ frnum +3 ] == '\n' )//0.0\n
		{
			temp =  ( p_buffer[ frnum  ] - 0x30 ) *10000 + ( p_buffer[ frnum +2 ] - 0x30 ) *1000 ;
		}
		else if( p_buffer[ frnum +4 ] == '\n' )//0.00 \n
		{
			if( p_buffer[ frnum +1 ] == '.' )
			{
				temp =  ( p_buffer[ frnum  ] - 0x30 ) *10000 + ( p_buffer[ frnum +2 ] - 0x30 ) *1000 + ( p_buffer[ frnum +3 ] - 0x30 ) *100 ;
			}
		}
		else if( p_buffer[ frnum +5 ] == '\n' )//0.000 \n
		{
			if( p_buffer[ frnum +1 ] == '.' )
			{
				temp =  ( p_buffer[ frnum  ] - 0x30 ) *10000 + ( p_buffer[ frnum +2 ] - 0x30 ) *1000 + ( p_buffer[ frnum +3 ] - 0x30 ) *100 + ( p_buffer[ frnum +4 ] - 0x30 ) *10 ;
			}
		}
		else if( p_buffer[ frnum +6 ] == '\n' )//0.0000 \n
		{
			if( p_buffer[ frnum +1 ] == '.' )
			{
				temp =  ( p_buffer[ frnum  ] - 0x30 ) *10000 + ( p_buffer[ frnum +2 ] - 0x30 ) *1000 + ( p_buffer[ frnum +3 ] - 0x30 ) *100 + ( p_buffer[ frnum +4 ] - 0x30 ) *10 + ( p_buffer[ frnum +5 ] - 0x30 ) ;
			}
		}
	}
	return temp;
	
}

vu8 DYN_SLEW( vu8  *p_buffer ) //动态模式电流斜率 上升和下降率 us/
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "SLEW " ,5 )==0 )
		{
			SET_DYN_UPKate = DYN_SL_VI( p_buffer , 9, DYN_SLEW_RISEMAX , DYN_SLEW_RISEMIN );
			SET_DYN_DOWNKate = SET_DYN_UPKate;
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "SLEW?" ,5 )==0 )
		{
			printf("%.4f" , ((float)SET_DYN_UPKate )/10000);//
			printf(", %.4f\r\n" , ((float)SET_DYN_DOWNKate )/10000);//
		}
	}
	return 0;
}

vu8 DYN_SLEW_UP( vu8  *p_buffer ) //动态模式电流斜率 上升率 us/
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "SLEW:" ,5 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[9], "RISE " ,5 )==0 )
			{
				SET_DYN_UPKate = DYN_SL_VI( p_buffer , 14, DYN_SLEW_RISEMAX , DYN_SLEW_RISEMIN );
				return 1;
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "RISE?" ,5 )==0 )
			{
				printf("%.4f\r\n" , ((float)SET_DYN_UPKate )/10000);//
				return 1;
			}
		}
	}
	return 0;
}

vu8 DYN_SLEW_DOWN( vu8  *p_buffer ) //动态模式电流斜率 下降率 us/
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "SLEW:" ,5 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[9], "FALL " ,5 )==0 )
			{
				SET_DYN_DOWNKate = DYN_SL_VI( p_buffer , 14, DYN_SLEW_FALLMAX , DYN_SLEW_FALLMIN );
				return 1;
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "FALL?" ,5 )==0 )
			{
				printf("%.4f\r\n" , ((float)SET_DYN_DOWNKate )/10000);//
				return 1;
			}
		}
	}
	return 0;
}

vu8 SET_DYN_MODE( vu8  *p_buffer ) //设置动态模式下的工作模式
{
	if( strncasecmp( (const char *)p_buffer, "DYN:" ,4 )==0 )
	{
		if( strncasecmp( (const char *)&p_buffer[4], "MODE " ,5 )==0 )
		{
			if( strncasecmp( (const char *)&p_buffer[9], "PULS" ,4 )==0 )
			{
				SET_DYN_WorkMode = DYN_PULS;
				return 1;
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "CONT" ,4 )==0 )
			{
				SET_DYN_WorkMode = DYN_CONT;
				return 1;
			}
			else if( strncasecmp( (const char *)&p_buffer[9], "TOGG" ,4 )==0 )
			{
				SET_DYN_WorkMode = DYN_TOGG;
				return 1;
			}
		}
		else if( strncasecmp( (const char *)&p_buffer[4], "MODE?" ,5 )==0 )
		{
		  if(SET_DYN_WorkMode == DYN_PULS)
				printf("PULS\r\n" );
			else if(SET_DYN_WorkMode == DYN_CONT)
				printf("CONT\r\n" );
			else if(SET_DYN_WorkMode == DYN_TOGG)
				printf("TOGG\r\n" );	
			return 1;
		}
	}
	return 0;
}

/******************************************************************
 * 函数名称    :
 * 输入        :
 * 输出        :
 * 返回值      :
 * = 0        :
 * = -1        :
*******************************************************************/
void Me_SCPI_TASK(void)
{
	if(flag_NOR_CODE==1)
	{
		flag_NOR_CODE=0;
		CLS( UART_Buffer_Rece );
		ESE( UART_Buffer_Rece );
		QUESE( UART_Buffer_Rece );
		IDN( UART_Buffer_Rece );
		OPC( UART_Buffer_Rece );
		RST( UART_Buffer_Rece );
		STB( UART_Buffer_Rece );
		ERR( UART_Buffer_Rece );
		VERS( UART_Buffer_Rece );
		SENS( UART_Buffer_Rece );
		QSENS( UART_Buffer_Rece );
		//BEER( UART_Buffer_Rece );
		//QBEER( UART_Buffer_Rece );
		LOCAL( UART_Buffer_Rece );
		REMT( UART_Buffer_Rece );
		REML( UART_Buffer_Rece );
		QUESTEVT( UART_Buffer_Rece );
		QUESTST( UART_Buffer_Rece );
		OPERT( UART_Buffer_Rece );
		OPERTCP( UART_Buffer_Rece );
		OPERTsT( UART_Buffer_Rece );
		REOPERTCP( UART_Buffer_Rece );
		ONOFF( UART_Buffer_Rece );
		QUONOFF( UART_Buffer_Rece );
		ShortONOFF( UART_Buffer_Rece );
		QUShortONOFF( UART_Buffer_Rece );
		SET_CURR_Level( UART_Buffer_Rece );
		QU_CURR_Level( UART_Buffer_Rece );
		SET_VOLT_Level( UART_Buffer_Rece );
		QU_VOLT_Level( UART_Buffer_Rece );
		SET_CURR_SLEW( UART_Buffer_Rece );
		SET_CURR_SLEW_RISE( UART_Buffer_Rece );
		SET_CURR_SLEW_FALL( UART_Buffer_Rece );
		SET_VOLT_SLEW_FALL( UART_Buffer_Rece );
		SET_CURR_PROT( UART_Buffer_Rece );
		SET_POW_PROT( UART_Buffer_Rece );
		SET_LoadST_VOLT( UART_Buffer_Rece );
		SET_FreeLoad_VOLT( UART_Buffer_Rece );
		SET_Load_MODE( UART_Buffer_Rece );
		SET_CC_CURR( UART_Buffer_Rece );
		SET_CV_VOLT( UART_Buffer_Rece );
		SET_POW_VAL( UART_Buffer_Rece );
		SET_RES_VAL( UART_Buffer_Rece );
		MEAS_CMD( UART_Buffer_Rece );
//		DYN_SETHIG( UART_Buffer_Rece );
//		DYN_SETLOW( UART_Buffer_Rece);
//		DYN_HIG_DELTime( UART_Buffer_Rece );
//		DYN_LOW_DELTime( UART_Buffer_Rece) ;
//		DYN_SLEW( UART_Buffer_Rece );
//		DYN_SLEW_UP( UART_Buffer_Rece );
//		DYN_SLEW_DOWN( UART_Buffer_Rece ) ;
//		SET_DYN_MODE( UART_Buffer_Rece );
		
		Clear_Date(UART_Buffer_Rece, 60);
  }		
}

/*----------------------------------COPY RIGHT BY XQ_GUO @2019---------------------------------------------*/

