/******************************************************************/
/* 函数名：                                                */
/* 功能：                                              */
/* 修改日期：                                                       */
/* 内容：                                                    */
/* 作者：QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/********************************************************************/
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
} flagA, flagB,flagC;
/****************************************************************/
vu8 Beep_Flag;
vu16 t_0_5S;
vu16 t_1S;
vu16 t_5S;
vu16 t_Basket;
vu16 Basket_V;
vu16 Basket_A;
vu16 t_OFF_V;
vu16 t_ON_V;
vu16 t_OFF_A;
vu16 t_ON_A;
/****************************************************************/
void BEEP_Ring(void)//蜂鸣器控制
{
//	if(Flag_Beep_OFF==0)
//	{
//		if(Flag_BEEP==1)
//		{
//			if(Flag_0_5S==0)
//			{
//				GPIO_SetBits(GPIOA,GPIO_Pin_5);
//			}
//			else 
//			{
//				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
//				Flag_BEEP=0;
//			}
//		}
//	}
}
/***************************************************************/
void BEEP_Tiggr(void)//蜂鸣器触发
{
	Flag_BEEP=1;
	t_0_5S=0;
//Flag_0_5S=0;
}
