/******************** (C) COPYRIGHT 2015 AVER ********************
 * 文件名  ：DAC.C
 * 作者    ：李振
 * 描述    ：库函数版
 * 内容    ：外置16BIT DAC
 * 硬件连接：
 * 修改日期：2015-8-18
*********************************************************************/
#include "my_register.h" 
#include "dac.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/**
  * @brief  使能DAC的时钟，初始化GPIO
  * @param  无
  * @retval 无
  */
/********************************************************************/
#define DAC_SYNC_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_10);
#define DAC_SYNC_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_10);   
      
#define DAC_SCK_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_8);
#define DAC_SCK_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_8); 

#define DAC_SDA_LOW   GPIO_ResetBits(GPIOA, GPIO_Pin_9);
#define DAC_SDA_HIGH  GPIO_SetBits(GPIOA, GPIO_Pin_9); 
/**********************************************************************************************************
*	函 数 名: AD5541_GPIOCoing
*	功能说明: 引脚初始化
*	形    参：
*	返 回 值: 无
**********************************************************************************************************/
void AD5541_GPIOCoing(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;//调用GPIO结构体
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_2MHz;//
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
	
}
/**********************************************************************************************************
*	函 数 名: AD5541_GPIOCoing
*	功能说明: 引脚初始化
*	形    参：
*	返 回 值: 无
**********************************************************************************************************/
void AD5541_Send(vu16 date)
{
	vu8 cont;
	DAC_SYNC_LOW;//SYNC置低
	DAC_SCK_HIGH;//SCK置高
	NOP;
	NOP;
	NOP;
	NOP;
	DAC_SYNC_HIGH;//SYNC置高
	NOP;
	DAC_SCK_HIGH;//SCK置高
	NOP;
	NOP;
	DAC_SYNC_LOW;//SYNC置低
	for(cont = 0; cont < 16; cont++)
	{
		if(date & 0x8000)		// 先传高位
		{
			DAC_SDA_HIGH;
		}
		else
		{
			DAC_SDA_LOW;
		}
		DAC_SCK_LOW;//SCK置低
		NOP;
		NOP;
		DAC_SCK_HIGH;//SCK置高
		NOP;
		NOP;
		date=date<<1;
	}
	DAC_SYNC_HIGH;//SYNC置高
	DAC_SCK_HIGH;//SCK置高
}
