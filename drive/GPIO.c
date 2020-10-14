/******************************************************************/
/* 函数名：                                                */
/* 功能：                                              */
/* 修改日期：                                                       */
/* 内容：                                                    */
/* 作者：QQ:363116119                                        */
/******************************************************************/

#include "gpio.h"
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void GPIO_Conf(void)
{
/**********************GPIO*******************************************************/
	GPIO_InitTypeDef GPIO_InitStruct;//
	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_AFIODeInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);//使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//开启路由时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);//禁用JTAG
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//禁用JTAG
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_11;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_50MHz;//
	GPIO_Init(GPIOA, &GPIO_InitStruct);//
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;//LVR
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//
	GPIO_Init(GPIOA, &GPIO_InitStruct);//


	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8;//
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//
	GPIO_InitStruct.GPIO_Speed =GPIO_Speed_50MHz;//
	GPIO_Init(GPIOB, &GPIO_InitStruct);//
}
