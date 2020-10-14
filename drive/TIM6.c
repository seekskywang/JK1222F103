/******************************************************************/
/* 名称TIM3 PWM                                                */
/* 效果：                                                        */
/* 内容：产生一个200HZ 正占空比：60.9% 负占空比：30.9%的PWM      */
/* 作者：李振                                                    */
/* 联系方式：QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x.h"
#include "tim6.h"
/*****************************************************************/
/*****************************************************************/
void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM3 的配置 ---------------------------------------------------
   TIM3 输入时钟(TIM3CLK) 设置为 APB2 时钟 (PCLK2)    
    => TIM3CLK = PCLK2 = SystemCoreClock
   TIM3CLK = SystemCoreClock, Prescaler = 0, TIM3 counter clock = SystemCoreClock
   SystemCoreClock 为48 MHz */
  /* TIM16 时钟使能 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
  /* Time 定时基础设置*/
  TIM_TimeBaseStructure.TIM_Prescaler = 2000;//时钟预分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time 定时设置为上升沿计算模式*/
  TIM_TimeBaseStructure.TIM_Period = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启定时器更新中断
	TIM_SetAutoreload(TIM4, 0xFF);//设置PWM分辨率
	TIM6_NVIC_Config();
  /* TIM3 计算器使能*/
  TIM_Cmd(TIM4, ENABLE);
}
/***********************************************************************/
static void TIM6_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
