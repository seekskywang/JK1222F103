/******************************************************************/
/* 名称:iwdg.c                                                  */
/* 效果：                                                        */
/* 内容： 看门狗初始化                                                       */
/* 作者：李振                                                    */
/* 联系方式：QQ:363116119                                        */
/******************************************************************/
#include "stm32f10x_iwdg.h"

/*****************************************************/
void IWDG_Inte(void)
{
	/* 设置重导计数器的值 获得250ms的IWDG超时可以按下面算式计算：  
     Counter Reload Value = 250ms/IWDG counter clock period
                          = 250ms / (LSI/32)
                          = 0.25s / (0xff/32)
                          = LsiFreq/(32 * 4)
                          = LsiFreq/128
   */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(0xff);
  IWDG_ReloadCounter();
	IWDG_Enable();
}
