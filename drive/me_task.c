
/******************** (C) COPYRIGHT 2019 shenzhen********************
 * 名称    :_ME_TASK
 * 作者    : Xq_Guo
 * 描述    :
 * 软件功能:
 * 硬件外设:
 * 修改日期:2019-4-15
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
#include "me_loadtask.h"


/*--------------------------------------------END CONFIG--------------------------------------------------*/

void	me_LoadTask(void)
{
  u32  V_tempSet=0;
 
	switch( LOAD_WorkMode )
	{
		case  CURR_MODE:
		
			break;
		case  VOLT_MODE:
		
			break;
		case  POW_MODE:
		
			break;
		case  RES_MODE:
		
			break;	
		case  LED_MODE:
		
			break;	
	}
}


/*----------------------------------COPY RIGHT BY XQ_GUO @2019---------------------------------------------*/


