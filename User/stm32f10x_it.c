/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "my_register.h"
#include "AD7689.h"
#include "modbus.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
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
} flagA,flagB,flagC,flagD,flagE,flagF;

extern __IO int32_t OS_TimeMS;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
	if(flag_Tim_USART==1)//串口清零计数
	{
		t_USART++;
	}
	if(t_USART>20)//大约2.6ms
	{
		t_USART=0;
		flag_Tim_USART=0;
		UART_Buffer_Size=0;	
	}
}

void USART1_IRQHandler(void)
{
	flag_Tim_USART=1;
 	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit( USART1, USART_IT_RXNE );
		UART_Buffer_Rece[UART_Buffer_Size]=USART_ReceiveData(USART1);

		if( UART_Buffer_Rece[0]== 0x01)//校准用
		{
			if(UART_Buffer_Size>8)//校准用
			{
				UART_Buffer_Size=0;	  	   		   
				UART_Buffer_Rece_flag=1;
				flag_Tim_USART=0;
				t_USART=0;
				return ;
			}
		}
		else if(UART_Buffer_Rece[UART_Buffer_Size] == 0x0A) //scpi指令用 判断尾指令是否为\n
		{
			flag_NOR_CODE=1;
			UART_Buffer_Size=0;
			flag_Tim_USART=0;				
			t_USART=0;	
			return ;
		}
		
		UART_Buffer_Size++;
		if(UART_Buffer_Size>200)
		{
			UART_Buffer_Size=0;
			UART_Buffer_Rece_flag=0;
			t_USART=0;	
		}

	}
}

void USART2_IRQHandler(void)
{
	flag_Tim_USART=1;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit( USART2, USART_IT_RXNE );
		UART_Buffer_Rece[UART_Buffer_Size]=USART_ReceiveData(USART2);

		if( UART_Buffer_Rece[0]== 0x01)
		{
			if(UART_Buffer_Rece[1]== 0x06)
			{
				if(UART_Buffer_Size>8)//设置参数
				{
					UART_Buffer_Size=0;	  	   		   
					UART_Buffer_Rece_flag=1;
					flag_Tim_USART=0;
					t_USART=0;
					if(UART_Buffer_Rece_flag==1)
					{
						UART_Buffer_Rece_flag=0;
						UART_Action();//处理数据
					}
					return ;
				}
			}else if(UART_Buffer_Rece[1]== 0x10){
				if(UART_Buffer_Size==132+28)//参数设置
				{
					UART_Buffer_Size=0;	  	   		   
					UART_Buffer_Rece_flag=1;
					flag_Tim_USART=0;
					t_USART=0;
					if(UART_Buffer_Rece_flag==1)
					{
						UART_Buffer_Rece_flag=0;
						UART_Action();//处理数据
					}
					return ;
				}
			}else if(UART_Buffer_Rece[1]== 0x03){
				if(UART_Buffer_Size==7)//读数据
				{
					UART_Buffer_Size=0;	  	   		   
					UART_Buffer_Rece_flag=1;
					flag_Tim_USART=0;
					t_USART=0;
					if(UART_Buffer_Rece_flag==1)
					{
						UART_Buffer_Rece_flag=0;
						UART_Action();//处理数据
					}
					return ;
				}
			}else if(UART_Buffer_Rece[1]== 0xA5){
				if(UART_Buffer_Size==8)//校准
				{
					UART_Buffer_Size=0;	  	   		   
					UART_Buffer_Rece_flag=1;
					flag_Tim_USART=0;
					t_USART=0;
					if(UART_Buffer_Rece_flag==1)
					{
						UART_Buffer_Rece_flag=0;
						UART_Action();//处理数据
					}
					return ;
				}
			}
		}else if( UART_Buffer_Rece[0]== 0x00)//写入地址
		{
			if(UART_Buffer_Size>8)//地址
			{
				UART_Buffer_Size=0;	  	   		   
				UART_Buffer_Rece_flag=1;
				flag_Tim_USART=0;
				t_USART=0;
				if(UART_Buffer_Rece_flag==1)
				{
					UART_Buffer_Rece_flag=0;
					UART_Action();//处理数据
				}
				return ;
			}
		}
//		else if(UART_Buffer_Rece[UART_Buffer_Size] == 0x0A) //scpi指令用 判断尾指令是否为\n
//		{
//			flag_NOR_CODE=1;
//			UART_Buffer_Size=0;
//			flag_Tim_USART=0;				
//			t_USART=0;	
//			return ;
//		}
		

		UART_Buffer_Size++;
		if(UART_Buffer_Size>200)
		{
			UART_Buffer_Size=0;
			UART_Buffer_Rece_flag=0;
			t_USART=0;	
		}
		
	}
}
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit( USART3, USART_IT_RXNE );
	}
}

void TIM4_IRQHandler(void)
{	
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//清除中断标志位
	if(TIME_1MS_OVER==0)//开启爬升或者下降计时标志
	{
		TIME_1MS_flag=1;//1MS定时标志
	}
	else 
	{
		TIME_1MS_flag=0;
	}
}
void DMA1_Channel1_IRQHandler(void)
{
	Flag_ADC_Full=1;//一个通道采集完成标志
	DMA_ClearITPendingBit(DMA1_IT_TC1);//清除中断标志位
}

void DMA1_Channel2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_IT_TC2);//清除中断标志位
	Flag_AD7655_DMA=1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
