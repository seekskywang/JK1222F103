/******************************************************************/
/* 名称uasrt                                           */
/* 效果：                                                        */
/* 内容：                                                        */
/* 作者：李振                                                    */
/* 联系方式：QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "string.h"
#include "stdio.h"
#include "string.h"

/*****************************************************************/
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
} flagA,flagB,flagC,flagD;
vu8 UART_Buffer_Rece[200];
vu8 UART_Buffer_Send[200];
vu8 UART_Buffer_Rece1[200];
vu8 UART_Buffer_Send1[200];
vu8 UART_Buffer_Size;
vu8 Transmit_BUFFERsize;
vu8 t_USART;
vu32 Baud=115200;
/*****************************************************************/
static void USART1_NVIC_Config(void)//串口接收中断配置
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void USART_Configuration(void)//串口初始化函数
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = Baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART1_NVIC_Config();//串口中断配置
	
	/******使能串口接收中断******************/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART1, ENABLE);//使能串口1
  USART_ClearFlag(USART1, USART_FLAG_TC);
  
}			

void USART2_Configuration(void)//串口初始化函数
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.03) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART1_NVIC_Config();//串口中断配置
	
	/******使能串口接收中断******************/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART2, ENABLE);//使能串口1
    USART_ClearFlag(USART2, USART_FLAG_TC);
  
}


void USART3_Configuration(void)//串口初始化函数
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART1_NVIC_Config();//串口中断配置
	
	/******使能串口接收中断******************/
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART3, ENABLE);//使能串口1
  USART_ClearFlag(USART3, USART_FLAG_TC);
  
}


void UART1_Send(void)
{
	vu8 i=0;
	for(i =0 ; i < Transmit_BUFFERsize; i++)
  {
			USART_SendData(USART1,UART_Buffer_Send1[i]);
			while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);//等待发送完成
  }
	UART_SEND_flag=0;//发送完成一帧数据		
}
void UART2_Send(void)
{
	vu8 i=0;
	for(i =0 ; i < Transmit_BUFFERsize; i++)
  {
			USART_SendData(USART2,UART_Buffer_Send[i]);
			while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);//等待发送完成
  }
	UART_SEND_flag=0;//发送完成一帧数据		
}
void UART3_Send(void)
{
	vu8 i=0;
	for(i =0 ; i < Transmit_BUFFERsize; i++)
  {
			USART_SendData(USART3,UART_Buffer_Send1[i]);
			while (USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);//等待发送完成
  }
	UART_SEND_flag=0;//发送完成一帧数据		
}
/**********************************************************************************************************
*	函 数 名: Baud_SET
*	功能说明: 串口波特率修改
*	形    参:  无
*	返 回 值: 无
**********************************************************************************************************/
void Baud_SET(void)//设置串口波特率
{
//	if(UART_Buffer_Rece[1] == 0X06)
//	{
//		if(UART_Buffer_Rece[3] == 17)//判断写寄存器地址是否为波特率
//		{
			if(Baud_rate==0)
			{
				Baud=4800;
			}else if(Baud_rate==1)
			{
				Baud=9600;
			}
			else if(Baud_rate==2)
			{
				Baud=14400;
			}
			else if(Baud_rate==3)
			{
				Baud=19200;
			}
			else if(Baud_rate==4)
			{
				Baud=115200;
			}
//			else if(Baud_rate==115200)
//			{
//				Baud_rate=115200;
//			}
//			else if(Baud_rate==128000)
//			{
//				Baud_rate=128000;
//			}
			else 
			{
				Baud=9600;
			}
			USART_Configuration();
//		}
//	}
//	if(UART_Buffer_Rece[1] == 0X10)
//	{
//		if(UART_Buffer_Rece[3] < 17)//判断写寄存器地址是否为波特率
//		{
//			if(Baud_rate==0)
//			{
//				Baud=4800;
//			}else if(Baud_rate==1)
//			{
//				Baud=9600;
//			}
//			else if(Baud_rate==2)
//			{
//				Baud=14400;
//			}
//			else if(Baud_rate==3)
//			{
//				Baud=19200;
//			}
//			else if(Baud_rate==4)
//			{
//				Baud=115200;
//			}
////			else if(Baud_rate==115200)
////			{
////				Baud_rate=115200;
////			}
////			else if(Baud_rate==128000)
////			{
////				Baud_rate=128000;
////			}
//			else 
//			{
//				Baud=9600;
//			}
//			USART_Configuration();
//		}
//	}
}

/*******************************************************************************
* Function Name  : DMA_Configuration					功能名称：使用USART1发送一串字符
* Description    : Configures the DMA.				描述		：
* Input          : None												输入		：无
* Output         : None												输出		：无
* Return         : None												返回		: 无
*******************************************************************************/ 
vu8 Usart1_Sendstring(char *s )
{
	vu8 i;
	vu8 length;
	length = strlen(s);
	
	for( i=0; i<length ; i++ ) //为字符串结束符 '\0'就退出
	{		
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  
		USART_SendData(USART1, s[i]); 
	}
	return 1;
}

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//////////////////////////////////////////////////////////////
void Clear_Date(vu8 *sd , vu8 length)
{
	vu8 i;
	for( i=0; i<length ; i++ ) //清除数据
	{		
		sd[i]=0;
	}
}
