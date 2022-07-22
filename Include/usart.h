/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * 名称    :usart.h
 * 作者    :李振
 * 描述    :
 * 软件功能:
 * 硬件外设:
 * 修改日期:2015-2-5
*******************************************************************/
	

#ifndef _usart_h_
#define _usart_h_

#include "stm32f10x.h"

void USART_Configuration(void);//串口1初始化函数
void USART2_Configuration(void);//串口2初始化函数
void USART3_Configuration(void);//串口3初始化函数
void UART1_Send(void);
void UART2_Send(void);
void UART3_Send(void);
vu8 Usart1_Sendstring(char *s );
static void USART1_NVIC_Config(void);
void Clear_Date(vu8 *sd , vu8 length);
void Baud_SET(void);
#endif
	