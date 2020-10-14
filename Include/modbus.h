/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * 名称    :modbus.h
 * 作者    :李振
 * 描述    :
 * 软件功能:
 * 硬件外设:
 * 修改日期:2015-2-5
*******************************************************************/
#include "my_register.h"	

#ifndef _modbus_h_
#define _modbus_h_

void UART_Action(void);
void Transformation_ADC(void);
vu16 Hardware_CRC(vu8 *p_buffer,vu8 count);
#endif
