/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * 名称    :ADC.h
 * 作者    :李振
 * 描述    :
 * 软件功能:
 * 硬件外设:
 * 修改日期:2015-2-5
*******************************************************************/
	

#ifndef _adc_h_
#define _adc_h_

void ADC1_DMA_Init(void);
void DAM1_ADC_NVIC(void);
void ADC_CH_Scan(void);//滤波
#endif
