/******************************************************************/
/* 函数名：                                                    */
/* 函数功能：                                                     */
/* 返回值：                                                     */
/* 作者：                                                 */
/* 联系方式:                                            */
/******************************************************************/
#include "adc.h"
#include "my_register.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
/*****************************************************************/
#define ADC1_DR_Address       0x4001244C
vu16 NTC_value;
vu16 NTC1_value;
vu16 Imon_value;
vu16 Vmon_value;
vu16 Contr_Laod;//
vu16 ADC1_Buffer[100];
vu16 ADC_NTC_Filt[50];
vu16 ADC_NTC1_Filt[50];
/*****************************************************************/
void ADC1_DMA_Init(void)
{
  ADC_InitTypeDef     ADC_InitStructure;
  DMA_InitTypeDef     DMA_InitStruct;
	GPIO_InitTypeDef    GPIO_InitStruct;
	  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);

  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

  /* Configure PA.0  as analog input */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStruct);	
	/* Configure PA.0  as analog input */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStruct.DMA_PeripheralBaseAddr = (vu32)ADC1_DR_Address;
  DMA_InitStruct.DMA_MemoryBaseAddr = (vu32)&ADC1_Buffer;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStruct.DMA_BufferSize =100;//连续转换100次
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStruct);
  
  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);
	/* 寮�鍚疍AM1閫氶亾1涓柇 */
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
  DAM1_ADC_NVIC();//DMA中断配置
  ADC_DMACmd(ADC1, ENABLE);  //使能DMA
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 				//
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//
	ADC_InitStructure.ADC_NbrOfChannel = 2;	 								//
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
  /* ADC Calibration */
  ADC_Cmd(ADC1, ENABLE);//使能ADC
	 
	ADC_ResetCalibration(ADC1);
	
	ADC_StartCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);   //开始转换
}
/***************************************************************/
void DAM1_ADC_NVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the adc1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************/
void ADC_CH_Scan(void)//滤波
{
	vu8 count,i;
	static vu8 a,b;
	vu32 sum;
	for(i=0;i<100;i++)
	{
		if ((i % 2) == 0)
		{
			ADC_NTC_Filt[a++]=ADC1_Buffer[i];
		}
		else if((i % 2) ==1 )
		{
			ADC_NTC1_Filt[b++]=ADC1_Buffer[i];
		}	
	}
	a=0;
	b=0;
/**********NTC**********************/
  sum=0;
	for(count=0;count<50;count++)
	{
		sum +=ADC_NTC_Filt[count];
	}
	NTC_value=sum/50;
/**********NTC1**********************/	
	sum=0;
	for(count=0;count<50;count++)
	{
		sum +=ADC_NTC1_Filt[count];
	}
	NTC1_value=sum/50;
}
