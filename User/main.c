/******************************************************************/
/* 名称：main                                                    */
/* 效果：                                                        */
/* 内容：                                                       */
/* 作者：zhan                                                  */
/* 联系方式QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "stm32f10x.h"
#include "tim6.h"
#include "modbus.h"
#include "dac.h"
#include "AD7689.h"
#include "menu.h"
#include "adc.h"
#include "clock.h"
#include "gpio.h"
#include "bsp_SysTick.h"
#include "usart.h"
#include "iwdg.h"
#include "IIC_24C01.h"
#include "flash.h"
#include "beep.h"
#include "sys_io_cfg.h"
#include "me_scpi.h"
#include "FAN_PRO.h"

struct bitDefine
{
    unsigned bit0: 1;
    unsigned bit1: 1;
    unsigned bit2: 1;
    unsigned bit3: 1;
    unsigned bit4: 1;
    unsigned bit5: 1;
    unsigned bit6: 1;
    unsigned bit7: 1;
} flagA, flagB,flagC,flagD,flagE,flagF,flagG;
vu16 date_dac;
RCC_ClocksTypeDef getrccclock;
int main(void)
{
	RCC_Configuration();
	SysTick_Init();
	GPIO_Conf();
	AD5541_GPIOCoing();
	AD7689_InitializeSPI2();

	ADC1_DMA_Init();
	i2c_CfgGpio();
	EEPROM_READ_Coeff();//读取校准参数
	USART_Configuration();
	USART2_Configuration();
	TIM6_Config();
	IWDG_Inte();
//	IO_FAN_OFF;
	ADDR = 1;
	while(1)
	{
//		RCC_GetClocksFreq(&getrccclock);
		IWDG_ReloadCounter(); //喂狗
		if(UART_Buffer_Rece_flag==1)
		{
			UART_Buffer_Rece_flag=0;
			UART_Action();//接收一帧数据
			Baud_SET();//设置串口波特率
//			MAXPAR_limit();//运行参数最大值限制
		}
		if(UART1_Buffer_Rece_flag==1)
		{
			UART1_Buffer_Rece_flag=0;
			UART1_Action();//接收一帧数据
//			Baud_SET();//设置串口波特率
//			MAXPAR_limit();//运行参数最大值限制
		}
//		Me_SCPI_TASK(); //SCPI串口任务
//		Wite_Runcont();//将运行参数写入EEPROM
		AD5541_Send(Contr_DACVlue);//设置DAC值
		AD7689_Scan_CH();//读取AD7689 AD值
		if(DAC_Flag == 0x00)
		{		
			Transformation_ADC(); 
		    All_protect();//各种保护			
		}
		if(Flag_ADC_Full==1) //NTC
		{
			Flag_ADC_Full=0;
			ADC_CH_Scan();
		}
		Temp_Comapre();//风扇
		worke_mode();//工作模式切换
	}
}


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
