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
#include "scpi/scpi.h"
#include "scpi-def.h"

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

size_t SCPI_Write(scpi_t * context, const char * data, size_t len) {
  (void) context;	
	if (len > 0)
	{	
		for(int i =0 ; i < len; i++)
		{
				USART_SendData(USART1,*(data+i));
				while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);//等待发送完成
		}
//		for (int i = 0;i < len;i++)
//		{
//			osMessageQueuePut(cmdTxQueueHandle,&data[i],NULL,0U);
//		}
//		LL_USART_EnableIT_TXE(USART1);
	}
	return len;
}

scpi_result_t SCPI_Flush(scpi_t * context) {
    (void) context;

    return SCPI_RES_OK;
}

int SCPI_Error(scpi_t * context, int_fast16_t err) {
    (void) context;

//    printf("**ERROR: %d, \"%s\"\r\n", (int16_t) err, SCPI_ErrorTranslate(err));
    return 0;
}

scpi_result_t SCPI_Control(scpi_t * context, scpi_ctrl_name_t ctrl, scpi_reg_val_t val) {
    (void) context;

    if (SCPI_CTRL_SRQ == ctrl) {
//        printf("**SRQ: 0x%X (%d)\r\n", val, val);
    } else {
//        printf("**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
    }
    return SCPI_RES_OK;
}

scpi_result_t SCPI_Reset(scpi_t * context) {
    (void) context;

//    printf("**Reset\r\n");
    return SCPI_RES_OK;
}

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t * context) {
    (void) context;

    return SCPI_RES_ERR;
}

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
	USART3_Configuration();
	TIM6_Config();
	IWDG_Inte();
	SCPI_Init(&scpi_context,
					scpi_commands,
					&scpi_interface,
					scpi_units_def,
					SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
					scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH,
					scpi_error_queue_data, SCPI_ERROR_QUEUE_SIZE);
//	IO_FAN_OFF;
//	ADDR = 1;
	while(1)
	{
//		RCC_GetClocksFreq(&getrccclock);
		IWDG_ReloadCounter(); //喂狗
//		if(UART_Buffer_Rece_flag==1)
//		{
//			UART_Buffer_Rece_flag=0;
//			UART_Action();//接收一帧数据
//			Baud_SET();//设置串口波特率
////			MAXPAR_limit();//运行参数最大值限制
//		}
//		if(UART1_Buffer_Rece_flag==1)
//		{
//			UART1_Buffer_Rece_flag=0;
//			UART1_Action();//接收一帧数据
////			Baud_SET();//设置串口波特率
////			MAXPAR_limit();//运行参数最大值限制
//		}
//		Me_SCPI_TASK(); //SCPI串口任务
//		Wite_Runcont();//将运行参数写入EEPROM
		AD5541_Send(Contr_DACVlue);//设置DAC值
		AD7689_Scan_CH();//读取AD7689 AD值
		if(DAC_Flag == 0x00)
		{		
			Transformation_ADC(); 
			if(SWDelay == 0)
			{
				All_protect();//各种保护	
			}else if(SWDelay > 0){
				SWDelay--;
			}
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
