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
iapfun jump2app; 
u8 bootflag;
vu32 ilow1[TOTALVERSION] = {1000,30000,30000,60000,60000,1000,1000,1000,30000,30000,30000,100000};
vu32 ilow2[TOTALVERSION] = {30000,60000,60000,100000,100000,20000,30000,30000,60000,60000,60000,200000};
vu32 ilow3[TOTALVERSION] = {70000,130000,130000,240000,240000,50000,70000,70000,130000,130000,160000,600000};

vu32 ihigh1[TOTALVERSION] = {100000,200000,200000,240000,240000,100000,100000,100000,200000,200000,200000,600000};
vu32 ihigh2[TOTALVERSION] = {200000,400000,400000,600000,600000,150000,200000,200000,400000,400000,400000,1000000};
vu32 ihigh3[TOTALVERSION] = {300000,600000,600000,1000000,1000000,200000,300000,300000,600000,600000,600000,2000000};
vu32 ihigh4[TOTALVERSION] = {400000,800000,800000,2000000,2000000,300000,400000,400000,800000,800000,800000,3000000};

vu32 vlowmax[TOTALVERSION] = {250000,150000,250000,150000,150000,150000,150000,150000,150000,150000,150000,150000};
vu32 vhigmax[TOTALVERSION] = {250000,150000,250000,150000,150000,150000,150000,150000,150000,150000,150000,150000};
vu32 ilowmax[TOTALVERSION] = {60000,120000,120000,240000,240000,40000,60000,60000,120000,120000,150000,500000};
vu32 powermax[TOTALVERSION] = {1200000,1200000,1200000,3200000,2400000,400000,600000,800000,1200000,2400000,4000000,6400000};
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

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



//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(u32 appxaddr)
{
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(vu32*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
}		

int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
	__enable_irq();
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
	BOOTMODE=0;
	Write_bootmode();
	MES_VOLT_MAX=14;//硬件版本
	//1.3增加modbusrtu标准协议选择
	//1.4读取命令标准协议CRC问题修正
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
		if(BOOTLOAD == 1)
		{
			bootflag=1;
			Write_bootflag();
			__disable_irq();
			iap_load_app(FLASH_BOOT_ADDR);
		}
	}
}


/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
