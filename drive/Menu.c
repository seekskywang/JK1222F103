/******************** (C) COPYRIGHT 2019 AVER********************
 * 文件名  :MENU.C
 * 作者   :
 * 描述    :
 * 内容    :
 * 硬件连接: 
 * 修改日期:
********************************************************************/
#include "my_register.h" 
#include "usart.h" 
#include "modbus.h" 
#include "stm32f10x.h"
#include "flash.h"
#include "menu.h"
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
} flagA,flagB,flagC,flagD,flagE;
vu8 Von_CONT;//拉载开启门槛单次计数器


/***************************************
函数名:Sence_SW_CONT
函数输入:
函数功能:远端测量控制
****************************************/
void Sence_SW_CONT(void)
{
	if(Sence_SW==1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);//开启远端测量
	}
	else 
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);//关闭远端测量
	}
}
/***************************************
函数名:I_SW_COTNR
函数输入:
函数功能:电流档位切换控制
****************************************/
void I_SW_COTNR(void)
{
	if(I_Gear_SW==1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);//电流测量为高档位
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);//电流测量为低档位
	}
	if(I_Gear_SW==0)
	{
		if(Current>I_LOW_MAX)//当测量电流大于低档最高电流时自动切换成高档位且锁定，需手动切换才能变成低档位
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_1);//电流测量为高档位
			I_Gear_SW=1;//自动切换为高档位
		}
	}
}
/***************************************
函数名:V_SW_COTNR
函数输入:
函数功能:电压档位切换
****************************************/
void V_SW_COTNR(void)
{
	if(V_Gear_SW==0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_11);//电压档位为低档 0-15V
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_11);//电压档位为高档 0-150V
	}
	if(V_Gear_SW==0)
	{
		if(Voltage>V_LOW_MAX)//当测量电压高于低档最大限制电压是档位自动跳转到高档
		{
			V_Gear_SW=1;
			GPIO_ResetBits(GPIOA,GPIO_Pin_11);//电压档位为高档
		}
	}
}
/***************************************
函数名:worke_mode
函数输入:
函数功能:工作模式切换
****************************************/
void worke_mode(void)
{
/**********模式切换**********************/
	if(MODE==0)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CC模式
	}
	else if(MODE==1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_0);//CV模式
	}
	else if(MODE==2)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CR模式
		if(SET_Resist==0)
		{
			SET_Resist=1;
		}
	}
	else if(MODE==3)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CP模式
	}
	else if(MODE==4)//动态模式
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CC模式拉载
	}
	else if(MODE==5)//LED模式 LED模式电压档位默认切换到高档位
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CC模式
		V_Gear_SW=1;
		GPIO_SetBits(GPIOA,GPIO_Pin_11);//电压档位为高档
	}
	else if(MODE==6)//短路模式
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//CC模式拉载
		if(I_Gear_SW==0)
		{
			SET_S_Current=199000;
		}
		else
		{
			SET_S_Current=180000;//短路模式下直接加载最大电流值
		}
	}
/*************ON/OFF开关***************************/
	if(onoff_ch==0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
		TIME_1MS_OVER=0;//打开爬升标志
		TIME_1MS_flag=0;//清零时间标志
		SET_I_TRAN=0;
		Von_CONT=0;//清除开启门槛计数器
		SET_V_TRAN=Voltage;//对设置值清零用于每次打开负载后爬升率作用
	}
	else if(onoff_ch==1)
	{
		if((MODE==0)||(MODE==1)||(MODE==2)||(MODE==3))
		{
			if(V_Gear_SW==1)//电压高档位
			{
				if(Voltage<VOFF_Voltage)//判断测量电压是否小于卸载电压
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//对设置值清零用于每次打开负载后爬升率作用
					TIME_1MS_OVER=0;//打开爬升标志
					TIME_1MS_flag=0;//清零时间标志
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
				}
				if((Voltage>VON_Voltage)&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
					Von_CONT=1;
				}
				else if((Voltage<VON_Voltage)&&(Von_CONT==0))
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//对设置值清零用于每次打开负载后爬升率作用
					TIME_1MS_OVER=0;//打开爬升标志
					TIME_1MS_flag=0;//清零时间标志
					Von_CONT=0;//清除开启门槛计数器
				}
			}
			else if(V_Gear_SW==0)//电压低档位VON VOFF都*10，因为设定参数只支持高档位设定
			{
				if(Voltage<(VOFF_Voltage*10))//判断测量电压是否小于卸载电压
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//对设置值清零用于每次打开负载后爬升率作用
					TIME_1MS_OVER=0;//打开爬升标志
					TIME_1MS_flag=0;//清零时间标志
				}
				else 
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
				}
				if((Voltage>(VON_Voltage*10))&&(Von_CONT==0))
				{
					GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
					Von_CONT=1;
				}
				else if((Voltage<(VON_Voltage*10))&&(Von_CONT==0))
				{
					GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
					SET_I_TRAN=0;
					SET_V_TRAN=Voltage;//对设置值清零用于每次打开负载后爬升率作用
					TIME_1MS_OVER=0;//打开爬升标志
					TIME_1MS_flag=0;//清零时间标志
					Von_CONT=0;//清除开启门槛计数器
				}
			}
		}
		else if(MODE==5)//LED模式
		{
			static vu32 LED_ON_V;
			LED_ON_V=0;
			LED_ON_V=(LED_VO*100)*LED_RD;
			LED_ON_V=LED_VO-(LED_ON_V/10000);//通过RD系数计算出LED加载门槛电压
			if(Voltage>=LED_ON_V)//判断电压是否大于LED导通电压
			{
				Cont_coeff_LEDMODE(LED_ON_V);
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
			}
			else
			{
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭负载
				SET_I_TRAN=0;
				TIME_1MS_OVER=0;//打开爬升标志
				TIME_1MS_flag=0;//清零时间标志
			}
		}
		else if(MODE==6)//短路模式
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);//打开负载
		}
	}
	I_SW_COTNR();//电流档位切换
	V_SW_COTNR();//电压档位切换
  Sence_SW_CONT();//SENC切换
}
/***************************************
函数名:Cont_coeff_LEDMODE
函数输入:
函数功能:LED模式线性关系系数计算
****************************************/
void Cont_coeff_LEDMODE(vu32 on_v)
{
	vu32 var16;
	vu32 var32;
	vu32 var32b;
	vu32 cont_A_READ,cont_B_READ;
	vu32 cont_A_ACT,cont_B_ACT;
	vu32 REG_LED_CorrectinonA;
	vu32 REG_LED_Offset;
	vu8 Polar_led;
/***********计算线性系数***************/
	cont_A_READ = on_v;
	cont_A_ACT = 0;

	cont_B_READ = LED_VO;
	cont_B_ACT = LED_IO;
	
	var32 = cont_B_ACT;
	var32 = var32 - cont_A_ACT;
	var32 = var32 << 12;
	var16 = cont_B_READ - cont_A_READ;
	var32 = var32 / var16;
	REG_LED_CorrectinonA = var32;
	var32 = cont_B_ACT;
	var32 = var32 << 12;
	var32b = cont_B_READ;
	var32b = var32b * REG_LED_CorrectinonA;
	if (var32 < var32b)
	{
		var32b = var32b - var32;
		REG_LED_Offset = var32b;
		Polar_led |= 0x01;
	}
	else 
	{
		var32 = var32 - var32b;
		REG_LED_Offset = var32;
		Polar_led &= ~0x01;						
	}
/*******************线性电流换算***************************/
	var32=0;
	var32 = Voltage;
	var32 = var32 * REG_LED_CorrectinonA;  //Y=KX+B 这里计算Y=KX
	if ((Polar_led & 0x01) == 0x01)		  //
	{
		if (var32 < REG_LED_Offset) 
		{
			var32 = 0;
		}
		else var32 = var32 - REG_LED_Offset;
	}
	else var32 = var32 + REG_LED_Offset;
	var32 = var32 >> 12;
	SET_R_Current = var32;
	var32 = 0;
}
/***************************************
函数名:MAXPAR_limit
函数输入:
函数功能:各设定参数最大限制
****************************************/
void MAXPAR_limit(void)
{
	if(SET_Voltage>MAX_V)
	{
		SET_Voltage=MAX_V;
	}
	if(SET_Current>MAX_I)
	{
		SET_Current=MAX_I;
	}
	if(VON_Voltage>MAX_V)
	{
		VON_Voltage=MAX_V;
	}
	if(VOFF_Voltage>MAX_V)
	{
		VOFF_Voltage=MAX_V;
	}
	if(I_Rise_Time>20000)
	{
		I_Rise_Time=20000;
	}
	if(CV_Down_Time>3000)
	{
		CV_Down_Time=3000;
	}
	if(I_Down_Time>20000)
	{
		I_Down_Time=20000;
	}
}