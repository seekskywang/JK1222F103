/******************************************************************/
/* 名称TIM3 PWM                                                */
/* 效果：                                                        */
/* 内容：产生一个200HZ 正占空比：60.9% 负占空比：30.9%的PWM      */
/* 作者：李振                                                    */
/* 联系方式：QQ:363116119                                        */
/******************************************************************/
#include "my_register.h"
#include "beep.h"
#include "sys_io_cfg.h"

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

vu16 PWM_VALU;
vu16 VP_T,IP_T;
vu16 PR_T1,PR_T2,PR_T3,PR_T4;
vu32 powprtflag=0;
vu32 powprtcount=0;
/**************************************************************************************/
//void Temp_Comapre(void)	  //温度来控制风扇
//{
//  if((NTC_value<800)||(NTC1_value<500)) //过温度保护 断负载
//	{
//		onoff_ch=0;
//		GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
//		protect_Flage=5;//过温度保护
//	}		
//	if((NTC_value<=2250)||(NTC1_value<=2250))//开风扇
//	{
//		IO_FAN_ON;
//	}
//	else if((NTC_value>=2300)&&(NTC1_value>=2300))//关风扇
//	{
//		IO_FAN_OFF;
//	}
//}

void Temp_Comapre(void)	  //温度来控制风扇
{
  if((NTC_value<600)/*||(NTC1_value<2000)*/) //过温度保护 断负载
	{
		onoff_ch=0;
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
		protect_Flage=5;//过温度保护
	}		
	if((NTC_value<=1400)/*||(NTC1_value<=2250)*/)//开风扇
	{
		IO_FAN_ON;
	}
	else if((NTC_value>=1550)/*&&(NTC1_value>=500)*/)//关风扇
	{
		IO_FAN_OFF;
	}
}
/********************************************/
void All_protect(void)
{
	 vu32 temp_power;
//	 double maxbuf;
  /*************过功率保护***************/
//	maxbuf = (double)(POWER_MAX)*0.9;
	if(Power_DATE > POWER_MAX*0.9)
	{
//		PR_T1=0;
		powprtflag = 1;
		if(Power_DATE > POWER_MAX)
		{
			PR_T1=0;
			onoff_ch=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
			protect_Flage=1;//过功率保护
		}else{
			if(powprtcount > 9000000)
			{
				PR_T1=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//过功率保护
			}
		}
	}else{
		powprtflag = 0;
		powprtcount = 0;
	}
	
	if((I_Gear_SW==0)&&(V_Gear_SW==1))
	{
		temp_power=Current/10;
		temp_power=temp_power*Voltage;
		temp_power=temp_power/100000;
		if(temp_power>MAX_P)//过功率保护
		{
			PR_T1++;
			if(PR_T1>2000)
			{
				PR_T1=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//过功率保护
			}
		}
	}
	else if((I_Gear_SW==1)&&(V_Gear_SW==0))
	{
		temp_power=Current;
		temp_power=temp_power*(Voltage/10);
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//过功率保护
		{
			PR_T2++;
			if(PR_T2>2000)
			{
				PR_T2=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//过功率保护
			}
		}
	}
	else if((I_Gear_SW==0)&&(V_Gear_SW==0))
	{
		temp_power=Current/10;
		temp_power=temp_power*(Voltage/10);
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//过功率保护
		{
			PR_T3++;
			if(PR_T3>2000)
			{
				PR_T3=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_15);//OFF
				protect_Flage=1;//过功率保护
			}
		}
	}
	else if((I_Gear_SW==1)&&(V_Gear_SW==1))
	{
		temp_power=Current;
		temp_power=temp_power*Voltage;
		temp_power=temp_power/1000;
		if(temp_power>MAX_P)//过功率保护
		{
			PR_T4++;
			if(PR_T4>2000)
			{
				PR_T4=0;
				onoff_ch=0;
				GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
				protect_Flage=1;//过功率保护
			}
		}
	}
	
 /**************反接保护************************/
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)==0)//反接保护
	{
		onoff_ch=0;
		GPIO_SetBits(GPIOA,GPIO_Pin_5);//关闭拉载
		protect_Flage=2;//反接保护
		GPIO_SetBits(GPIOB,GPIO_Pin_8);//打开蜂鸣器
	}
	else 
	{
		protect_Flage=0;//反接保护
		GPIO_ResetBits(GPIOB,GPIO_Pin_8);//关闭蜂鸣器
	}
	/************过压保护************************/
	if((Voltage>MAX_V)&&(V_Gear_SW==1))//过压保护
	{
		VP_T++;
		if(VP_T>5)
		{
			VP_T=0;
			onoff_ch=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
			protect_Flage=3;//过压保护
		}
	}
	/***************过流保护****************/
	if((Current>MAX_I)&&(I_Gear_SW==1))//过流保护
	{
		IP_T++;
		if(IP_T>5)
		{
			IP_T=0;
			onoff_ch=0;
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//OFF
			protect_Flage=4;//过流保护
		}
	}
}