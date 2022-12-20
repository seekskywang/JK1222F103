/******************** (C) COPYRIGHT 2014 KUNKIN********************
 * 文件名  ：my_register
 * 作者    ：李振
 * 描述    ：库函数版
 * 内容    ：Define register定义系统使用的寄存器，全局变量
 * 注意		 ：头文件中只能有声明，不能有定义
 * 修改日期：2014-12-9
*********************************************************************/

#include "STM32F10x.h"
#include "me_scpi.h"

#ifndef __my_register_h__
#define __my_register_h__
//=============================================================================

//=============================================================================
#define NOP   __nop();	  //空指令

//============================================================================
extern struct bitDefine flagA,flagB,flagC,flagD,flagE,flagF,flagG;
#define   Flag_ADC_Full             (flagA.bit0)//DMA传输完成标志位 
#define   Flag_BEEP						    	(flagA.bit1)//蜂鸣器触发标志
#define   Flag_Key_Scan							(flagA.bit2)//按键定时扫描标志
#define   UART_SEND_flag						(flagA.bit3)//串口发送完成标志
#define   UART_Buffer_Rece_flag			(flagA.bit4)//串口接收完成标志
#define   Flag_Swtich_ON      			(flagA.bit5)//启动按键处于ON标志
#define   Flag_DAC_OFF              (flagA.bit6)//是否加载DAC标志
#define   Flag_Beep_OFF             (flagA.bit7)//蜂鸣器关闭标志

#define   Flag_I_LOW                (flagB.bit0)//电流档位切换标志
#define   flag_CC_MODE							(flagB.bit1)//工作模式标志
#define   flag_V_level							(flagB.bit2)//电压档位切换标志
#define   flag_NOR_CODE							(flagB.bit3)//SCPI接收一帧正确数据
#define   flag_Tim_USART            (flagB.bit4)//串口定时清理
#define   TIME_1MS_flag						  (flagB.bit5)//1ms时间标志位
#define   TIME_1MS_OVER				  	  (flagB.bit6)//开启爬升或者下降计时标志
#define   Flag_AD7655_DMA				    (flagB.bit7)//AD7655_DMA传输完成标志

#define   Flag_Save_SW				      (flagC.bit0)//将设置值存入EEPROM标志位

//=============================================================================


//--------------------------
extern vu8 DAC_Flag;
extern vu8 Beep_Flag;
extern vu8 t_beep;
extern vu8 t_KEYON;
extern vu16 t_0_5S;
extern vu16 t_1S;
extern vu16 t_5S;
extern vu16 t_USART;
extern vu8 Setcontr_ADDR;//设置参数存入FLASH位置
extern vu8 UART1_Buffer_Rece_flag;
extern vu8 UART3_Buffer_Rece_flag;
//--------------------------
extern vu8 UART_Buffer_Size;//串口接收数据长度
extern vu8 Transmit_BUFFERsize;
//---------------------------
extern vu16 NTC_value;   //AD值
extern vu16 NTC1_value;   //AD值
extern vu16 Imon_value;
extern vu16 Vmon_value;
extern vu16 Contr_DACVlue;//DAC控制值

extern vu32 Transition_Date[10];//过渡变量
#define SET_V_TRAN                  Transition_Date[0]   //设置电压过渡变量
#define SET_I_TRAN                  Transition_Date[1]   //设置电流过渡变量
#define SET_R_Current               Transition_Date[2]   //CR模式换算设置电流值
#define SET_P_Current				Transition_Date[3]   //CP模式换算设置电流值
#define SET_S_Current				Transition_Date[4]   //短路模式加载的设置电流值
#define SET_R_LED					Transition_Date[5]   //LED模式换算设置加载电阻


extern vu32 Run_Control[55];	
#define Voltage                     Run_Control[0]   //电压测量值
#define Current                     Run_Control[1]   //电流测量值
#define R_DATE                      Run_Control[2]   //电阻测量值
#define Power_DATE                  Run_Control[3]   //功率测量值

#define	MES_VOLT_MAX				Run_Control[4]   //测量电压峰值VP+
#define	MES_VOLT_MIN				Run_Control[5]  //测量电压峰值VP-
#define	MES_VOLT_PTP				Run_Control[6]  //测量电压峰峰值

#define	MES_CURR_MAX				Run_Control[7]   //测量电流峰值VP+
#define	MES_CURR_MIN				Run_Control[8]   //测量电流峰值VP-
#define	MES_CURR_PTP				Run_Control[9]   //测量电流峰峰值   
/*****************以上寄存器为只读寄存器严禁程序操作**********************/

#define Operation_MODE 				Run_Control[10]   //操作模式开关 0-面板操作  1-远程操作(通讯)
#define protect_Flage 				Run_Control[11]   //各种保护标志 1.过功率保护  2.输入反接保护标志  3.输入过压保护标志  4.负载过流保护标志 

#define onoff_ch                    Run_Control[12]   //ON/OFF 0-关闭所有  1-打开负载  
#define MODE                        Run_Control[13]   //模式开关  0-CC  1-CV  2-CR 3-CP 4-动态模式 5-LED 6-短路模式
#define I_Gear_SW					Run_Control[14]   //电流档位切换开关  0-低档位  1-高档位
#define V_Gear_SW					Run_Control[15]   //电压档位切换开关  0-低档位  1-高档位
#define ADDR						Run_Control[16]   //本机地址
#define Baud_rate					Run_Control[17]   //波特率
#define Sence_SW 					Run_Control[18]   //远端测量开关

#define SET_Voltage                 Run_Control[19]   //设置电压值
#define SET_Current                 Run_Control[20]   //设置电流值
#define SET_Resist                  Run_Control[21]   //设置电阻值
#define SET_Power                   Run_Control[22]   //设置功率值
#define OVP_Voltage 				Run_Control[23]   //OVP
#define OCP_Current					Run_Control[24]   //OCP
#define OPP_POWER					Run_Control[25]   //OPP
#define MAX_P						Run_Control[26]   //最大限制功率
#define MAX_V						Run_Control[27]   //最大限制电压
#define MAX_I						Run_Control[28]   //最大限制电流
#define VON_Voltage					Run_Control[29]   //加载电压
#define VOFF_Voltage				Run_Control[30]   //卸载电压
#define I_Rise_Time 				Run_Control[31]   //电流爬升率
#define I_Down_Time 				Run_Control[32]   //电流下降率
#define CV_Down_Time 				Run_Control[33]   //CV模式电压下降时间

#define LED_VO 						Run_Control[34]   //LED模式顺向工作电压
#define LED_IO 						Run_Control[35]   //LED模式顺向工作电流
#define LED_RD 						Run_Control[36]   //LED模式RD系数，此参数是用来计算加载电压阈值范围为0-100%，例如：设置VO为30V RD-0.2  此时加载电压应为30*0.2=6，VI=30-6=24V。

#define DYNA_Ia 					Run_Control[37]   //动态模式拉载电流A
#define DYNA_Ib 					Run_Control[38]   //动态模式拉载电流B
#define DYNA_Ta   					Run_Control[39]   //动态模式拉载持续时间A
#define DYNA_Tb						Run_Control[40]   //动态模式拉载持续时间B
#define DYNA_IRise					Run_Control[41]   //动态模式电流上升率
#define DYNA_IDown					Run_Control[42]   //动态模式电流下降率
#define DYNA_MODE					Run_Control[43]   //动态模式触发模式选择

#define COMM_MODE					Run_Control[44]   //通讯方式选择
#define Class_5						Run_Control[45]   //5V
#define Class_9						Run_Control[46]   //9V
#define Class_12					Run_Control[47]   //12V
#define Class_15					Run_Control[48]   //15V
#define Class_20					Run_Control[49]   //20V
#define BOOTLOAD					Run_Control[50]   //跳转进入BOOT程序标志
#define BOOTMODE					Run_Control[52]   //开机启动模式：0-直接进入APP，1-进入BOOTLOADER
/***********以上寄存器可读可写*****************************************/
typedef struct {
	float TestVL;//测量电压低档k
	float TestVM;//测量电压中档k
	float TestVH;//测量电压高档k
	
	float SetVL;//设置电压低档k
	float SetVM;//设置电压中档k
	float SetVH;//设置电压高档k
	
	
	float OffsetVL;//测量电压低档b
	float OffsetVM;//测量电压中档b
	float OffsetVH;//测量电压高档b
	
	float OffsetSetVL;//设置电流低档b
	float OffsetSetVM;//设置电流中档b
	float OffsetSetVH;//设置电流高档b
	
		
/////////////////////////////////////		
	float TestCL;//测量电流低档k
	float TestCM;//测量电流中档k
	float TestCH;//测量电流高档k
	
	float SetCL;//设置电压低档k
	float SetCM;//设置电压中档k
	float SetCH;//设置电压高档k
	
	float OffsetCL;//测量电流低档b
	float OffsetCM;//测量电流中档b
	float OffsetCH;//测量电流高档b
	
	float OffsetSetCL;//设置电流低档b
	float OffsetSetCM;//设置电流中档b
	float OffsetSetCH;//设置电流高档b
	
/////////////////////////////////////	

}CAL;

typedef union
{
    vu8 CalFlash[96];
    CAL CalPara;
}CalUni; 

typedef  void (*iapfun)(void);				//定义一个函数类型的参数.
#define FLASH_BOOT_ADDR		0x08000000  	//BOOTLOAD地址(存放在FLASH)
extern CAL CalPara;
extern u8 bootflag;
extern u8 ledmode;
extern vu32 Correct_Parametet[26];//校准系数
#define  REG_CorrectionV_LOW       Correct_Parametet[0]
#define  REG_CorrectionV_HIG       Correct_Parametet[1]
#define  REG_CorrectionV_MID_HIG   Correct_Parametet[2]
#define  REG_CorrectionV_MID_HIG1   Correct_Parametet[20]
#define  REG_CorrectionV_MID_HIG2   Correct_Parametet[21]
#define  REG_CorrectionV_MID_HIG3   Correct_Parametet[22]

#define  REG_CorrectionA_LOW       Correct_Parametet[3]
#define  REG_CorrectionA_HIG       Correct_Parametet[4]
#define  REG_CorrectionA_MID_HIG   Correct_Parametet[5]


#define  SET_CorrectionV_LOW       Correct_Parametet[6]
#define  SET_CorrectionV_HIG       Correct_Parametet[7]
#define  SET_CorrectionV_MID_HIG   Correct_Parametet[8]
#define  SET_CorrectionV_MID_HIG1   Correct_Parametet[23]
#define  SET_CorrectionV_MID_HIG2   Correct_Parametet[24]
#define  SET_CorrectionV_MID_HIG3   Correct_Parametet[25]

#define  SET_CorrectionA_LOW       Correct_Parametet[9]
#define  SET_CorrectionA_HIG       Correct_Parametet[10]
#define  SET_CorrectionA_MID_HIG   Correct_Parametet[11]

#define  REG_CorrectionA_LOW2       Correct_Parametet[12]
#define  REG_CorrectionA_LOW3       Correct_Parametet[13]
#define  REG_CorrectionA_HIG1       Correct_Parametet[14]
#define  REG_CorrectionA_HIG2   	Correct_Parametet[15]

#define  SET_CorrectionA_LOW2       Correct_Parametet[16]
#define  SET_CorrectionA_LOW3       Correct_Parametet[17]
#define  SET_CorrectionA_HIG1       Correct_Parametet[18]
#define  SET_CorrectionA_HIG2   	Correct_Parametet[19]

extern vu8 coefficient[13];//校准系数
#define  Polar0            			 coefficient[0]
#define  Polar1           			 coefficient[1]
#define  Polar2						 coefficient[2]
#define  Polar3						 coefficient[3]
#define  Polar4            			 coefficient[4]
#define  Polar5            			 coefficient[5]
#define  Polar6						 coefficient[6]
#define  Polar7						 coefficient[7]
#define  Polar8						 coefficient[8]
#define  Polar9						 coefficient[9]
#define  Polar10					 coefficient[10]
#define  Polar11					 coefficient[11]
#define  Polar12					 coefficient[12]

extern vu32 Correct_Strong[26];//校准系数
#define  REG_ReadV_Offset_LOW       Correct_Strong[0]
#define  REG_ReadV_Offset_HIG       Correct_Strong[1]
#define  REG_ReadV_Offset_MID_HIG   Correct_Strong[2]
#define  REG_ReadV_Offset_MID_HIG1  Correct_Strong[20]
#define  REG_ReadV_Offset_MID_HIG2  Correct_Strong[21]
#define  REG_ReadV_Offset_MID_HIG3  Correct_Strong[22]

#define  REG_ReadA_Offset_LOW       Correct_Strong[3]
#define  REG_ReadA_Offset_HIG       Correct_Strong[4]
#define  REG_ReadA_Offset_MID_HIG   Correct_Strong[5]

#define  SET_ReadV_Offset_LOW       Correct_Strong[6]
#define  SET_ReadV_Offset_HIG       Correct_Strong[7]
#define  SET_ReadV_Offset_MID_HIG   Correct_Strong[8]
#define  SET_ReadV_Offset_MID_HIG1  Correct_Strong[23]
#define  SET_ReadV_Offset_MID_HIG2  Correct_Strong[24]
#define  SET_ReadV_Offset_MID_HIG3  Correct_Strong[25]

#define  SET_ReadA_Offset_LOW       Correct_Strong[9]
#define  SET_ReadA_Offset_HIG       Correct_Strong[10]
#define  SET_ReadA_Offset_MID_HIG   Correct_Strong[11]

#define  REG_ReadA_Offset_LOW2       Correct_Strong[12]
#define  REG_ReadA_Offset_LOW3       Correct_Strong[13]
#define  REG_ReadA_Offset_HIG1       Correct_Strong[14]
#define  REG_ReadA_Offset_HIG2   	Correct_Strong[15]

#define  SET_ReadA_Offset_LOW2       Correct_Strong[16]
#define  SET_ReadA_Offset_LOW3       Correct_Strong[17]
#define  SET_ReadA_Offset_HIG1       Correct_Strong[18]
#define  SET_ReadA_Offset_HIG2   	Correct_Strong[19]
//---------------------------
extern vu8 UART_Buffer_Rece[200];
extern vu8 UART_Buffer_Send[200];
extern vu8 UART_Buffer_Rece1[200];
extern vu8 UART_Buffer_Send1[200];
extern vu8 dynaflagA;
extern vu8 dynaflagB;
extern vu8 dynatrigflag;
extern u8 dynaonflag;
extern vu16 SWDelay;

////============================================================================= 

//#define ILOW1   60000   //电流低档位跳档值1
//#define ILOW2   100000   //电流低档位跳档值2
//#define ILOW3   240000   //电流低档位跳档值3

//#define IHIGH1   240000   //电流低档位跳档值1
//#define IHIGH2   600000   //电流低档位跳档值2
//#define IHIGH3   1000000   //电流低档位跳档值3
//#define IHIGH4   2000000   //电流低档位跳档值4

//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
////#define V_LOW_MAX   120000   //电压低档最高电压
////#define V_HIG_MAX   120000   //电压高档最高电压
//#define V_LOW_MAX   120000   //电压低档最高电压
//#define V_HIG_MAX   1200000   //电压高档最高电压
//#define I_LOW_MAX   240000   //电流低档位最高限制电流
//#define POWER_MAX   2400000   //电流低档位最高限制功率
////#define POWER_MAX   800000   //电流低档位最高限制功率
////#define POWER_MAX   600000   //电流低档位最高限制功率
////=============================================================================
//============================================================================= 


//3200W
#define ILOW1   60000   //电流低档位跳档值1
#define ILOW2   100000   //电流低档位跳档值2
#define ILOW3   240000   //电流低档位跳档值3

#define IHIGH1   240000   //电流低档位跳档值1
#define IHIGH2   600000   //电流低档位跳档值2
#define IHIGH3   1000000   //电流低档位跳档值3
#define IHIGH4   2000000   //电流低档位跳档值4
#define SWDELAY   1000 
//============================================================================= 
#define Receive_BUFFERSIZE   10
#define V_LOW_MAX   150000   //电压低档最高电压
#define V_HIG_MAX   150000   //电压高档最高电压
#define I_LOW_MAX   240000   //电流低档位最高限制电流
#define POWER_MAX   3200000   //电流低档位最高限制功率

////2400W
//#define ILOW1   60000   //电流低档位跳档值1
//#define ILOW2   100000   //电流低档位跳档值2
//#define ILOW3   240000   //电流低档位跳档值3

//#define IHIGH1   240000   //电流低档位跳档值1
//#define IHIGH2   600000   //电流低档位跳档值2
//#define IHIGH3   1000000   //电流低档位跳档值3
//#define IHIGH4   2000000   //电流低档位跳档值4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   150000   //电压低档最高电压
//#define V_HIG_MAX   150000   //电压高档最高电压
//#define I_LOW_MAX   240000   //电流低档位最高限制电流
//#define POWER_MAX   2400000   //电流低档位最高限制功率

////400W
//#define ILOW1   1000   //电流低档位跳档值1
//#define ILOW2   20000   //电流低档位跳档值2
//#define ILOW3   50000   //电流低档位跳档值3

//#define IHIGH1   100000   //电流低档位跳档值1
//#define IHIGH2   150000   //电流低档位跳档值2
//#define IHIGH3   200000   //电流低档位跳档值3
//#define IHIGH4   300000   //电流低档位跳档值4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   100
//#define V_LOW_MAX   150000   //电压低档最高电压
//#define V_HIG_MAX   150000   //电压高档最高电压
//#define I_LOW_MAX   40000   //电流低档位最高限制电流
//#define POWER_MAX   400000   //电流低档位最高限制功率

////600W
//#define ILOW1   1000   //电流低档位跳档值1
//#define ILOW2   30000   //电流低档位跳档值2
//#define ILOW3   70000   //电流低档位跳档值3

//#define IHIGH1   100000   //电流低档位跳档值1
//#define IHIGH2   200000   //电流低档位跳档值2
//#define IHIGH3   300000   //电流低档位跳档值3
//#define IHIGH4   400000   //电流低档位跳档值4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   150000   //电压低档最高电压
//#define V_HIG_MAX   150000   //电压高档最高电压
//#define I_LOW_MAX   60000   //电流低档位最高限制电流
//#define POWER_MAX   600000   //电流低档位最高限制功率


////800W
//#define ILOW1   20000   //电流低档位跳档值1
//#define ILOW2   40000   //电流低档位跳档值2
//#define ILOW3   90000   //电流低档位跳档值3

//#define IHIGH1   200000   //电流低档位跳档值1
//#define IHIGH2   400000   //电流低档位跳档值2
//#define IHIGH3   500000   //电流低档位跳档值3
//#define IHIGH4   600000   //电流低档位跳档值4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   150000   //电压低档最高电压
//#define V_HIG_MAX   150000   //电压高档最高电压
//#define I_LOW_MAX   80000   //电流低档位最高限制电流
//#define POWER_MAX   800000   //电流低档位最高限制功率

////1200W
//#define ILOW1   30000   //电流低档位跳档值1
//#define ILOW2   60000   //电流低档位跳档值2
//#define ILOW3   130000   //电流低档位跳档值3

//#define IHIGH1   200000   //电流低档位跳档值1
//#define IHIGH2   400000   //电流低档位跳档值2
//#define IHIGH3   600000   //电流低档位跳档值3
//#define IHIGH4   800000   //电流低档位跳档值4
//#define SWDELAY   1000 
////============================================================================= 
//#define Receive_BUFFERSIZE   10
//#define V_LOW_MAX   150000   //电压低档最高电压
//#define V_HIG_MAX   150000   //电压高档最高电压
//#define I_LOW_MAX   120000   //电流低档位最高限制电流 
//#define POWER_MAX   1200000   //电流低档位最高限制功率


#endif
/******************* (C) COPYRIGHT 2015 KUNKIN *****END OF FILE*************************/
