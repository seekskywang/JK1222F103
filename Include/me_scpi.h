
#ifndef  __ME_SCPI_H
#define  __ME_SCPI_H

#include "stm32f10x.h"
#include "me_scpi.h"
#include "my_register.h" 

//转换数组的值
#define    U8(Halfword) 	(*((   vu8*)(&Halfword)))
#define   U16(Halfword) 	(*((  vu16*)(&Halfword)))
#define   U32(Halfword) 	(*((  vu32*)(&Halfword)))
#define   FLOAT(Halfword) 	(*((float*)(&Halfword)))

/*
-------
此头文件定义了SCPI要用到的所有寄存器地址，使用地址时不要重复使用

Run_Control[20]--
--------
*/
extern vu16 Run_temp_contr[10];
#define   StandEventREG    Run_temp_contr[0] //标准事件寄存器
#define   QuestEventREG    Run_temp_contr[1] //查询事件
#define   OpertEventREG    Run_temp_contr[2] //操作事件
#define   BitCropeREG      Run_temp_contr[3] //位组寄存器
#define   ErrorCodeREG     Run_temp_contr[4] //错误代码

#define   BeeperREG  			 Run_temp_contr[6] //蜂鸣器寄存器 1：on  0：off

#define   LoadONOffREG  	 Run_Control[12] //负载ON/OFF
#define   ShortENREG  	   Run_temp_contr[5] //输入短路状态 ON/OFF


#define   LOAD_StartloadV  U32(Run_Control[29]) //负载开始带载电压值 最大允许带载的电压
#define   LOAD_StartfreeV  U32(Run_Control[30]) //负载开始卸载电压值 

#define   SET_HIG_DYNCUR   		  U32(Run_Control[64]) //设置动态模式高准位拉载电流  Run_Control[65]
#define   SET_HIG_DYNCURTIM     U32(Run_Control[66])		 //设置动态模式高准位拉载电流持续时间 us Run_Control[67]
#define   SET_LOW_DYNCUR   		  U32(Run_Control[68]) //设置动态模式低准位拉载电流 
#define   SET_LOW_DYNCURTIM     U32(Run_Control[70]) 		 //设置动态模式低准位拉载电流持续时间 
#define   SET_DYN_UPKate        Run_Control[72] 		 //设置动态模式上升率
#define   SET_DYN_DOWNKate      Run_Control[73] 		 //设置动态模式下降率
#define   SET_DYN_WorkMode      Run_Control[74] 		 //设置动态模式下的工作模式

//--------负载工作模式---------------------------------------------------------------
#define   CURR_MODE				 0 //CC模式
#define   VOLT_MODE				 1 //CV模式
#define   RES_MODE				 2 //CR模式
#define   POW_MODE				 3 //CP模式
#define   LED_MODE				 4 //LED模式
#define   Baty_MODE				 5 //电池模式
#define   DYN_MODE				 6 //动态操作模式


#define   DYN_PULS				 0 //动态模式下的工作模式
#define   DYN_CONT				 1 //动态模式下的工作模式
#define   DYN_TOGG				 2 //动态模式下的工作模式

#define   HIGH_Level			 1 //高档
#define   LOW_Level			   0 //低档

/*
-------
定义最大电压电流
MAX CURRENT
MAX VOLOT
--------
*/

#define		MAX_SET_CURRENT       200000    //20.0000A  最大设置电流 (低档 2.00000A  高档 20.0000)
#define		MAX_SET_VOLT          150000    //150.000V  最大设置电压 (低档 15.0000A  高档 150.000)
#define		MAX_SET_POW           120000    //120.000W  最大功率
#define		MAX_SET_RES           50000.0   //50000.0R  最大电阻


#define		MAX_CURRENT    				20000			//最大电流 ( 2A )不同档位电流小数点不同 ( 0- 2.0000, 0-20.000)
#define		MIN_CURRENT    				1000			//最小电流  用于判断档位区间


#define		MAX_CURRENTUP    			30000			//最大电流上升率
#define		MIN_CURRENTUP   			6			    //最小电流上升率
#define		MAX_CURRENTDOWN    		30000			//最大电流下降率
#define		MIN_CURRENTDOWN  			6			    //最小电流下降率
#define		MAX_CURRENTPOOTE  		210000		//最大电流保护值 （21.0000）

#define		MAX_POWERPOOTE  		 	15000			//最大功率保护值 （150.00）

#define		MAX_VOLOTE    				150000		//最大电压  (0-15.0000， 0-150.000)
#define		MIN_VOLOTE    				100			  //最小电压

#define		MAX_LoadVOLOTE    		150000		//最大开始带载电压值 150.000
#define		MIN_LoadVOLOTE    		0			    //最小开始带载电压值

#define		MAX_FreeLoadVOLOTE    120000		//最大开始卸载电压值
#define		MIN_FreeLoadVOLOTE    0			    //最小开始卸载电压值


#define		DYN_HIGH_MAX          210000    //动态模式高准位拉载电流 21.0000  MAX
#define		DYN_HIGH_MIN          0         //动态模式高准位拉载电流 0        MIN
#define		DYN_LOW_MAX           210000    //动态模式低准位拉载电流 21.0000  MAX
#define		DYN_LOW_MIN           0         //动态模式低准位拉载电流 0        MIN


#define		DYN_HIGH_DELLMAX      50000000    //动态模式高准位拉载电流时间 50.00000S  MAX  >10s最后一位无效
#define		DYN_HIGH_DELLMIN      10         //动态模式高准位拉载电流时间 0.000010S  MIN

#define		DYN_LOW_DELLMAX      50000000    //动态模式低准位拉载电流时间 50.00000S  MAX >10s最后一位无效
#define		DYN_LOW_DELLMIN      10         //动态模式低准位拉载电流时间 0.000010S  MIN

#define		DYN_SLEW_RISEMAX     30000      //动态模式电流上升率 MAX  3.0000
#define		DYN_SLEW_RISEMIN     6          //动态模式电流上升率 MAX  0.0006

#define		DYN_SLEW_FALLMAX     30000      //动态模式电流下降率 MAX  3.0000
#define		DYN_SLEW_FALLMIN     6          //动态模式电流下降率 MAX  0.0006




void Me_SCPI_TASK(void);

#endif


/*----------------------------------COPY RIGHT BY XQ_GUO 2019---------------------------------------------*/

