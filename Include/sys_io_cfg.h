
#ifndef  __SYS_IO_CFG_H
#define  __SYS_IO_CFG_H

#include "my_register.h"
#include "stm32f10x.h"

#define  IO_FAN_ON 						GPIOA->BSRR = GPIO_Pin_4		/* ·çÉÈÒý½Å ---*/
#define  IO_FAN_OFF 					GPIOA->BRR = GPIO_Pin_4

#endif




/*----------------------------------COPY RIGHT BY XQ_GUO @2019---------------------------------------------*/

