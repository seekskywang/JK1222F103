#ifndef __AT24C01_H
#define	__AT24C01_H
#include "my_register.h"
/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */

/* AT24C01/02每页有8个字节 
 * AT24C04/08A/16A每页有16个字节 
 */
	

#define EE_DEV_ADDR			0xA0		/* 24xx02的设备地址 */
#define EE_PAGE_SIZE		8			  /* 24xx02的页面大小 */
#define EE_SIZE				256			  /* 24xx02总容量 */


vu8 ee_CheckOk(void);
vu8 ReadBytes(vu8 *_pReadBuf, vu16 _usAddress, vu16 _usSize);
vu8 WriteBytes(vu8 *_pWriteBuf, vu16 _usAddress, vu16 _usSize);
void ee_Erase(void);
void EEPROM_WriteByte(vu16 Addr,vu8 Data);
vu8 EEPROM_READ_Byte(vu8 ReadAddr);

#endif /* __I2C_EE_H */
