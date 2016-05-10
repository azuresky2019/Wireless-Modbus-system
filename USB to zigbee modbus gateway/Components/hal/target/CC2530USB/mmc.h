/*#######################################################################################
Connect ARM to MMC/SD 

Copyright (C) 2004 Ulrich Radig

ARM modifications by Ingo Busker 2005
#######################################################################################*/

#ifndef _MMC_H_
 #define _MMC_H_

  
#include "Sd.h"

/*********************************************************************************
 * CONTANTS
 *********************************************************************************/
#define  SD_RFID_EVENT    0
#define  SD_TIME_EVENT    1
#define  SD_TIME_RESULT   2
#define  SD_JUMP_EVENT    3
#define SD_JUMP_SCORE     4


#ifndef MMC_WRITE_ENABLE
  #define MMC_WRITE_ENABLE  FALSE
#endif
/*********************************************************************************
 * MACROS
 *********************************************************************************/
//set MMC_Chip_Select to high (MMC/SD-Karte Inaktiv)
#define MMC_Disable()   MMC_Chip_Select=1;          //MMC_Write|= (1<<MMC_Chip_Select);

//set MMC_Chip_Select to low (MMC/SD-Karte Aktiv)
#define MMC_Enable()   MMC_Chip_Select=0;          //MMC_Write&=~(1<<MMC_Chip_Select);

//#define nop()  __asm__ __volatile__ ("nop" ::)

/*********************************************************************************
 * GLOBAL VARIABLES
 ********************************************************************************
#if MMC_WRITE_ENABLE
  extern uint8    SectorBuffer[520];
#else
  extern uint8 SectorBuffer[1];
#endif  */

/*********************************************************************************
 * LOCAL FUNCTIONS
 *********************************************************************************/

//Prototypes
//uint8 mmc_read_byte(void);
//void mmc_write_byte(uint8);
/*********************************************************************************
 * GLOOBAL FUNCTIONS
 *********************************************************************************/
extern void SD_test(void);

extern void mmc_read_block(uint8 *,uint8 *,uint16);

extern uint8 mmc_init(void);
extern void SD_SPIDelay(uint8 value);

extern void mmc_write_byte (uint8 Byte);
extern uint8 mmc_read_byte (void);
extern void mmc_write_event( uint8 modbusId, uint8 *data, uint8 event);

#if 0
extern uint8 mmc_read_sector (uint32,uint8 *);

extern uint8 mmc_write_sector (uint32,uint8 *);

extern uint8 mmc_write_command (uint8 *);

extern uint8 mmc_read_csd (uint8 *);

extern uint8 mmc_read_cid (uint8 *);

extern void DELAY_Us(uint16 loop);

extern void mmc_write_file(uint8* data, uint16 len);
extern void mmc_read_file( void);
#endif //0

#endif //_MMC_H_


