/******************************************************************************
 * INCLUDE
 */
#include "osal.h"
#include "mmc.h"

#include "MT.h"
#include "OSAL_Clock.h"
#include "OnBoard.h"
#include "stdio.h"
#include "modbus.h"

#include "../FatFs/ff.h"
#include "../sd/sddriver.h"

FATFS *fs;
FIL sd_file;
UINT bw;

void DELAY_Us(uint16 loop);
/******************************************************************************
 * @fn      DELAY_Us
 * 
 * @brief   Delay wait 
 * 
 * @param   uint16- us time to wait
 * 
 * @return  none
 */

void DELAY_Us(uint16 loop)
{
  while(loop--)
    asm("NOP");
}

/******************************************************************************
 * @fn      mmc_init
 * 
 * @brief   Initialize mmc support
 * 
 * @param   none
 * 
 * @return  uint8 - status
 */
uint8 mmc_init ( void)
{
//  uint8 Timeout = 0;
//  uint8 a,b;
  
  P0SEL &= ~0x3c;
//  P1SEL &= ~0x0c;
  
  P0DIR |= 0x38;
//  P1DIR |= 0x08;
//  P1DIR &= ~0x04;
  P0DIR &= ~0x04;
  
  SPI_DI=1; 
  SPI_Clock=1; 
  MMC_Chip_Select=1;    

  return(0);  
//	printf("success\r\n");  
//	while(1){};//  成功
}

/******************************************************************************
 * @fn      mmc_read_byte
 * 
 * @brief   read one byte by simulate SPI port
 * 
 * @param   none
 * 
 * @return  uint8 - The byte was been read
 */

uint8 mmc_read_byte (void)
{
	uint8 Byte = 0;
	uint8 i = 0;
	SPI_DI=1;
	for (i=0; i<8; i++) 
	{
		SPI_Clock=0;    
		DELAY_Us(6);
		Byte=Byte<<1;                            //  先接收最高位。
		if (SPI_DI==1) 
		{
				Byte |= 0x01;
		}

		SPI_Clock=1;    
		DELAY_Us(6);
	}
	return (Byte);
}

/******************************************************************************
 * @fn      mmc_write_byte
 * 
 * @brief   write one byte by simulate SPI port
 * 
 * @param   uint8 - The byte was wanted to be writen
 * 
 * @return  none
 */

void mmc_write_byte (uint8 Byte)

{
	uint8 i ;
	for (i =0; i<8; i++) 
	{
		if (Byte&0x80)	     //   先写高位的。
		{
			SPI_DO=1; 
		}
		else
		{
			SPI_DO=0; 
		}	
		SPI_Clock=0;   
		DELAY_Us(6);
		Byte=Byte<<1;
		SPI_Clock=1;   
		DELAY_Us(6);
	}
	SPI_DO=1;          
}


/******************************************************************************
 * @fn      mmc_write_event
 * 
 * @brief   Write all kinds of events to SD in one format
 * 
 * @param   modbusId - the id of gate who send the event
 *          data - pointer to the buf that will be writen
 *          event - event id 
 * 
 * @return  none
 */


void mmc_write_event( uint8 modbusId, uint8 *data, uint8 event)
{
#if MMC_WRITE_ENABLE
  char *pBuf;  
  uint8 temp1[11];
  uint8 temp2[4];
  uint8 pUTCTime[20];
  
  uint16 conv1,conv2;
  uint8 *p;
  
  uint8 temp3[4];
  uint8 temp4[4];
  uint8 temp5[10];
  
  uint16 microsec;
  uint16 millisec;
  uint16 sec;
  uint32 tempTime = BUILD_UINT32( data[0], data[1], data[2], data[3]);
  
  UTCTime utcSecs;
  UTCTimeStruct utcTime;
  
  utcSecs = osal_getClock();
  osal_ConvertUTCTime( &utcTime, utcSecs);

  utcTime.month++;
  utcTime.day++;
  p = pUTCTime;
  
  osal_memset( pUTCTime, '0', 19);
  if( utcTime.hour < 10)
    _itoa( (uint16)( utcTime.hour & 0x00FF), &pUTCTime[1], 10); 
  else
    _itoa( (uint16)( utcTime.hour & 0x00FF), pUTCTime, 10); 
  pUTCTime[2] = ':';
  
  if( utcTime.minutes < 10)
    _itoa( (uint16)( utcTime.minutes & 0x00FF), &pUTCTime[4], 10);
  else
    _itoa( (uint16)( utcTime.minutes & 0x00FF), &pUTCTime[3], 10);
  pUTCTime[5] = ':';
  
  if( utcTime.seconds < 10)
    _itoa( (uint16)( utcTime.seconds & 0x00FF), &pUTCTime[7], 10);
  else
    _itoa( (uint16)( utcTime.seconds & 0x00FF), &pUTCTime[6], 10);
  pUTCTime[8] = ' ';
  
  if( utcTime.month < 10)
    _itoa( (uint16)( utcTime.month & 0x00FF), &pUTCTime[10], 10);
  else
    _itoa( (uint16)( utcTime.month & 0x00FF), &pUTCTime[9], 10);
  pUTCTime[11] = '/';
  
  if( utcTime.day < 10)
    _itoa( (uint16)( utcTime.day & 0x00FF), &pUTCTime[13], 10);
  else
    _itoa( (uint16)( utcTime.day & 0x00FF), &pUTCTime[12], 10);
  pUTCTime[14] = '/';
  _itoa( utcTime.year, &pUTCTime[15], 10);

  for( uint8 i = 0; i < 18; i++, p++)
  {
    if( *p == 0)
      *p = '0';
  }
  pBuf = osal_mem_alloc( 256);
  
  osal_memset( pBuf,0x20, 256);
  pBuf[254] = '\r';
  pBuf[255] = '\n';
  
  _itoa( (uint16)modbusId, temp2, 10);
  
  if( pBuf)
  {
    switch( event)
    {
      case SD_RFID_EVENT:
        conv1 = BUILD_UINT16( data[1], data[0]);
        conv2 = BUILD_UINT16( data[3], data[2]);
        
        _itoa( conv1, temp1, 10);
        _itoa( conv2, &temp1[5], 10);
  
        sprintf( pBuf, "Event: RFID Log in\r\nModBus ID:%s\r\nRFID Num:%s\r\nTime:%s\r\n", temp2, temp1, pUTCTime);
        break;
      case SD_TIME_EVENT:
        
        microsec = tempTime % 1000;
        millisec = (tempTime / 1000) % 1000;
        sec = tempTime / 1000 /1000;
        
        _itoa( microsec, temp3, 10);
        _itoa( millisec, temp4, 10);
        _itoa( sec, temp5, 10);
        
        sprintf( pBuf, "Event: Time Record\r\nModBus ID:%s\r\nTime Data:%s Sec %s MilliSec %s MicroSec\r\nTime:%s\r\n", temp2, temp5, temp4, temp3, pUTCTime);
        break;
      case SD_TIME_RESULT:
        microsec = tempTime % 1000;
        millisec = (tempTime / 1000) % 1000;
        sec = tempTime / 1000 /1000;
        
        _itoa( microsec, temp3, 10);
        _itoa( millisec, temp4, 10);
        _itoa( sec, temp5, 10);
        sprintf( pBuf, "Event: Time Result\r\nModBus ID:%s\r\nResult:%s Sec %s MilliSec %s MicroSec\r\nTime:%s\r\n", temp2, temp5, temp4, temp3, pUTCTime);
        break;
      /*case SD_JUMP_EVENT:
        sec = BUILD_UINT16( data[0], data[1]);//(uint16)(data[0]&0x00ff);
        _itoa( sec, temp3, 10);
        sprintf( pBuf, "Event: Reach Record\r\nModBus ID:%s\r\nHeight:%s cm\r\nTime:%s\r\n", temp2, temp3, pUTCTime);
        break;
      case SD_JUMP_SCORE:
        sec = BUILD_UINT16( data[0], data[1]) - BUILD_UINT16( modbus_jump_height[0],modbus_jump_height[1]);
        _itoa( sec, temp3, 10);
        sprintf( pBuf, "Event: Jump Record\r\nModBus ID:%s\r\nVertical:%s cm\r\nTime:%s\r\n", temp2, temp3, pUTCTime);
        break;*/
      default:
        sprintf( pBuf, "UNKNOW EVENT\r\nMODBUS ID:%s\r\nDATA:%s\r\nTime:%s", temp2, temp1, pUTCTime);
        break;
    }
  }
  
//  mmc_write_file( (uint8*)pBuf, strlen(pBuf));
  fs = osal_mem_alloc( sizeof(FATFS));
  f_mount(0, fs);
  f_open( &sd_file, "0:/GATE.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
  f_lseek(&sd_file, f_size(&sd_file));
  f_write(&sd_file, (uint8 *)pBuf, 256, &bw);
  f_close(&sd_file);
  osal_mem_free( fs);
  osal_mem_free( pBuf);
#endif
}
