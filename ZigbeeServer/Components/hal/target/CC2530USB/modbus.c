#if defined ( MODBUS_SUPPLY)
/*********************************************************************
 * INCLUDE
 */
#include "modbus.h"
#include "hal_defs.h"
#include "hal_uart.h"
#include "GenericApp.h"
#include "AF.h"
#include "i2c.h"
#include "OnBoard.h"

#if defined ( RFID_SUPPLY)
  #include "OSAL_Clock.h"
#endif
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 ttt[6] = {11, 22, 33, 44, 55, 66};

uint8 const auchCRCHi[256] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;	
	/* Table of CRC values for high Corder byte */


uint8 const auchCRCLo[256] = {

0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40

} ;

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 CRClo, CRChi;
#if defined ( RFID_SUPPLY)
uint8 modbus_id = 254;
#else
uint8 modbus_id = 253;
#endif
uint8 hardware_ver = 0;
uint8 modbus_device_id = 0;
uint8 modbus_group_id = 2;
uint8 modbus_job_id = 0;
uint8 modbus_user_id = 0;
uint8 modbus_charging_stat = 1;
uint16 modbus_left_quantity = 0;
uint8 modbus_left_hours = 2;
#if defined ( JUMP_MACHINE_DONGLE)
  uint8 modbus_jump_height[2];
  uint8 modbus_jump_record[2];
#endif
#if defined ( RSSI_DISPLAY)
  bool panId_send = FALSE;
  uint8 rssi_send_hi, rssi_send_lo;
  int8 modbus_rssi;
#endif
bool restroe_factory_setting = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
// static uint16 CRC16( uint8 *puchMsg , uint8 usDataLen);
static void modbus_send_byte( uint8 byte, uint8 crc);
static bool check_data( uint8 *buf_com, uint8 len);
#if defined ( RFID_SUPPLY)
  static void modbus_process_msg( uint8 *data_buffer, uint8 len);
#endif
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
void modbus_uart_data_process( uint8 *data_buffer, uint8 len);

extern byte GenericApp_TransID;
extern endPointDesc_t GenericApp_epDesc;
/*********************************************************************
 * MAIN PROGRAM
 */

/******************************************************************************
 * @fn      initCRC16
 * 
 * @brief   Initialize the 16 bit CRC variables
 * 
 * @param   none
 * 
 * @return  none
 */
void initCRC16( void)
{
  CRClo = 0xff;
  CRChi = 0xff;
}
/*
static uint16 CRC16( uint8 *puchMsg , uint8 usDataLen)
{
  uint16 uchCRCHi = 0xff;
  uint8 uchCRCLo = 0xff;
  uint8 uIndex;
  
  while( usDataLen--)
  {
    uIndex = uchCRCHi ^ *puchMsg++ ; 
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
    uchCRCLo = auchCRCLo[uIndex] ;
  }  
  return ( uchCRCHi << 8 | uchCRCLo);
}
*/

/******************************************************************************
 * @fn      CRC16_byte
 * 
 * @brief   Calculate the 16 bit CRC variables
 * 
 * @param   uint8 - the byte wanted to be calc
 * 
 * @return  none
 */
void CRC16_byte( uint8 ch)
{
  uint8 uIndex;
  
  uIndex = CRChi ^ ch;
  CRChi = CRClo ^ auchCRCHi[uIndex];
  CRClo = auchCRCLo[uIndex];
}

#if defined ( RSSI_DISPLAY)
void calcCRC16( uint8* data, uint8 len)
{
  uint8 *pData = osal_mem_alloc( len);
  if( pData)
    osal_memcpy( pData, data, len);

  pData[rssi_send_hi] = 255;
  pData[rssi_send_lo] = modbus_rssi;
  initCRC16();
  for( uint8 i = 0; i<(len-2); i++)
  {
    CRC16_byte(pData[i]);
  }
  pData[len-2] = CRChi;
  pData[len-1] = CRClo;

  HalUARTWrite( 0, pData, len);
  osal_mem_free( pData);
}
#endif
/******************************************************************************
 * @fn      modbus_send_byte
 * 
 * @brief   Send one byte and calculate the crc byte
 * 
 * @param   uint8 - the byte wanted to be sent
 *          uint8 - the crc flag, need to calculate or not
 * 
 * @return  none
 */
static void modbus_send_byte( uint8 byte, uint8 crc)
{
  HalUARTWrite( 0, &byte, 1);
  if ( crc == CRC_NO)
    CRC16_byte(byte);
}

/******************************************************************************
 * @fn      modbus_uart_data_process
 * 
 * @brief   Process the message from UART
 * 
 * @param   uint8 - the pointer the buffer that to be process
 *          uint8 - length
 * 
 * @return  none
 */
void modbus_uart_data_process( uint8 *data_buffer, uint8 len)
{
  if( TRUE == check_data( data_buffer, len))
  {
    initCRC16();
#if defined ( RFID_SUPPLY)
    modbus_process_msg( data_buffer, len);
#endif
  }
}

/******************************************************************************
 * @fn      check_data
 * 
 * @brief   Check message to confirm it's a modbus message
 * 
 * @param   uint8 - the pointer the buffer that to be checked
 *          uint8 - length
 * 
 * @return  bool
 */

static bool check_data( uint8 *buf_com, uint8 len)
{
//  crc_check = CRC16( buf_com, len);
  
//  if( ( buf_com[6] != HI_UINT16(crc_check)) || ( buf_com[7] != LO_UINT16( crc_check)))
//    return FALSE;
  
  if( ( buf_com[1] != MODBUS_SINGLE_READ) && (buf_com[1] != MODBUS_SINGLE_WRITE) && ( buf_com[1] != MODBUS_MULTI_WRITE))
    return FALSE;
  
  return TRUE;
}

/******************************************************************************
 * @fn      modbus_setUtcTime
 * 
 * @brief   Set UTC Time
 * 
 * @param   uint8 - the pointer the buffer
 * 
 * @return  none
 */

#if defined ( RFID_SUPPLY)

static void modbus_setUtcTime( uint8 *pBuf)
{
  UTCTime utcSecs = 0;
  UTCTimeStruct utc;
  
  utc.hour = pBuf[1];
  utc.minutes = pBuf[3];
  utc.seconds = pBuf[5];
  utc.month = pBuf[7] - 1;
  utc.day = pBuf[9] - 1;
  utc.year = BUILD_UINT16( pBuf[11], pBuf[10]);
  
  if ((utc.hour < 24) && (utc.minutes < 60) && (utc.seconds < 60) &&
        (utc.month < 12) && (utc.day < 31) && (utc.year > 1999) && (utc.year < 2136))
  {
    if ((utc.month != 1) || (utc.day < (IsLeapYear( utc.year ) ? 29 : 28)))
    {
      utcSecs = osal_ConvertUTCSecs( &utc );
    }
  }
  
  if( utcSecs)
  {
    osal_setClock( utcSecs );
  }
}

/******************************************************************************
 * @fn      modbus_process_msg
 * 
 * @brief   Process modbus message
 * 
 * @param   uint8 - the pointer the buffer
 *          uint8 - length 
 * 
 * @return  none
 */

static void modbus_process_msg( uint8 *data_buffer, uint8 len)
{
  uint8 num, tempByte;
  uint8 zero = 0;
  uint16 i;
  uint16 address;
  
  UTCTime utcSecs;
  UTCTimeStruct utcTime;

  utcSecs = osal_getClock();
  osal_ConvertUTCTime( &utcTime, utcSecs );

  afAddrType_t destAddr;
  destAddr.addrMode = afAddrBroadcast;
  destAddr.addr.shortAddr = 0xffff;                
  destAddr.endPoint = 10;
  
  modbusId_t *pGate = pModbusId;
  rfidInfo_t *pRfid = pRfidInfo;
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if(( data_buffer[0] == modbus_id) || ( data_buffer[0] == 255))  
  {
    if(data_buffer[1] == MODBUS_SINGLE_WRITE)
    {
      HalUARTWrite( 0, data_buffer, len);
      if( address == MODBUS_ADDRESS)
      {
        modbus_id = data_buffer[5];
      }
      else if ( address == MODBUS_SYS_RESTORE)
      {
        restroe_factory_setting = TRUE;
        restore_factory_setting();
      }
      else if ( address == MODBUS_DEVICE_ID)
      {
        modbus_device_id = data_buffer[5];
      }
      else if ( address == MODBUS_GROUP_ID)
      {
        modbus_group_id = data_buffer[5];
      }
      else if( address == MODBUS_JOB_ID)
      {
        modbus_job_id = data_buffer[5];
      }
      else if( address == MODBUS_USER_ID)
      {
        modbus_user_id = data_buffer[5];
      }
      else if(( address >= MODBUS_GATE_DISTANCE_1) && ( address <MODBUS_GATE_TIME_1_SEC))
      {
        if ( address == MODBUS_GATE_DISTANCE_1)
        {
          pGate = pModbusId;
        }
        else
        {
          for( uint8 j = 0; j < ( address-MODBUS_GATE_DISTANCE_1); j++)
          {
            if( pGate)
              pGate = pGate->next;
          }
        }
        if( pGate)
        {
          afAddrType_t dest_Addr;
          dest_Addr.addrMode = afAddrBroadcast;
          dest_Addr.addr.shortAddr = pGate->shortAddr;                
          dest_Addr.endPoint = 10;
          
          uint8 temp_send[7] = {0x0c, 0xff, 0x0c, 0x00, 0x00, 0x00, 0x00};
  
          pGate->distance = data_buffer[5];
          
          temp_send[6] = data_buffer[5];
          for( uint8 k = 0; k < 3; k++)
          {
            AF_DataRequest( &dest_Addr, &GenericApp_epDesc,
                             MODBUS_PROTOCAL_CLUSTERID,
                             7,
                             temp_send,
                             &GenericApp_TransID,
                             AF_DISCV_ROUTE,
                             AF_DEFAULT_RADIUS );
          }
        }
      }
    }
    else if( data_buffer[1] == MODBUS_MULTI_WRITE)
    {
      // write system UTC
      if( address == MODBUS_SYS_HOUR)
      {
        if( data_buffer[6] >= 12)
          modbus_setUtcTime( &data_buffer[7]);
      }
      else if( (address >= MODBUS_RFID_NAME) && ( address < MODBUS_RFID_END))
      {
        rfidInfo_t* rfidInfo_list = findRfidInfo( rfid_num);
        if(  rfidInfo_list != NULL)
        {
          for( uint8 k = 0; k<10; k++)
            rfidInfo_list->rfidName[k] = data_buffer[7+k];
        }
      }
    }
    else if( data_buffer[1] == MODBUS_SINGLE_READ)
    {
      num = data_buffer[5];
      modbus_send_byte( data_buffer[0], CRC_NO);
      modbus_send_byte( data_buffer[1], CRC_NO);
      modbus_send_byte( num*2, CRC_NO);
      
      for( i = 0; i < num; i++)
      {
        if ( i + address <= MODBUS_SERIALNUMBER_LOWORD + 3)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[ i + address - MODBUS_SERIALNUMBER_LOWORD], CRC_NO);
        }
        else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_LO)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[4], CRC_NO);
        }
        else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_HI)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( ttt[5], CRC_NO);
        }
        else if( i + address == MODBUS_ADDRESS)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_id, CRC_NO);
        }
        else if( i + address == MODBUS_HARDWARE_REV)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( hardware_ver, CRC_NO);
        }
        // System UTC Time
        else if( i + address == MODBUS_SYS_HOUR)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.hour, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_MINUTES)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.minutes, CRC_NO);          
        }
        else if( i + address == MODBUS_SYS_SECONDS)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( utcTime.seconds, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_MONTH)
        {
          tempByte = utcTime.month + 1;
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_DAY)
        {
          tempByte = utcTime.day + 1;
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_SYS_YEAR)
        {
          tempByte = HI_UINT16( utcTime.year);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = LO_UINT16( utcTime.year);
          modbus_send_byte( tempByte, CRC_NO);
        }
        // restore flag, while true, restore to factory settings
        else if( i + address == MODBUS_SYS_RESTORE)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( restroe_factory_setting, CRC_NO);
        }
        // PAN ID info
        else if( i + address == MODBUS_PANID)
        {
          tempByte = HI_UINT16( _NIB.nwkPanId);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = LO_UINT16( _NIB.nwkPanId);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_DEVICE_ID)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_device_id, CRC_NO);
        }
        else if( i + address == MODBUS_GROUP_ID)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_group_id, CRC_NO);
        }
        else if( i + address == MODBUS_JOB_ID)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_job_id, CRC_NO);
        }
        else if( i + address == MODBUS_USER_ID)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_user_id, CRC_NO);
        }
        else if( i + address == MODBUS_CHARGING_STAT)
        {
          //modbus_charging_stat = Read_Data(0x02);
          //Read_Data(0x02);
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_charging_stat, CRC_NO);
        }
        else if( i + address == MODBUS_LEFT_QUANTITY)
        {
//          modbus_left_quantity = BUILD_UINT16(Read_Data( 0x09),Read_Data( 0x08));
          
          modbus_send_byte( ((modbus_left_quantity >> 8) & 0xff) , CRC_NO);
          modbus_send_byte( modbus_left_quantity, CRC_NO);
        }
        else if( i + address == MODBUS_LEFT_HOURS)
        {
          //modbus_left_hours = Read_Data( 0x01);
          //Read_Data( 0x01);
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbus_left_hours, CRC_NO);
        }
#if defined (JUMP_MACHINE_DONGLE)
        else if( i + address == MODBUS_JUMP_HEIGHT)
        {
          modbus_send_byte( modbus_jump_height[1] , CRC_NO);
          modbus_send_byte( modbus_jump_height[0], CRC_NO);
        }
        else if( i + address == MODBUS_JUMP_RECORD)
        {
          modbus_send_byte( modbus_jump_record[1], CRC_NO);
          modbus_send_byte( modbus_jump_record[0], CRC_NO);
        }
#endif
        else if( i + address == MODBUS_GATE_NUMS)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( modbusId_index, CRC_NO);
        }
        else if(( i + address >= MODBUS_GATE_ADDRESS_1) && ( i + address < MODBUS_GATE_DISTANCE_1))
        {
          if ( i + address == MODBUS_GATE_ADDRESS_1)
          {
            pGate = pModbusId;
          }
          else
          {
            pGate = pModbusId;
            for( uint8 j = 0; j < ( (i+address)-MODBUS_GATE_ADDRESS_1); j++)
            {
              if( pGate)
                pGate = pGate->next;
            }
          }
          modbus_send_byte( zero, CRC_NO);
          if( pGate !=NULL)
          {
            modbus_send_byte( pGate->modbusId, CRC_NO);
            pGate = pGate->next;
          }
          else
            modbus_send_byte( zero, CRC_NO);
        }
        else if(( i + address >= MODBUS_GATE_DISTANCE_1) && ( i + address < MODBUS_GATE_TIME_1_SEC))
        {
          if ( i + address == MODBUS_GATE_DISTANCE_1)
          {
            pGate = pModbusId;
          }
          else
          {
            pGate = pModbusId;
            for( uint8 j = 0; j < ( (i+address)-MODBUS_GATE_DISTANCE_1); j++)
            {
              if( pGate)
                pGate = pGate->next;
            }
          }
          modbus_send_byte( zero, CRC_NO);
          if( pGate !=NULL)
          {
            modbus_send_byte( pGate->distance, CRC_NO);
          }
          else
            modbus_send_byte( zero, CRC_NO);
        }
        // 计算两个时间之间的差
        else if( (i + address >= MODBUS_GATE_TIME_1_SEC) && ( i + address <MODBUS_GATE_TIME_1_MICROSEC))
        {
          if( i + address == MODBUS_GATE_TIME_1_SEC)
            pGate = pModbusId;
          else
          {
            pGate = pModbusId;
            for( uint8 j = 0; j< ((i + address)-MODBUS_GATE_TIME_1_SEC);j++)
            {
              if( pGate)
                pGate = pGate->next;
            }
          }
          if( pGate !=NULL)
          {
            tempByte = HI_UINT16( pGate->timeRecord/1000000);
            modbus_send_byte( tempByte, CRC_NO);
            tempByte = LO_UINT16( pGate->timeRecord/1000000);
            modbus_send_byte( tempByte, CRC_NO);
          }
          else
          {
            modbus_send_byte( zero, CRC_NO);
            modbus_send_byte( zero, CRC_NO);
          }
        }
        else if( (i + address >= MODBUS_GATE_TIME_1_MICROSEC) && ( i + address <MODBUS_GATE_FUNC_1_ID))
        {
          if( i + address == MODBUS_GATE_TIME_1_MICROSEC)
            pGate = pModbusId;
          else
          {
            pGate = pModbusId;
            for( uint8 j = 0; j< ((i + address)-MODBUS_GATE_TIME_1_MICROSEC);j++)
            {
              if( pGate)
                pGate = pGate->next;
            }
          }
          if( pGate !=NULL)
          {
            tempByte = HI_UINT16( ((pGate->timeRecord)/1000)%1000);
            modbus_send_byte( tempByte, CRC_NO);
            tempByte = LO_UINT16( ((pGate->timeRecord)/1000)%1000);
            modbus_send_byte( tempByte, CRC_NO);
          }
          else
          {
            modbus_send_byte( zero, CRC_NO);
            modbus_send_byte( zero, CRC_NO);
          }
        }
        else if( (i + address >= MODBUS_GATE_FUNC_1_ID) && ( i + address <MODBUS_RFID_NUM))
        {
          if ( i + address == MODBUS_GATE_FUNC_1_ID)
          {
            pGate = pModbusId;
          }
          else
          {
            pGate = pModbusId;
            for( uint8 j = 0; j < ( (i+address)-MODBUS_GATE_FUNC_1_ID); j++)
            {
              if( pGate)
                pGate = pGate->next;
            }
          }
          modbus_send_byte( zero, CRC_NO);
          if( pGate !=NULL)
          {
            modbus_send_byte( pGate->funcId, CRC_NO);
          }
          else
            modbus_send_byte( zero, CRC_NO);
        }
        else if( i + address == MODBUS_RFID_NUM)
        {
          modbus_send_byte( zero, CRC_NO);
          modbus_send_byte( rfidInfo_index, CRC_NO);
        }
        else if( (i+ address >= MODBUS_RFID_ID_START ) && (i+address <MODBUS_RFID_NAME))
        {
          pRfid = pRfidInfo;
          if( i+address != MODBUS_RFID_ID_START)
          {
            for( uint8 j= 0; j<( (i+address)-MODBUS_RFID_ID_START); j+=4)
            {
              if(pRfid)
                pRfid = pRfid->next;
            }
          }
          if( pRfid!= NULL)
          {
            for( uint8 n= 0; n<4; n++)
            {
              modbus_send_byte( zero, CRC_NO);
              modbus_send_byte( BREAK_UINT32(pRfid->rfidNum, n), CRC_NO);
            }
            i += 3;
          }
          else{
            modbus_send_byte( zero,CRC_NO);
            modbus_send_byte( zero,CRC_NO);
          }
        }
        else if( (i + address >= MODBUS_RFID_NAME ) && ( i+address < MODBUS_RFID_END))
        {
          pRfid = pRfidInfo;
          if ( i + address != MODBUS_RFID_NAME)
          {
            for( uint8 j = 0; j< ( (i+address) - MODBUS_RFID_NAME); j+=10)
            {
              if( pRfid)
                pRfid = pRfid->next;
            }
          }
          if ( pRfid != NULL)
          {
            for( uint8 n=0; n < 10; n++)
            {
              modbus_send_byte( zero,CRC_NO);
              modbus_send_byte( pRfid->rfidName[n],CRC_NO);
            }
            i += 9;
          }
          else
          {
            modbus_send_byte( zero,CRC_NO);
            modbus_send_byte( zero,CRC_NO);
          }
        }
 
        else
        {
          modbus_send_byte( 0, CRC_NO);
          modbus_send_byte( 1, CRC_NO);
        }
      }
      modbus_send_byte( CRChi, CRC_YES);
      modbus_send_byte( CRClo, CRC_YES);
    }
  }

  else
  {
#if defined ( RSSI_DISPLAY)
    if( data_buffer[1] == MODBUS_SINGLE_READ)
    {
      for( uint16 k = 0; k < 23; k++)
      {
        if( k == address)
        {
          panId_send = TRUE;
          rssi_send_hi = 47 - 2*k;
          rssi_send_lo = 48 - 2*k;
        }
      }
    }
#endif
    AF_DataRequest( &destAddr, &GenericApp_epDesc,
                     MODBUS_PROTOCAL_CLUSTERID,
                     len,
                     data_buffer,
                     &GenericApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS );
  }
}

#endif
#if 0
void modbus_addr_proc( uint8 *buf_com)
{
  switch( buf_com[1])
  {
  case MODBUS_READ:
    
    break;
  case MODBUS_SINGLE_WRITE:
    
    break;
  case MODBUS_MULTI_WRITE:
    
    break;
  default:
    
    break;
  }
}

#define DEFAULT_SERIAL_NUM        0
#define DEFAULT_SOFTWARE_VER      0
#define DEFAULT_MODBUS_ADDR       0xFF
#define DEFAULT_PRODUCT_MODEL     0
#define DEFAULT_FIRMWARE_VER      0
#define DEFAULT_HARDWARE_VER      0
#define DEFAULT_CHARGING_STAT     0
#define DEFAULT_NOT_CHARGING      0
#define DEFAULT_LEFT_QUANTITY     0
#define DEFAULT_LEFT_HOURS        0

uint32 modbus_serial_num = DEFAULT_SERIAL_NUM;
uint16 modbus_software_ver = DEFAULT_SOFTWARE_VER;
uint8 modbus_addr = DEFAULT_MODBUS_ADDR;
uint8 modbus_product_model = DEFAULT_PRODUCT_MODEL;
uint8 modbus_hardware_ver = DEFAULT_FIRMWARE_VER;
uint8 modbus_firmware_ver = DEFAULT_HARDWARE_VER;
uint8 modbus_charging_stat = DEFAULT_CHARGING_STAT;
uint8 modbus_not_charging = DEFAULT_NOT_CHARGING;
uint8 modbus_left_quantity = DEFAULT_LEFT_QUANTITY;
uint8 modbus_left_hours = DEFAULT_LEFT_HOURS;

#define MODBUS_SERIAL_NUM         0x00E1
#define MODBUS_SOFTWARE_VER       0x00E2
#define MODBUS_ADDRESS            0x00E3
#define MODBUS_PRODUCT_MODEL      0x00E4
#define MODBUS_HARDWARE_VER       0x0

static CONST zgItem_t ModbusItemTable[] = 
{
  {
    MODBUS_SERIAL_NUM, sizeof( modbus_serial_num), &mobus_serial_num
  },
  {
    MODBUS_SOFTWARE_VER, sizeof( modbus_software_ver), &modbus_software_ver
  },
  {
    MODBUS_ADDRESS, sizeof( modbus_addr), &modbus_addr
  },
  {
    MODBUS_PRODUCT_MODEL, sizeof( modbus_product_model), &modbus_product_model
  },
  {
    MODBUS_HARDWARE_VER, sizeof( modbus_hardware_ver), &modbus_hardware_ver
  },
  {
    MODBUS_FIRMWARE_VER, sizeof( modbus_firmware_ver), &modbus_firmware_ver
  },
  {
    MODBUS_CHARGING_STAT, sizeof( modbus_charging_stat), &modbus_charging_stat
  },
  {
    MODBUS_NOT_CHARGING, sizeof( modbus_not_charging), &modbus_not_charging
  },
  {
    MODBUS_LEFT_QUANTITY, sizeof( modbus_left_quantity), &modbus_left_quantity
  },
  {
    MODBUS_LEFT_HOURS, sizeof( modbus_left_quantity), &modbus_left_hours
  }
}

bool get_modbus_id( uint16 addr)
{

}

void modbus_data_resp( void)
{
  uint16 temp_array;
  uint8 id_add; 
}

 {

        uint8 startOptions, type;

        NLME_InitNV();

        NLME_SetDefaultNV();

        zgWriteStartupOptions( ZG_STARTUP_SET,ZCD_STARTOPT_DEFAULT_NETWORK_STATE );

        startOptions = ZCD_STARTOPT_CLEAR_STATE | ZCD_STARTOPT_CLEAR_CONFIG;

        osal_nv_write(ZCD_NV_STARTUP_OPTION, 0, sizeof(uint8),&startOptions);

        type = ZG_DEVICETYPE_ENDDEVICE;

        osal_nv_write(ZCD_NV_LOGICAL_TYPE, 0, sizeof(uint8), &type);

        zgWriteKeyCmdOptions(0xff,ZCD_NV_SEND_BEACON);

        SystemReset();

      } 
#if 0
        else if( i + address == MODBUS_GATE_TIME_1_HI)
        {
          tempByte = BREAK_UINT32( modbusId_list[0].timeRecord, 3);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = BREAK_UINT32( modbusId_list[0].timeRecord, 2);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_1_LO)
        {
          tempByte = BREAK_UINT32( modbusId_list[0].timeRecord, 1);
          modbus_send_byte( tempByte, CRC_NO);
          tempByte = BREAK_UINT32( modbusId_list[0].timeRecord, 0);
          modbus_send_byte( tempByte, CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_2_HI)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[1].timeRecord, 3), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[1].timeRecord, 2), CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_2_LO)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[1].timeRecord, 1), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[1].timeRecord, 0), CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_3_HI)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[2].timeRecord, 3), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[2].timeRecord, 2), CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_3_LO)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[2].timeRecord, 1), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[2].timeRecord, 0), CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_4_HI)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[3].timeRecord, 3), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[3].timeRecord, 2), CRC_NO);
        }
        else if( i + address == MODBUS_GATE_TIME_4_LO)
        {
          modbus_send_byte( BREAK_UINT32( modbusId_list[3].timeRecord, 1), CRC_NO);
          modbus_send_byte( BREAK_UINT32( modbusId_list[3].timeRecord, 0), CRC_NO);
        }
#endif
#endif
#endif // MODBUS_SUPPLY