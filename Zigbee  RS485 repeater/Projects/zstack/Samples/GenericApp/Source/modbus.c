#if defined ( MODBUS_SUPPLY)
/*********************************************************************
 * INCLUDE
 */
#include "modbus.h"
#include "hal_uart.h"
#include "OSAL_Clock.h"
#include "OnBoard.h"
#include "AF.h"
#include "OSAL_Nv.h"
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 ttt[6] = {0, 0, 0, 0, 0, 1};

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
uint8 modbus_id = 1;

extern uint8 modbus_set_baudrate;
extern uint8 numMonitor;
extern boatMonitor_t* pBoatMonitor;
//extern void send_str_Uart( uint8* data, uint8 len, uint8 port);
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void modbus_send_byte( uint8 byte, uint8 crc);
static bool check_data( uint8 *buf_com, uint8 len);
static void modbus_process_msg( uint8 *data_buffer, uint8 len);
static void initCRC16( void);
static void CRC16_byte( uint8 ch);
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
//void modbus_uart_data_process( uint8 *data_buffer, uint8 len);
void response_scan_cmd(void);
/******************************************************************************
 * @fn      initCRC16
 * 
 * @brief   Initialize the 16 bit CRC variables
 * 
 * @param   none
 * 
 * @return  none
 */
static void initCRC16( void)
{
  CRClo = 0xff;
  CRChi = 0xff;
}

/******************************************************************************
 * @fn      CRC16_byte
 * 
 * @brief   Calculate the 16 bit CRC variables
 * 
 * @param   uint8 - the byte wanted to be calc
 * 
 * @return  none
 */
static void CRC16_byte( uint8 ch)
{
  uint8 uIndex;
  
  uIndex = CRChi ^ ch;
  CRChi = CRClo ^ auchCRCHi[uIndex];
  CRClo = auchCRCLo[uIndex];
}

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
  //HalUARTWrite( 0, &byte, 1);
  send_str_Uart( &byte, 1, 0);
  //send_char_Uart(byte, 0);
  if ( crc == CRC_NO)
    CRC16_byte(byte);
}

void response_scan_cmd(void)
{
  uint8 *pBuf;
  initCRC16();
  pBuf = osal_mem_alloc(9);
  if(pBuf!=NULL)
  {
    pBuf[0]=0xff;
    pBuf[1]=0x19;
    pBuf[2]=modbus_id;
    pBuf[3]=ttt[0];
    pBuf[4]=ttt[1];
    pBuf[5]=ttt[2];
    pBuf[6]=ttt[3];
  }
  for(uint8 i=0;i<7;i++)
    CRC16_byte(pBuf[i]);
  pBuf[7]=CRChi;
  pBuf[8]=CRClo;
  send_str_Uart( pBuf, 9, 0);
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
  P2_1 = 0;
  if( TRUE == check_data( data_buffer, len))
  {
    initCRC16();
    modbus_process_msg( data_buffer, len);
  }
  P2_1 = 1;
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
  boatMonitor_t *pTempMonitor;
  
  UTCTime utcSecs;
  UTCTimeStruct utcTime;

  utcSecs = osal_getClock();
  osal_ConvertUTCTime( &utcTime, utcSecs );
  
  address = BUILD_UINT16( data_buffer[3], data_buffer[2]);
  
  if(( data_buffer[0] == modbus_id) || ( data_buffer[0] == 255))
  {
    if(data_buffer[1] == MODBUS_SINGLE_WRITE)
    {
      HalUARTWrite( 0, data_buffer, len);
      if( address == MODBUS_ADDRESS)
      {
        modbus_id = data_buffer[5];
        osal_nv_write( ZCD_NV_MODBUS_ID, 0, sizeof(modbus_id), &modbus_id);
      }
      if( address == MODBUS_BAUDRATE)
      {
        if((data_buffer[5] >=0) &&(data_buffer[5] <= 4))
        {
          modbus_set_baudrate = data_buffer[5];
          osal_nv_write( ZCD_NV_BAUDRATE, 0, sizeof(modbus_set_baudrate), &modbus_set_baudrate);
          restore_factory_setting();
        }
      }
    }
    else if( data_buffer[1] == MODBUS_MULTI_WRITE)
    {
      if( address == MODBUS_SERIALNUMBER_LOWORD)
      {
        if(data_buffer[6] == 8)
        {
          for( uint8 k=0;k<4;k++)
          {
            ttt[k] = data_buffer[2*k+8];
            osal_nv_write( ZCD_NV_SERIAL_NUM, 0, sizeof(ttt), ttt);
          }
        }
      }
      
      // write system UTC
      if( address == MODBUS_SYS_HOUR)
      {
        if( data_buffer[6] >= 12)
          modbus_setUtcTime( &data_buffer[7]);
      }
    }
    else if( data_buffer[1] == MODBUS_SINGLE_READ)
    {
      num = data_buffer[5];
      uint8 *pBuf;
      uint8 index=0;
      pBuf = osal_mem_alloc(num*2+5);
      if( pBuf != NULL)
      {
        pBuf[index++] = data_buffer[0];
        pBuf[index++] = data_buffer[1];
        pBuf[index++] = num*2;
      //modbus_send_byte( data_buffer[0], CRC_NO);
      //modbus_send_byte( data_buffer[1], CRC_NO);
      //modbus_send_byte( num*2, CRC_NO);
        for( i = 0; i < num; i++)
        {
          if ( i + address <= MODBUS_SERIALNUMBER_LOWORD + 3)
          {
            //modbus_send_byte( zero, CRC_NO);
            pBuf[index++] = zero;
            pBuf[index++] =  ttt[ i + address - MODBUS_SERIALNUMBER_LOWORD];
          }
          else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_LO)
          {
            pBuf[index++] = zero;
            pBuf[index++] = ttt[4];
          }
          else if( i + address == MODBUS_FIRMWARE_VERSION_NUMBER_HI)
          {
            pBuf[index++] = zero;
            pBuf[index++] = ttt[5];
          }
          else if( i + address == MODBUS_ADDRESS)
          {
            pBuf[index++] = zero;
            pBuf[index++] = modbus_id;
          }
          else if( i + address == MODBUS_PRODUCT_ID)
          {
            pBuf[index++] = zero;
            pBuf[index++] = 210;
          }
          // System UTC Time
          else if( i + address == MODBUS_SYS_HOUR)
          {
            pBuf[index++] = zero;
            pBuf[index++] = utcTime.hour;
          }
          else if( i + address == MODBUS_SYS_MINUTES)
          {
            pBuf[index++] = zero;
            pBuf[index++] = utcTime.minutes;          
          }
          else if( i + address == MODBUS_SYS_SECONDS)
          {
            pBuf[index++] = zero;
            pBuf[index++] = utcTime.seconds;
          }
          else if( i + address == MODBUS_SYS_MONTH)
          {
            tempByte = utcTime.month + 1;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_SYS_DAY)
          {
            tempByte = utcTime.day + 1;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_SYS_YEAR)
          {
            tempByte = HI_UINT16( utcTime.year);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( utcTime.year);
            pBuf[index++] = tempByte;
          }
          /*else if( i + address == MODBUS_WATER_SWITCH)
          {
            if(leaveTimeCounter){
              tempByte = water_switch;
              //leaveTimeCounter=0;
            }
            else
              tempByte = zero;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_DETECT_POWER)
          {
            uint16 power_temp;
            if(leaveTimeCounter){
              power_temp = (ad_power-196)*10/10.57+90;//(ad_power-186)*100/74+90;//((ad_power-644)*64)/100+70;
              //leaveTimeCounter = 0;
            }else
              power_temp = 0;
            
            tempByte = HI_UINT16( power_temp);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( power_temp);
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_WATER_ID)
          {
            if(leaveTimeCounter){
              tempByte = water_id;
              //leaveTimeCounter=0;
            }
            else
              tempByte = zero;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_WATER_RSSI)
          {
            if(leaveTimeCounter){
              tempByte = (uint8)water_rssi;
              
            }
            else
              tempByte = zero;
            pBuf[index++] = 255;
            pBuf[index++] = tempByte;
          }
          
          else if( i + address == MODBUS_WATER_SWITCH1)
          {
            if(leaveTimeCounter1){
              tempByte = water_switch1;
              //leaveTimeCounter=0;
            }
            else
              tempByte = zero;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_DETECT_POWER1)
          {
            uint16 power_temp;
            if(leaveTimeCounter1){
              power_temp = (ad_power1-196)*10/10.57+90;//(ad_power-186)*100/74+90;//((ad_power-644)*64)/100+70;
              //leaveTimeCounter = 0;
            }else
              power_temp = 0;
            
            tempByte = HI_UINT16( power_temp);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( power_temp);
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_WATER_ID1)
          {
            if(leaveTimeCounter1){
              tempByte = water_id1;
              //leaveTimeCounter=0;
            }
            else
              tempByte = zero;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }*/
          else if( i + address == MODBUS_BAUDRATE)
          {
            tempByte = modbus_set_baudrate;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          
          else if( i + address == MODBUS_PANID)
          {
            tempByte = HI_UINT16( _NIB.nwkPanId);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( _NIB.nwkPanId);
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_DEVICE_TYPE)
          {
            tempByte = zgDeviceLogicalType;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_CHANNEL_LIST_HI)
          {
            tempByte = BREAK_UINT32(zgDefaultChannelList, 3);
            pBuf[index++] = tempByte;
            tempByte = BREAK_UINT32(zgDefaultChannelList, 2);
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_CHANNEL_LIST_LO)
          {
            tempByte = BREAK_UINT32(zgDefaultChannelList, 1);
            pBuf[index++] = tempByte;
            tempByte = BREAK_UINT32(zgDefaultChannelList, 0);
            pBuf[index++] = tempByte;
          }
          else if( (i + address >= MODBUS_EXTENDED_ADDR_HI) && (i + address <= MODBUS_EXTENDED_ADDR_LO))
          {
            tempByte = aExtendedAddress[ i+address-MODBUS_EXTENDED_ADDR_HI];
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( (i + address >= MODBUS_SECURITY_KEY_START) && (i + address <= MODBUS_SECURITY_KEY_END))
          {
            tempByte = defaultKey[ i+address-MODBUS_SECURITY_KEY_START];
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          /*else if( i + address == MODBUS_DETECT_PIN1)
          {
            uint16 power_temp;
            if(leaveTimeCounter){
              power_temp = detectPin1;
            }else
              power_temp = 0;
            
            tempByte = HI_UINT16( power_temp);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( power_temp);
            pBuf[index++] = tempByte;
          }
          else if( i + address == MODBUS_DETECT_PIN2)
          {
            uint16 power_temp;
            if(leaveTimeCounter){
              power_temp = detectPin2;
              //leaveTimeCounter=0;
            }else
              power_temp = 0;
            
            tempByte = HI_UINT16( power_temp);
            pBuf[index++] = tempByte;
            tempByte = LO_UINT16( power_temp);
            pBuf[index++] = tempByte;
          }*/
          else if( i + address == MODBUS_MONITOR_NUM)
          {
            tempByte = numMonitor;
            pBuf[index++] = zero;
            pBuf[index++] = tempByte;
          }
          else if( (i + address >= MODBUS_FIRST_MONITOR_ID) && ( i+ address <= MODBUS_LAST_MONITOR_ID))
          {
            if( i + address == MODBUS_FIRST_MONITOR_ID)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_MONITOR_ID); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            pBuf[index++]= zero;
            if(pTempMonitor != NULL)
            {
              pBuf[index++]= pTempMonitor->modbus_id;
            }
            else
              pBuf[index++]= zero;
          }
          else if( (i + address >= MODBUS_FIRST_DETECT_POWER) && ( i+ address <= MODBUS_LAST_DETECT_POWER))
          {
            if( i + address == MODBUS_FIRST_DETECT_POWER)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_DETECT_POWER); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            if(pTempMonitor != NULL)
            { 
              uint16 power_temp;
              power_temp = (pTempMonitor->ad_power-196)*10/10.57+90;
              tempByte = HI_UINT16( power_temp);
              pBuf[index++] = tempByte;
              tempByte = LO_UINT16( power_temp);
              pBuf[index++] = tempByte;
            }
            else
            {
              pBuf[index++]= zero;
              pBuf[index++]= zero;
            }
          }
          else if( (i + address >= MODBUS_FIRST_WATER_SWITCH) && ( i+ address <= MODBUS_LAST_WATER_SWITCH))
          {
            if( i + address == MODBUS_FIRST_WATER_SWITCH)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_WATER_SWITCH); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            pBuf[index++]= zero;
            if(pTempMonitor != NULL)
            {
              pBuf[index++]= pTempMonitor->water_switch;
            }
            else
              pBuf[index++]= zero;
          }
          else if((i + address >= MODBUS_FIRST_DETECT_PIN1) && ( i+ address <= MODBUS_LAST_DETECT_PIN1))
          {
            if( i + address == MODBUS_FIRST_DETECT_PIN1)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_DETECT_PIN1); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            if(pTempMonitor != NULL)
            { 
              tempByte = HI_UINT16( pTempMonitor->detectPin1);
              pBuf[index++] = tempByte;
              tempByte = LO_UINT16( pTempMonitor->detectPin1);
              pBuf[index++] = tempByte;
            }
            else
            {
              pBuf[index++]= zero;
              pBuf[index++]= zero;
            }
          } 
          else if((i + address >= MODBUS_FIRST_DETECT_PIN2) && ( i+ address <= MODBUS_LAST_DETECT_PIN2))
          {
            if( i + address == MODBUS_FIRST_DETECT_PIN2)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_DETECT_PIN2); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            if(pTempMonitor != NULL)
            { 
              tempByte = HI_UINT16( pTempMonitor->detectPin2);
              pBuf[index++] = tempByte;
              tempByte = LO_UINT16( pTempMonitor->detectPin2);
              pBuf[index++] = tempByte;
            }
            else
            {
              pBuf[index++]= zero;
              pBuf[index++]= zero;
            }
          }
          else if( (i + address >= MODBUS_FIRST_RSSI) && ( i+ address <= MODBUS_LAST_RSSI))
          {
            if( i + address == MODBUS_FIRST_RSSI)
            {
              pTempMonitor = pBoatMonitor;
            }
            else
            {
              pTempMonitor = pBoatMonitor;
              for( uint8 j=0; j<((i+address)-MODBUS_FIRST_RSSI); j++)
              {
                if(pTempMonitor)
                  pTempMonitor = pTempMonitor->next;
              }
            }
            if(pTempMonitor->rssi>=0)
              pBuf[index++] = 0;
            else
              pBuf[index++]= 0xff;
            
            if(pTempMonitor != NULL)
            {
              pBuf[index++]= pTempMonitor->rssi;
            }
            else
              pBuf[index++]= zero;
          }
          
          
          else
          {
            pBuf[index++] = zero;
            pBuf[index++] = zero;
          }
        }
        for( i=0; i<num*2+3; i++)
          CRC16_byte(pBuf[i]);
        pBuf[num*2+3] = CRChi;
        pBuf[num*2+4] = CRClo;
        P1_0 = 1;
        send_str_Uart( pBuf, num*2+5, 0);
        P1_0 = 0;
        osal_mem_free(pBuf);
      }
      else{
        send_str_Uart( pBuf, num*2+5, 0);
      }
    }
  }
}
#endif  // MODBUS_SUPPLY