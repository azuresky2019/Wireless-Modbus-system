#if 1
#if defined ( RFID_SUPPLY)
/******************************************************************************
 * INCLUDE
 */
#include "hal_mcu.h"
#include "rfid.h"
#include "MT.h"
#include "GenericApp.h"
#include "osal.h"
#include "mmc.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "OnBoard.h"
#include "modbus.h"
#include "af.h"

/******************************************************************************
 * CONTANTS
 */
#define MAX_CLK_CNT 400
#define MIN_CLK_CNT 200

#define LOW		0
#define HIGH	        1

/******************************************************************************
 * MACROS
 */

#define RFID_CLK_IN   P0_1
#define RFID_DATA_IN  P0_0

#define TIMER3_INIT(timer)  \
  st(                       \
    T##timer##CTL = 0x06;   \
    T##timer##CCTL0 = 0x00; \
    T##timer##CC0 = 0x00;   \
    T##timer##CCTL1 = 0x00; \
    T##timer##CC1 = 0x00;)
        
#define TIMER3_ENABLE_INT(timer, val) \
  (T##timer##CTL = (val) ? T##timer##CTL | 0x08 :T##timer##CTL & ~0x08)

#define TIMER3_SET_CLOCK_DIVIDE(timer, val) \
  st(                                       \
    T##timer##CTL &= ~0xe0;                 \
      (val == 2) ? (T##timer##CTL |=0x20):  \
      (val == 4) ? (T##timer##CTL |= 0x40): \
      (val == 8) ? (T##timer##CTL |= 0x60):  \
      (val == 16) ? (T##timer##CTL |= 0x80):  \
      (val == 32) ? (T##timer##CTL |= 0xa0):  \
      (val == 64) ? (T##timer##CTL |= 0xc0):  \
      (val == 128) ? (T##timer##CTL |= 0xe0):  \
      (T##timer##CTL|=0X00);)
                    
#define TIMER3_SET_MODE(timer,val)                \
  st(                                             \
    T##timer##CTL &= ~0X03;                               \
    (val==1)?(T##timer##CTL|=0X01):  /*DOWN            */ \
    (val==2)?(T##timer##CTL|=0X02):  /*Modulo          */ \
    (val==3)?(T##timer##CTL|=0X03):  /*UP / DOWN       */ \
    (T##timer##CTL|=0X00);)

#define TIMER3_START(timer,val)                         \
    (T##timer##CTL = (val) ? T##timer##CTL | 0X10 : T##timer##CTL&~0X10)

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 rfid_dat[64];  // rfid have 64 bits
static uint8 rfid_dat1[64];

static uint8 current_CLK = 0;
static uint8 pre_CLK = 0;
static uint8 flag_no_card = false;

static uint8 loop = 0;
volatile uint8 temp[8];
static uint8 rfid_buf[4];
static uint8 flag_header = false;
static uint8 index = 0;

uint8 name_num = 0;
/******************************************************************************
 * GLOBAL VARIABLES
 */
uint8 rfid_lock_flag = 0;
extern endPointDesc_t GenericApp_epDesc;
//extern uint8 time_reduce_flag ;
/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void RFID_Get_ID(void);


/******************************************************************************
 * @fn      RFID_init
 * 
 * @brief   Initialize the RFID support
 * 
 * @param   none
 * 
 * @return  none
 */
void RFID_init( void)
{
  P0SEL |= 0x03;
  P0DIR &= ~0x03;
  
  TIMER3_INIT(3);                  //初始化T4
  TIMER3_ENABLE_INT(3,1);  //开T4中断
  HAL_ENABLE_INTERRUPTS();
  T3IE = 1;

  TIMER3_SET_CLOCK_DIVIDE(3,8);
  TIMER3_SET_MODE(3,0);                 //自动重装00－>0xff
  TIMER3_START(3,1);                    //启动
}

/******************************************************************************
 * @fn      HAL_ISR_FUNCTION
 * 
 * @brief   Timer 3 ISR
 * 
 * @param   
 * 
 * @return  
 */

HAL_ISR_FUNCTION(T3_ISR,T3_VECTOR)
{
  HAL_ENTER_ISR();
  
  IRCON = 0x00;
  
  RFID_Get_ID();
  
  HAL_EXIT_ISR();
  //P1_0 = ~P1_0;
}

uint8 dec_buf[11];
/******************************************************************************
 * @fn      RFID_filter
 * 
 * @brief   To filter the invalid number
 * 
 * @param   none
 * 
 * @return  none
 */
void RFID_filter(void)
{
  uint8 loop1;	
  uint8 loop2;
  uint8 headernum = 0;
  
//  uint16 conv1,conv2;
//  uint8 conv_buf1[5], conv_buf2[5];
  // 9 continuous '1'
//  uint8 string_to_store[32];
  
  if(flag_no_card == false)
  {
    flag_header = 0;
    for(loop = 0;loop < 64 && flag_header == 0;loop++)
    {		
      //  get header , 9 continous '1'
      if(rfid_dat[loop] == HIGH)
      {
        if(headernum < 8)
          headernum++;
        else// if(flag_header == 0)
        {
          flag_header = 1;
          headernum = 0;
          // get header
          
          for(loop1 = 0;loop1 < 64 - loop + 8;loop1++)
          {
            rfid_dat1[loop1] = rfid_dat[loop1 + loop - 8];
          }
          
          for(loop2 = 0;loop2 < loop - 8;loop2++)
          {
            rfid_dat1[loop1 + loop2] = rfid_dat[loop2];
          }
  //  tbd : add verify
      // 每行数据为4 位数据位 + 1 位偶校验位
          
               uint8 check = 0;
              uint8 dat = 0;
                for(uint8 i = 0;i < 10;i++)
               {
                  check = 0;
                  dat = 0;
                  dec_buf[i] = 0;
                  for(uint8 j = 0;j < 5;j++)
                  {                     
                     if(rfid_dat1[i * 5 + j + 9] == 1)
                     {
                        if(j < 4)
                           dat += (0x08 >> (j % 4));
                        check++;
                     }
   
                  }
                 // if(i > 1) // 暂时不检查第一个字节，接收数据不正确， 需要修改
                  {
                     if ((check & 0x01) != 0)
                       {
                        //Lcd_Show_String(2,12,"ERR ",1,10);
                    //     HalUARTWrite( 0, "AAAAAAAA", 8);
                           return ;
                       }
                  }
                  dec_buf[i] = dat;

               }
               if(!rfid_lock_flag)
            {
                rfid_buf[0] = dec_buf[8] * 16 + dec_buf[9];//(((dec_buf[8] << 3) & 0xf0) | (dec_buf[9] >> 1));
                rfid_buf[1] = dec_buf[6] * 16 + dec_buf[7];//(((dec_buf[6] << 3) & 0xf0) | (dec_buf[7] >> 1));
                rfid_buf[2] = dec_buf[4] * 16 + dec_buf[5];//(((dec_buf[4] << 3) & 0xf0) | (dec_buf[5] >> 1));
                rfid_buf[3] = dec_buf[2] * 16 + dec_buf[3];//(((dec_buf[2] << 3) & 0xf0) | (dec_buf[3] >> 1));
                
              //  HalUARTWrite( 0, rfid_buf, 4);
             /*for(loop = 0;loop < 64;loop++)
               {
                  //Lcd_Show_Data(loop/21 + 2,loop%21,rfid_dat1[loop]);
                   //MT_BuildAndSendZToolResponse( 0x67, 0x16, 1, rfid_dat1);  
                 HalUARTWrite( 0, &rfid_dat1[loop], 1);
               } */  
                
               //time_reduce_flag = 0; 
              rfid_num = BUILD_UINT32( rfid_buf[0],
                                    rfid_buf[1],
                                    rfid_buf[2],
                                    rfid_buf[3]);
              rfidInfo_t* rfidInfo_list = (rfidInfo_t*)NULL;
                afStatus_t status_rfid = afStatus_INVALID_PARAMETER;
              rfidInfo_list = findRfidInfo( rfid_num);
              
              
              if( rfidInfo_list == NULL) 
              {
                registerRfidInfo( rfid_num);
                rfidInfo_index++;
                
              }
             //   rfidInfo_list = findRfidInfo( rfid_num);
                
                if( rfid_num == 0x003f9735)//(rfid_buf[0] == 0x35)&& (rfid_buf[1] == 0x97) &&(rfid_buf[2]==0x3f) && (rfid_buf[3]==0x00))
                {
              //    if( rfidInfo_list != NULL)
               //     osal_memcpy( rfidInfo_list->rfidName, "Green", 5);
                  name_num = 1;
                  
                }
                else if( rfid_num == 0x00866fa2)//(rfid_buf[0] == 0xa2)&& (rfid_buf[1] == 0x6f) &&(rfid_buf[2]==0x86) && (rfid_buf[3]==0x00))
                 {
              //     if( rfidInfo_list != NULL)
                //    osal_memcpy( rfidInfo_list->rfidName, "Orange", 6);
                   name_num = 2;
                 }
                else if( rfid_num == 0x003f3277)//(rfid_buf[0] == 0x77)&& (rfid_buf[1] == 0x32) &&(rfid_buf[2]==0x3f) && (rfid_buf[3]==0x00))
                 {
              //     if( rfidInfo_list != NULL)
                //    osal_memcpy( rfidInfo_list->rfidName, "Pink", 4);
                   name_num = 3;
                 }
                else
                {
               //   if( rfidInfo_list != NULL)
                //    osal_memcpy( rfidInfo_list->rfidName, "Error", 5);
                  name_num = 4;
                }
/*
              uint8 rfidNumMsg[29] = {0xff, 0x10, 0x00, 0xbf, 0x00, 0x0a, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00,0x00}; 
      
              afAddrType_t destAddr;
              uint8 TransID;
              
              TransID = 0;
              destAddr.addrMode = afAddrBroadcast;
              destAddr.endPoint = AF_BROADCAST_ENDPOINT;
              destAddr.addr.shortAddr = 0xFFFF;
              
              osal_memcpy( &rfidNumMsg[7],rfidInfo_list->rfidName,10);
              
              initCRC16();
              for( uint8 k = 0; k<29;k++)
              {
                CRC16_byte(rfidNumMsg[k]);
              }
              rfidNumMsg[27] = CRChi;
              rfidNumMsg[28] = CRClo;
              afStatus_t status_rfid = afStatus_INVALID_PARAMETER;
              status_rfid = AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                         0x0004,
                                         29,
                                         rfidNumMsg,
                                         &TransID,
                                         AF_DISCV_ROUTE,
                                         AF_DEFAULT_RADIUS );    
*/
              
                afAddrType_t destAddr;
                uint8 TransID;
                
                TransID = 0;
                destAddr.addrMode = afAddrBroadcast;
                destAddr.endPoint = AF_BROADCAST_ENDPOINT;
                destAddr.addr.shortAddr = 0xFFFF;
                
#if !defined ( JUMP_MACHINE_DONGLE)
                status_rfid = AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                           GATE_RFID_RECORD,
                                           1,
                                           &name_num,
                                           &TransID,
                                           AF_DISCV_ROUTE,
                                           AF_DEFAULT_RADIUS ); 
                name_num = 0;
#else
                uint8 rfid_sent[7] = {10,255,10, 0,0,0,0};
              
              rfid_sent[3] = rfid_buf[0];
              rfid_sent[4] = rfid_buf[1];
              rfid_sent[5] = rfid_buf[2];
              rfid_sent[6] = rfid_buf[3];
                status_rfid = AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                           GATE_PROTOCAL_CLUSTERID,
                                           7,
                                           rfid_sent,
                                           &TransID,
                                           AF_DISCV_ROUTE,
                                           AF_DEFAULT_RADIUS );
#endif
              rfid_lock_flag = 1;
              mmc_modbusId = modbus_id;
              mmc_event = SD_RFID_EVENT;
              P0_6 = 1;
            //  P1_0 = 0;
          //MicroWait( 1000);
           //   osal_start_timerEx( GenericApp_TaskID, GENERIC_WRITE_MMC, WRITE_MMC_TIMEOUT);
              if( status_rfid == afStatus_SUCCESS)
              {
                mmc_write_event( modbus_id, rfid_buf, SD_RFID_EVENT);
             //   P0_6 = 0;
            //    P1_0 = 1;
              }
              else 
              {
                SystemReset();
              }
              osal_start_timerEx( GenericApp_TaskID, RFID_DELAY_FLAG, 500);
            }
        }
      }
      else
      {
        flag_header = 0;
        headernum = 0;
      }
    }		
  }
}

/******************************************************************************
 * @fn      RFID_Get_ID
 * 
 * @brief   To get the number from the two wire of RFID
 * 
 * @param   none
 * 
 * @return  none
 */

static void RFID_Get_ID(void)
{
  static uint8 high = 0;
  static uint8 low = 0;
	
	
  if(RFID_CLK_IN)
    current_CLK = 1;
  else 
    current_CLK = 0;
          
  if(current_CLK == pre_CLK)
  {
    if(current_CLK == HIGH)
          high++;
    else
          low++;
    if(high > 6 )
    {
  //    P0_6 = ~P0_6;
      high = 0;flag_no_card = true;index = 0;
    }
    if(low > 6)
    {
      low = 0;flag_no_card = true;index = 0;
    }
  }
  else if((current_CLK == HIGH) && (pre_CLK == LOW))
  {
    //low = 0;
    
    high = 0;
    //LCD_BACK_OFF;
    if(index >= 64)		
    {
      index = 0;
      flag_no_card = false;
    }
    
    if(RFID_DATA_IN)
      rfid_dat[index++] = HIGH;
    else 
      rfid_dat[index++] = LOW;			

  }
  else if((current_CLK == LOW) && (pre_CLK == HIGH))
  {
    //high = 0;
    low = 0;

  }
  
  pre_CLK = current_CLK;
	
}
#endif
#endif



#if 0
          for(loop = 0;loop < 8;loop++)
          {
            temp[loop] = 0;
          }
          for(loop = 9;loop < 64;loop++)
          {
            if(rfid_dat1[loop] == HIGH)
              temp[(loop - 9) / 8] += (0x80 >> ((loop - 9) % 8)); 
          }
          for(loop = 1;loop < 5;loop++)
          {
            rfid_buf[loop-1] = temp[loop];
          }
          if((rfid_buf[0] == 0x61 && rfid_buf[1] == 0xf9 && rfid_buf[2] == 0xea && rfid_buf[3] == 0x30)
             || (rfid_buf[0] == 205 && rfid_buf[1] == 9 && rfid_buf[2] == 211 && rfid_buf[3] == 72)
             || (rfid_buf[0] == 31 && rfid_buf[1] == 106 && rfid_buf[2] == 17 && rfid_buf[3] == 240)
             || (rfid_buf[0] == 13 && rfid_buf[1] == 110 && rfid_buf[2] == 114 && rfid_buf[3] == 208))
          {
    /*        rfid_num = BUILD_UINT32( rfid_buf[0],
                                    rfid_buf[1],
                                    rfid_buf[2],
                                    rfid_buf[3]); */
            if(!rfid_lock_flag)
            {
   //           MT_BuildAndSendZToolResponse( 0x67, 0x16, 4, rfid_buf);
 /*             
              conv1 = BUILD_UINT16( rfid_buf[1], rfid_buf[0]);
              conv2 = BUILD_UINT16( rfid_buf[3], rfid_buf[2]);
              
              _itoa( conv1, conv_buf1, 10);
              _itoa( conv2, conv_buf2, 10);
              
              osal_memcpy( string_to_store, "RFID Number:", 12);
              for( i = 0; i < 5; i++)
              {
                string_to_store[i+13] = conv_buf1[i];
                string_to_store[i+18] = conv_buf2[i];
              }
              mmc_write_file( string_to_store, 24);
              */
              rfid_num = BUILD_UINT32( rfid_buf[0],
                                    rfid_buf[1],
                                    rfid_buf[2],
                                    rfid_buf[3]);
              mmc_write_event( modbus_id, rfid_buf, SD_RFID_EVENT);
              
              afAddrType_t destAddr;
              uint8 TransID;
              uint8 rfid_sent[7] = {10,255,10, 0,0,0,0};
              
              rfid_sent[3] = rfid_buf[0];
              rfid_sent[4] = rfid_buf[1];
              rfid_sent[5] = rfid_buf[2];
              rfid_sent[6] = rfid_buf[3];
              
              TransID = 0;
              destAddr.addrMode = afAddrBroadcast;
              destAddr.endPoint = AF_BROADCAST_ENDPOINT;
              destAddr.addr.shortAddr = 0xFFFF;
              
              AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                           GATE_PROTOCAL_CLUSTERID,
                                           7,
                                           rfid_sent,
                                           &TransID,
                                           AF_DISCV_ROUTE,
                                           AF_DEFAULT_RADIUS );
              
              rfid_lock_flag = 1;
          P0SEL &= ~BV(6);
          P0DIR |= BV(6);
          
              osal_start_timerEx( GenericApp_TaskID, RFID_DELAY_FLAG, 2000);
            }
          }
#endif