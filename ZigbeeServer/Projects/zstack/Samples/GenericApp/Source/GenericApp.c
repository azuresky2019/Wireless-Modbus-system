/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

//#include "hal_sd.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

#if defined ( GATE_PROTOCAL)
#include "MT.h"
#endif

#if defined ( RFID_SUPPLY)
#include "rfid.h"
#include "mmc.h"
#include "i2c.h"
#endif

#include "modbus.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
#if defined ( CC2530_TIME_SYNC_ROUTER) || defined (CC2531_TIME_SYNC_COORD)
  TIME_SYNC_CLUSTERID,
#endif
#if defined ( MODBUS_SUPPLY)
  MODBUS_PROTOCAL_CLUSTERID,
#endif
#if defined ( GATE_PROTOCAL)
  GATE_PROTOCAL_CLUSTERID
#endif
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

//uint8 sd_buf[520];    // test SD, LJ

#if defined ( RFID_SUPPLY)
  modbusId_t* pModbusId;
  rfidInfo_t* pRfidInfo;
  
  uint8 modbusId_index = 0;
  uint8 rfidInfo_index = 0;
  
  uint32 rfid_num;
  //uint32 rfid_name;
  uint16 jump_record;
  uint8 mmc_modbusId;
  uint8 mmc_event;
#else
  signalStrength_t *pSignalStren;
  signalStrength_t *pRssiIndex;
  uint8 numSignalStren = 0;
  uint8 rfidName[29] = {0xff, 0x10, 0x00, 0xbf, 0x00, 0x0a, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00,0x00}; ;
  bool confirmRfidName = FALSE;
#endif

#if defined ( JUMP_MACHINE)

  bool confirm_rfid = FALSE;
  uint8 rfid_buf[12];
  uint8 num_confirm_rfid = 0;
  Tx_data_t TxBuffer;
#endif
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
  uint8 time_sync_lock_flag = 1;
#endif
  
  
  
//  uint8 test_clock[] = {0xff,0x10,0x00,0x08,0x00,0x06,0x0c,0x00,0x0f,0x00,0x0f,0x00,0x3a,0x00,0x0c,0x00,0x0c,0x07,0xdc,0x2c,0xa1};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
#if defined ( RFID_SUPPLY)
  static Status_t registerGate( uint16 shortAddr, uint8 modbusId);
  static modbusId_t* findGateByAddr( uint16 shortAddr);
  static modbusId_t* findGateById( uint8 modbusId);
#else
  static Status_t register_signalStrength( uint8 modbus_id, int8 rssi);
  static signalStrength_t *findSignalStrength( uint8 modbus_id);
#endif

#if 1
#if defined ( RFID_SUPPLY)
static void battery_manage_init( void);
#endif
#endif

#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
static void GenericApp_SendTimeSyncReq( afAddrType_t *destAddr);
static void GenericApp_SendBroadcastSyncReq( void);
#endif
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
//  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

  P0SEL &= ~BV(6);
  P0DIR |= BV(6);
  P0_6 = 0;
  
#if defined ( RFID_SUPPLY)
  pModbusId = (modbusId_t *)NULL;
  battery_manage_init();
  mmc_init();
  RFID_init();
//  I2c_init();
  
  osal_start_timerEx( GenericApp_TaskID, BATTERY_MANAGEMENT, 3000);
#else
  pSignalStren = ( signalStrength_t*)NULL;
  pRssiIndex = pSignalStren;
#endif
//  modbus_uart_data_process( test_clock, 21);
//  mmc_write_event( modbus_id, "1234", SD_RFID_EVENT);
}
#if 1
#if defined ( RFID_SUPPLY)
void battery_manage_init( void)
{
  // P2.0接STAT1， P1.6接STAT2, P1.7接STAT3
  P1SEL &= ~(BV(7) & BV(6));
  P2SEL &= ~BV(0);
  
  P1DIR &= ~(BV(7) & BV(6));
  P2DIR &= ~BV(0);
}
#endif
#endif
/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter
  
#if !defined ( RFID_SUPPLY)
/*  afAddrType_t destAddr;
  uint16 len = 1;
  uint8 buf = 0; */
  uint8 modbusBuf[8] = { 0xff, 0x03, 0x00, 0x06, 0x00, 0x01, 0x71, 0xd5 };
#endif

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD)
              || (GenericApp_NwkState == DEV_ROUTER)
              || (GenericApp_NwkState == DEV_END_DEVICE) )
          {
#if !defined ( RFID_SUPPLY)
  #if !defined ( JUMP_MACHINE)
            osal_start_timerEx( GenericApp_TaskID, ASK_RSSI_REQ, 1000);
            osal_start_timerEx( GenericApp_TaskID, SEND_RSSI_ID, 6000);
  #endif
            osal_start_timerEx( GenericApp_TaskID, GET_MODBUS_ADDRESS, 3000);
//            osal_set_event( GenericApp_TaskID, SEND_RSSI_ID);
//            mmc_write_event( modbus_id, "1234", SD_RFID_EVENT);
#else
//            osal_set_event( GenericApp_TaskID, SEND_RSSI_EVT);      
#endif

#if defined ( CC2530_TIME_SYNC_ROUTER)|| defined ( CC2531_TIME_SYNC_COORD)
            // 开启时间同步事件
//            osal_set_event( GenericApp_TaskID, GENERICAPP_SEND_TIME_SYNC_EVT);
            
#endif
            
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
  // 用来作为延时使用，防止因为广播消息多次转播，多次同步
  if ( events & GENERICAPP_SEND_TIME_SYNC_EVT)
  {
//    GenericApp_SendTimeSyncReq();
    
/*    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_TIME_SYNC_EVT,
                        GENERICAPP_SEND_TIME_SYNC_TIMEOUT ); */
    time_sync_lock_flag = 1;
    
    return (events ^ GENERICAPP_SEND_TIME_SYNC_EVT);
  }
#endif

#if defined ( RFID_SUPPLY)
  if( events & SEND_RSSI_EVT)
  {
    afAddrType_t dstAddr;
    uint8 TransID;
    
    TransID = 0;
    dstAddr.addrMode = afAddrBroadcast;
    dstAddr.endPoint = AF_BROADCAST_ENDPOINT;
    dstAddr.addr.shortAddr = 0xffff;
    
    uint8 rssiMsg[7] = { 0xff, 0x03, 0x02, 0x00, 0xfe, 0x00, 0x00};
    AF_DataRequest( &dstAddr, &GenericApp_epDesc,
                                 MODBUS_PROTOCAL_CLUSTERID,
                                 7,
                                 rssiMsg,
                                 &TransID,
                                 AF_DISCV_ROUTE,
                                 1 );
    
    osal_start_timerEx( GenericApp_TaskID, SEND_RSSI_EVT, 15000);
    
    return ( events ^SEND_RSSI_EVT);
  }
  if( events & RFID_DELAY_FLAG)
  {
    rfid_lock_flag = 0;
    P0_6 = 0;
    //rfidInfo_t* rfidInfo_list = findRfidInfo( rfid_num);
   // if(  rfidInfo_list != NULL)
 /*   {
      afAddrType_t destAddr;
      uint8 TransID;
      
                    
      TransID = 0;
      destAddr.addrMode = afAddrBroadcast;
      destAddr.endPoint = AF_BROADCAST_ENDPOINT;
      destAddr.addr.shortAddr = 0xFFFF;
      
      uint8 rfid_sent[7] = {10,255,10, 0,0,0,0};
      
      rfid_sent[3] = BREAK_UINT32(rfid_num,0);
      rfid_sent[4] = BREAK_UINT32(rfid_num,1);
      rfid_sent[5] = BREAK_UINT32(rfid_num,2);
      rfid_sent[6] = BREAK_UINT32(rfid_num,3); 
              
      AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                   GATE_PROTOCAL_CLUSTERID,
                                   7,
                                   rfid_sent,
                                   &TransID,
                                   AF_DISCV_ROUTE,
                                   AF_DEFAULT_RADIUS ); 
    }*/
    return ( events ^ RFID_DELAY_FLAG);
  }
  
  if( events & BATTERY_MANAGEMENT)
  {
    if( BATTERY_PG)
    {
      if( (0 == BATTERY_STAT1 ) && ( 1 == BATTERY_STAT2))
      {
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
      }
      else if(( 1 == BATTERY_STAT1) && ( 0 == BATTERY_STAT2))
      {
        //HalLedBlink ( HAL_LED_1, 0, 50, 500 );
        HalLedSet( HAL_LED_3, HAL_LED_MODE_FLASH );
      }
    }
    else
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    }
    osal_start_timerEx( GenericApp_TaskID, BATTERY_MANAGEMENT, 10000);
    return ( events ^ BATTERY_MANAGEMENT);
  }
//  if( events & GENERIC_WRITE_MMC)
//  {
//    mmc_write_event( mmc_modbusId, dataBuf, mmc_event);
//    return ( events ^ BATTERY_MANAGEMENT);
//  }
#else  
  if( events & COMFIRM_RFID_NAME)
  {
    if( !confirmRfidName)
    {
      initCRC16();
      for( uint8 k = 0; k<29;k++)
      {
        CRC16_byte(rfidName[k]);
      }
      rfidName[27] = CRChi;
      rfidName[28] = CRClo;
      HalUARTWrite( 0, rfidName, 29);
      osal_start_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME, 500);
    }
    else
    {
      osal_stop_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME);
    }
    return ( events ^ COMFIRM_RFID_NAME);
  }
  if( events & GET_MODBUS_ADDRESS)
  {
    HalUARTWrite( 0, modbusBuf, 8);
    osal_start_timerEx( GenericApp_TaskID, GET_MODBUS_ADDRESS, 30000);
    
    
    return ( events ^ GET_MODBUS_ADDRESS);
  }
  
  if( events & ASK_RSSI_REQ)
  {
    afAddrType_t destAddr;
    uint8 TransID;
     
    TransID = 0;
    destAddr.addrMode = afAddrBroadcast;
    destAddr.endPoint = AF_BROADCAST_ENDPOINT;
    destAddr.addr.shortAddr = 0xFFFF;
      
    AF_DataRequest( &destAddr, &GenericApp_epDesc,
                                   ASK_RSSI_CLUSTER,
                                   1,
                                   &TransID,
                                   &TransID,
                                   AF_DISCV_ROUTE,
                                   1 );
    osal_start_timerEx( GenericApp_TaskID, ASK_RSSI_REQ, 3000);
    return ( events ^ ASK_RSSI_REQ);
  }
  
  if( events & SEND_RSSI_ID)
  {
    uint8 signalPack[13] = { 0xff, 0x10, 0x00, 0x12, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if( pRssiIndex != NULL)
    {
      if( pRssiIndex->delay_flag)
      {
        signalPack[8] = pRssiIndex->modbus_id;
        signalPack[10] = pRssiIndex->rssi;
        initCRC16();
        for( uint8 k = 0; k<13;k++)
        {
          CRC16_byte(signalPack[k]);
        }
        signalPack[11] = CRChi;
        signalPack[12] = CRClo;
        HalUARTWrite( 0, signalPack, 13);
        pRssiIndex->delay_flag = FALSE;
        if( pRssiIndex == NULL)
        {
          pRssiIndex = pSignalStren;
        }
        osal_start_timerEx( GenericApp_TaskID, SEND_RSSI_ID, 3000);
      }
      else
      {
        osal_set_event( GenericApp_TaskID, SEND_RSSI_ID);
      }
      pRssiIndex = pRssiIndex->next;
    }
    else
    {
      pRssiIndex = pSignalStren;
      osal_set_event( GenericApp_TaskID, SEND_RSSI_ID);
    }
    return ( events ^ SEND_RSSI_ID);
  }
  
#endif
  
#if defined ( JUMP_MACHINE) // 跳高器没收到RFID信息，就重发
  if ( events & CONFIRM_RFID_RSP)
  {
    if( (!confirm_rfid) && ( num_confirm_rfid < 3))
    {
      MT_BuildAndSendZToolResponse( 0x67, 0x14, 12, rfid_buf);
      osal_start_timerEx( GenericApp_TaskID, CONFIRM_RFID_RSP, 1000);
      num_confirm_rfid++;
    }
    else
    {
      num_confirm_rfid = 0;
      osal_stop_timerEx ( GenericApp_TaskID, CONFIRM_RFID_RSP);
    }
    confirm_rfid = FALSE;
    return ( events ^ CONFIRM_RFID_RSP);
  }
  if ( events & MSG_PROGRESS_EVT)
  {
    afAddrType_t timeAddr;
          uint8 TransID;
          
          TransID = 0;
          timeAddr.addrMode = Addr16Bit;
          timeAddr.endPoint = GENERICAPP_ENDPOINT;
          timeAddr.addr.shortAddr = 0x0;
          
          AF_DataRequest( &timeAddr, &GenericApp_epDesc,
                                       GATE_PROTOCAL_CLUSTERID,
                                       TxBuffer.size,
                                       TxBuffer.buf,
                                       &TransID,
                                       AF_DISCV_ROUTE,
                                       AF_DEFAULT_RADIUS );
          TxBuffer.size = 0;
    return ( events ^ MSG_PROGRESS_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
//uint8 time_reduce_flag = 0;
//uint32 first_time_record = 0;
//uint16 first_time_record_hi = 0;
//uint16 first_time_record_lo = 0;
//uint32 time_difference = 0;  // 时间差，测试用
//uint16 time_diff_hi = 0;
//uint16 time_diff_lo = 0;

static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
//  uint32 current_time;
  uint32 recv_time;
  uint32 timeSync[3];
  uint32 timeSync1;
  uint32 timeSync21;
  uint32 timeSync3;
  uint32 timeSync41;
  int32 time_correction;
#endif
#if defined ( GATE_PROTOCAL)
  uint8 *temp, *pTemp;
#endif
  
#if defined ( MODBUS_SUPPLY)
  uint8 ask_modbus_id[8] = {0xff, 0x03, 0x00, 0x06, 0x00, 0x01, 0x71, 0xd5}; 
//  bool modbusId_exist = FALSE;
#endif
#if defined ( RFID_SUPPLY)
  uint8 j;
  uint8 change_modbusId[8] = {0xff, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00};
//  bool store_ModbusId = TRUE;
  modbusId_t *modbusId_list = (modbusId_t *)NULL;
#endif
  
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;


#if defined ( RFID_SUPPLY) // USB dongle收到时间和卡消息
      // 时间消息
  case GATE_TIME_RECORD:
    modbusId_list = findGateByAddr( pkt->srcAddr.addr.shortAddr);
    if( modbusId_list != NULL)
    {
      uint16 temp_rec_lo = 0;
      uint16 temp_rec_hi = 0;
      // 存储到SD卡
//        osal_memcpy( dataBuf, &pkt->cmd.Data[3], 4);
      temp_rec_lo = BUILD_UINT16( pkt->cmd.Data[6], pkt->cmd.Data[7]);
      temp_rec_hi = BUILD_UINT16( pkt->cmd.Data[4], pkt->cmd.Data[5]);
      
      modbusId_list->distance = BUILD_UINT16( pkt->cmd.Data[1], pkt->cmd.Data[2]);
      modbusId_list->funcId = pkt->cmd.Data[3];
      modbusId_list->timeRecord = (uint32)( temp_rec_hi * 100000) + (uint32)(( temp_rec_lo * 43)/10);


      afAddrType_t timeAddr;
      uint8 TransID;
//      uint8 rfid_sent[7] = {11,255,11, 0,0,0,0};
      
//      rfid_sent[3] = LO_UINT16( temp_rec_hi);
//      rfid_sent[4] = HI_UINT16( temp_rec_hi);
//      rfid_sent[5] = LO_UINT16( temp_rec_lo);
//      rfid_sent[6] = HI_UINT16( temp_rec_lo);
      
      TransID = 0;
      timeAddr.addrMode = afAddrBroadcast;
      timeAddr.endPoint = AF_BROADCAST_ENDPOINT;
      timeAddr.addr.shortAddr = 0xFFFF;
      
      AF_DataRequest( &timeAddr, &GenericApp_epDesc,
                                   GATE_PROTOCAL_CLUSTERID,
                                   pkt->cmd.DataLength,
                            //       rfid_sent,
                                   pkt->cmd.Data,
                                   &TransID,
                                   AF_DISCV_ROUTE,
                                   AF_DEFAULT_RADIUS );
      
//        mmc_write_event( modbusId_list[i].modbusId, (uint8 *)&modbusId_list[i].timeRecord, SD_TIME_EVENT);  
        mmc_modbusId = modbusId_list->modbusId;
        mmc_event = SD_TIME_EVENT;
//        osal_memcpy( dataBuf, (uint8 *)&modbusId_list->timeRecord, 4);
        mmc_write_event( mmc_modbusId, (uint8 *)&modbusId_list->timeRecord, SD_TIME_EVENT);
   //     osal_start_timerEx( GenericApp_TaskID, GENERIC_WRITE_MMC, WRITE_MMC_TIMEOUT);
    }
    break;
    
#if defined ( JUMP_MACHINE_DONGLE)
  case JUMP_HEIGHT_RECORD:
      modbus_jump_height[0] =  pkt->cmd.Data[2];//= BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3]);//pkt->cmd.Data[2];
      modbus_jump_height[1] = pkt->cmd.Data[3];
      mmc_modbusId = 253;
      mmc_event = SD_JUMP_EVENT;
      mmc_write_event( mmc_modbusId, modbus_jump_height, SD_JUMP_EVENT);
//      osal_memcpy( dataBuf, pkt->cmd.Data, 4);
//      osal_start_timerEx( GenericApp_TaskID, GENERIC_WRITE_MMC, WRITE_MMC_TIMEOUT);
    break;
    
  case  JUMP_HEIGHT_SCROE:
      modbus_jump_record[0] =  pkt->cmd.Data[2];//= BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3]);//pkt->cmd.Data[2];
      modbus_jump_record[1] = pkt->cmd.Data[3];
      mmc_modbusId = 253;
      mmc_event = SD_JUMP_SCORE;
      mmc_write_event( mmc_modbusId, modbus_jump_record, SD_JUMP_SCORE);
      break;
#endif
    // RFID 消息
  case GATE_RFID_RECORD:
    if( pkt->cmd.DataLength == 13)
    {
//      for( i = 0; i < modbusId_index; i++)
      {
//        if( pkt->cmd.Data[1] == modbusId_list[i].modbusId)
        {
          /*rfid_name = BUILD_UINT32( pkt->cmd.Data[3],
                                   pkt->cmd.Data[4],
                                   pkt->cmd.Data[5],
                                   pkt->cmd.Data[6]);*/
        }
      }
    }
    else if( pkt->cmd.DataLength == 7)
    {
//      for( i = 0; i < modbusId_index; i++)
      {
//        if( pkt->cmd.Data[1] == modbusId_list[i].modbusId)
        {
//          modbusId_list[i].rfid_num = BUILD_UINT32( pkt->cmd.Data[3],jump_num++;
//          if(osal_nv_item_init( ZCD_NV_JUMP_NUM, sizeof(jump_num), &jump_num );
//          osal_nv_write( ZCD_NV_JUMP_NUM, 0, sizeof(jump_num), &jump_num );
          rfid_num = BUILD_UINT32( pkt->cmd.Data[3],
                                   pkt->cmd.Data[4],
                                   pkt->cmd.Data[5],
                                   pkt->cmd.Data[6]);
 //         osal_memcpy( dataBuf, &pkt->cmd.Data[3], 4);
          
 
    /*        
#ifdef RFID_SUPPLY
#define ZCD_NV_JUMP_NUM         0x00E1
#endif
uint8 jump_num = 0;
          osal_nv_read( ZCD_NV_JUMP_NUM, 0, sizeof(jump_num), &jump_num );
          for( uint8 i = 0; i< jump_num; i++)
          {
            osal_nv_read( (ZCD_NV_JUMP_NUM + 1 + (i*2)), 0, sizeof( uint32), &temp_rfid); 
            if( temp_rfid == rfid_num)
            {
              osal_nv_read( ( (ZCD_NV_JUMP_NUM + 2 + (i*2)), 0, sizeof( uint16), &temp_rfid)
            }
          } */
          
          mmc_modbusId = modbusId_list->modbusId;
          mmc_event = SD_RFID_EVENT;
  //        osal_start_timerEx( GenericApp_TaskID, GENERIC_WRITE_MMC, WRITE_MMC_TIMEOUT);
          mmc_write_event( modbusId_list->modbusId, pkt->cmd.Data, SD_RFID_EVENT);
          
          //time_reduce_flag = 0;
          
        }
      }
    }
    break;
    
#else     // GATE收到RFID或时间消息
    case GATE_TIME_RECORD:
      MT_BuildAndSendZToolResponse( 0x67, 0x14, pkt->cmd.DataLength, pkt->cmd.Data);
      break;
      
    case GATE_RFID_RECORD:
      //MT_BuildAndSendZToolResponse( 0x67, 0x14, pkt->cmd.DataLength, pkt->cmd.Data);
      
      #if !defined ( RFID_SUPPLY)
       if (pkt->cmd.Data[0]== 1 )
       {
         confirmRfidName = FALSE;
         osal_memcpy( &rfidName[7], "Green    ", 9);
         osal_start_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME, 500);
       }
       else if(pkt->cmd.Data[0]== 2 )
       {
         confirmRfidName = FALSE;
         osal_memcpy( &rfidName[7], "Orange   ", 9);
         osal_start_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME, 500);
       }
       else if(pkt->cmd.Data[0]== 3 )
       {
         confirmRfidName = FALSE;
         osal_memcpy( &rfidName[7], "Pink     ", 9);
         osal_start_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME, 500);
       }
       else
       {
         confirmRfidName = FALSE;
         osal_memcpy( &rfidName[7], "Error    ", 9);
         osal_start_timerEx( GenericApp_TaskID, COMFIRM_RFID_NAME, 500);
       }
      #endif
      break;
      
      
      signalStrength_t* pInSignal;
    case RSP_RSSI_CLUSTER:
      pInSignal = findSignalStrength( pkt->cmd.Data[0]);
      if( pInSignal != NULL)
      {
        if(pInSignal->rssi != pkt->rssi)
        {
          pInSignal->rssi = pkt->rssi;
        }
        pInSignal->delay_flag = TRUE;
      }
      else
      {
        register_signalStrength( pkt->cmd.Data[0], pkt->rssi);
        numSignalStren++;
      }
      break;
#endif
           
    case ASK_RSSI_CLUSTER:
      AF_DataRequest( &pkt->srcAddr, &GenericApp_epDesc,
                 RSP_RSSI_CLUSTER,
                 1,
                 &modbus_id,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
      break;
      
    
    
#if defined ( MODBUS_SUPPLY)
    case MODBUS_PROTOCAL_CLUSTERID:
#if defined ( RFID_SUPPLY)
      // 添加MODBUS ID表
      if( (pkt->cmd.Data[0] == 0xff) && (pkt->cmd.Data[1] == 0x03) && ( pkt->cmd.Data[2] == 0x02))
      {
        modbusId_list = findGateByAddr( pkt->srcAddr.addr.shortAddr);
        
        if( modbusId_list == NULL)  //不存在，添加
        {
          registerGate( pkt->srcAddr.addr.shortAddr, pkt->cmd.Data[4]);
          modbusId_index++;
        }
        else  // 存在，判断id是否冲突
        {
          modbusId_list = findGateById( pkt->cmd.Data[4]);
          if( modbusId_list != NULL)
          {
            if( modbusId_list->shortAddr != pkt->srcAddr.addr.shortAddr)
            {
              change_modbusId[5] = modbusId_list->modbusId - 1;
              
              initCRC16();
              for( j = 0; j < 6; j++)
              {
                CRC16_byte( change_modbusId[j]);
              }
              change_modbusId[6] = CRChi;
              change_modbusId[7] = CRClo;
              
              AF_DataRequest( &pkt->srcAddr, &GenericApp_epDesc,
                 MODBUS_PROTOCAL_CLUSTERID,
                 8,
                 change_modbusId,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
            }
          }
        }
      }
      
      else        // 其他消息发送到串口
//#else
 /*     if( (pkt->cmd.Data[0] == 0xff)&& (pkt->cmd.Data[1] == 0x03) && ( pkt->cmd.Data[2] == 0x02))
      {
        signalStrength_t* pInSignal = findSignalStrength( pkt->cmd.Data[4]);
        if( pInSignal != NULL)
        {
          if(pInSignal->rssi != pkt->rssi)
          {
            pInSignal->rssi = pkt->rssi;
          }
          pInSignal->delay_flag = TRUE;
        }
        else
        {
          register_signalStrength( pkt->cmd.Data[4], pkt->rssi);
          numSignalStren++;
        }
      } */
      // uint8 signalPack[8] = { 0xff, 0x06, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00};
      if( pkt->cmd.Data[0] == 0)
      {
        HalUARTWrite( 0, ask_modbus_id, 8);
      }
      else
      {
//#if !defined ( RFID_SUPPLY)
//        if( (pkt->cmd.Data[1] == 0x06) && (pkt->cmd.Data[3] == 0x06))
//          modbus_id = pkt->cmd.Data[5];
//#endif
  #if defined ( RSSI_DISPLAY)
          if( panId_send == TRUE)
          {
            modbus_rssi = pkt->rssi;
            panId_send = FALSE;
            calcCRC16( pkt->cmd.Data, pkt->cmd.DataLength);
          }
          else
            HalUARTWrite( 0, pkt->cmd.Data, pkt->cmd.DataLength);
  #else
       if(pkt->cmd.Data[0] != 0xff)
          HalUARTWrite( 0, pkt->cmd.Data, pkt->cmd.DataLength);
  #endif    //defined ( RSSI_DISPLAY)
      
      }
#endif   //defined ( RFID_SUPPLY)

#if !defined (RFID_SUPPLY)
      HalUARTWrite( 0, pkt->cmd.Data, pkt->cmd.DataLength);
#endif
      break;
#endif   // defined ( MODBUS_SUPPLY)
      
#if defined ( GATE_PROTOCAL)      
      // LJ Gate项目协议，收到命令处理
    case GATE_PROTOCAL_CLUSTERID:
      if( 0 == pkt->srcAddr.addr.shortAddr)
      {
        if( 2 == pkt->cmd.Data[1] || 3 == pkt->cmd.Data[1] || 4 == pkt->cmd.Data[1]) // 外部芯片上做时间同步时，把短地址加上
        {
          temp = osal_mem_alloc( pkt->cmd.DataLength + 2);
          if ( NULL != temp)
          {
            pTemp = temp;
            
            *pTemp++ = LO_UINT16( pkt->srcAddr.addr.shortAddr);
            *pTemp++ = HI_UINT16( pkt->srcAddr.addr.shortAddr);
            osal_memcpy( pTemp, pkt->cmd.Data, pkt->cmd.DataLength);
            
            MT_BuildAndSendZToolResponse( 0x67, 0x15, pkt->cmd.DataLength + 2, temp);
            
            osal_mem_free( temp);
          }
        }
        else
        {
          HAL_TOGGLE_LED1();          // 给跳高器测试用
          MT_BuildAndSendZToolResponse( 0x67, 0x14, pkt->cmd.DataLength, pkt->cmd.Data);
#if defined ( JUMP_MACHINE)
          if( pkt->cmd.DataLength != 4)
          {
            osal_memcpy( rfid_buf, pkt->cmd.Data, 12);
            osal_start_timerEx( GenericApp_TaskID, CONFIRM_RFID_RSP, 1000);
          }
#endif
        }
      }
      else
      {
        if( 68 == pkt->cmd.Data[1]    // 收到设备ID，加上自己的短地址发到串口
           || 2 == pkt->cmd.Data[1] || 3 == pkt->cmd.Data[1] || 4 == pkt->cmd.Data[1]) // 外部芯片上做时间同步时，把短地址加上
        {
          temp = osal_mem_alloc( pkt->cmd.DataLength + 2);
          if ( NULL != temp)
          {
            pTemp = temp;
            
            *pTemp++ = LO_UINT16( pkt->srcAddr.addr.shortAddr);
            *pTemp++ = HI_UINT16( pkt->srcAddr.addr.shortAddr);
            osal_memcpy( pTemp, pkt->cmd.Data, pkt->cmd.DataLength);
            
            MT_BuildAndSendZToolResponse( 0x67, 0x15, pkt->cmd.DataLength + 2, temp);
            
            osal_mem_free( temp);
          }
        }
        else
          MT_BuildAndSendZToolResponse( 0x67, 0x14, pkt->cmd.DataLength, pkt->cmd.Data);
      }
      // 打包发送到串口
      break;
#endif
      // LJ 时间同步测试
      /* 2012.9.28更改为任一节点发出请求，其他节点都可以同步时间, 该算法只能在节点能互相对传信息的时候才奏效*/
#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
    case TIME_SYNC_CLUSTERID:
      
      // 收到设置coord时间命令
      if(( 5 == pkt->cmd.DataLength) && ( SET_COORD_TIME == pkt->cmd.Data[0]))
      {
        recv_time = BUILD_UINT32( pkt->cmd.Data[1],
                                 pkt->cmd.Data[2],
                                 pkt->cmd.Data[3],
                                 pkt->cmd.Data[4]);
        osal_SetSystemClock( recv_time);
        
        // 发送广播通知全网开始同步
        GenericApp_SendBroadcastSyncReq ();
      }
      
      // 收到同步时间请求
      else if( (5 == pkt->cmd.DataLength) && ( TIME_SYNC_REQ == pkt->cmd.Data[0]))
      {
        timeSync[0] = BUILD_UINT32( pkt->cmd.Data[1], 
                                   pkt->cmd.Data[2], 
                                   pkt->cmd.Data[3], 
                                   pkt->cmd.Data[4]);   // 收到请求节点发送时的时间
        timeSync[1] = timeSync_recvMsgTime;             // 收到请求消息的时间
        timeSync[2] = osal_GetSystemClock();            // 发送包含三个时间的消息
        
        AF_DataRequest( &pkt->srcAddr, &GenericApp_epDesc,
                       TIME_SYNC_CLUSTERID,
                       3 * sizeof(uint32),
                       (byte *)&timeSync,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS );
      }             
                      
//    case TIME_SYNC_CLUSTERID:
      // 收到同步请求回复
//      if( (13 == pkt->cmd.DataLength) && ( TIME_SYNC_RSP == pkt->cmd.Data[0]))
      else if(12 == pkt->cmd.DataLength)
      {
        timeSync1 = BUILD_UINT32( pkt->cmd.Data[0], 
                                   pkt->cmd.Data[1], 
                                   pkt->cmd.Data[2], 
                                   pkt->cmd.Data[3]);
        timeSync21 = BUILD_UINT32( pkt->cmd.Data[4],
                                  pkt->cmd.Data[5], 
                                   pkt->cmd.Data[6], 
                                   pkt->cmd.Data[7]);
        timeSync3 = BUILD_UINT32( pkt->cmd.Data[8],
                                 pkt->cmd.Data[9], 
                                   pkt->cmd.Data[10], 
                                   pkt->cmd.Data[11]);
        timeSync41 = osal_GetSystemClock();
        
        time_correction = (( timeSync21 - timeSync1) - (timeSync41 - timeSync3))/2;
  
        osal_ChangeSystemClock( time_correction);
      }
      // 收到同步时间请求
      else if( ( 1 == pkt->cmd.DataLength) && ( TIME_SYNC_REQ == pkt->cmd.Data[0]))
      {
//        osal_set_event( GenericApp_TaskID, GENERICAPP_SEND_TIME_SYNC_EVT);
        if( time_sync_lock_flag)
        {
          GenericApp_SendTimeSyncReq( &pkt->srcAddr);
          // 同步一次后暂时关闭同步功能，2秒后再开启
          time_sync_lock_flag = 0;
          osal_start_timerEx( GenericApp_TaskID, GENERICAPP_SEND_TIME_SYNC_EVT, 2000);
        }
      }
      break;
#endif
  }
}

#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
/*********************************************************************
 * @fn      GenericApp_SendTimeSyncReq
 *
 * @brief   发送时间同步请求
 *
 * @param   none
 *
 * @return  none
 */
 // LJ router发起时间同步请求
static void GenericApp_SendTimeSyncReq( afAddrType_t *destAddr)
{
  uint8 timeAlive[5];
  uint32 current_time = osal_GetSystemClock();  // 获取当前时间，单位ms
  
/*  afAddrType_t destAddr;
  destAddr.addrMode = (afAddrMode_t)Addr16Bit;
  destAddr.addr.shortAddr = 0;                // 发送给coord
  destAddr.endPoint = GENERICAPP_ENDPOINT; */
  
  timeAlive[0] = TIME_SYNC_REQ;
  timeAlive[1] = BREAK_UINT32( current_time, 0);
  timeAlive[2] = BREAK_UINT32( current_time, 1);
  timeAlive[3] = BREAK_UINT32( current_time, 2);
  timeAlive[4] = BREAK_UINT32( current_time, 3);
  
  AF_DataRequest( destAddr, &GenericApp_epDesc,
                 TIME_SYNC_CLUSTERID,
                 5*sizeof( uint8),
                 timeAlive,
//                 (uint8 *)&current_time,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
}

static void GenericApp_SendBroadcastSyncReq( void)
{
  afAddrType_t destAddr;
  uint8 cmd = TIME_SYNC_REQ;
  
  destAddr.addrMode = afAddrBroadcast;
  destAddr.addr.shortAddr = 0xFFFF;                
  destAddr.endPoint = AF_BROADCAST_ENDPOINT;
  
  AF_DataRequest( &destAddr, &GenericApp_epDesc,
                 TIME_SYNC_CLUSTERID,
                 sizeof( uint8),
                 &cmd,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
}

#endif

#if defined ( RFID_SUPPLY)
static Status_t registerGate( uint16 shortAddr, uint8 modbusId)
{
  modbusId_t *pNewItem;
  modbusId_t *pLoop;
  
  pNewItem = osal_mem_alloc( sizeof( modbusId_t));
  if( pNewItem ==NULL)
    return ZMemError;
  
  pNewItem->next = (modbusId_t*) NULL;
  pNewItem->shortAddr = shortAddr;
  pNewItem->modbusId = modbusId;
  pNewItem->timeRecord = 0;
  pNewItem->distance = 0;
  pNewItem->funcId = 0;
  
  if( pModbusId == NULL)
  {
    pModbusId = pNewItem;
  }
  else
  {
    pLoop = pModbusId;
    while( pLoop->next!=NULL)
    {
      pLoop = pLoop->next;
    }
    pLoop->next = pNewItem;
  }
  
  return SUCCESS;
}

static modbusId_t* findGateByAddr( uint16 shortAddr)
{
  modbusId_t *pLoop = pModbusId;
  
  while( pLoop != NULL)
  {
    if( shortAddr == pLoop->shortAddr)
    {
      return pLoop;
    }
    pLoop = pLoop->next;
  }
  
  return (modbusId_t *)NULL;
}

static modbusId_t* findGateById( uint8 modbusId)
{
  modbusId_t *pLoop = pModbusId;
  
  while( pLoop!=NULL)
  {
    if( pLoop->modbusId == modbusId)
      return pLoop;
    pLoop = pLoop->next;
  }
  
  return (modbusId_t *)NULL;
}

Status_t registerRfidInfo( uint32 rfidNum)
{
  rfidInfo_t* pNewItem;
  rfidInfo_t* pLoop;
  
  pNewItem = osal_mem_alloc( sizeof( rfidInfo_t));
  
  if( pNewItem == NULL)
    return ZMemError;
  
  pNewItem->next = (rfidInfo_t*)NULL;
  pNewItem->rfidNum = rfidNum;
  pNewItem->rfidName = osal_mem_alloc( 10*sizeof(uint8));
  if(pNewItem->rfidName == NULL)
    return ZMemError;
  osal_memset( pNewItem->rfidName, 0, 10);
  
  if( pRfidInfo == NULL)
    pRfidInfo = pNewItem;
  else
  {
    pLoop = pRfidInfo;
    while( pLoop->next != NULL)
    {
      pLoop = pLoop->next;
    }
    pLoop->next = pNewItem;
  }
  return SUCCESS;
}

rfidInfo_t* findRfidInfo( uint32 rfidNum)
{
  rfidInfo_t* pLoop = pRfidInfo;
  
  while( pLoop !=NULL)
  {
    if( pLoop->rfidNum == rfidNum)
      return pLoop;
    pLoop = pLoop->next;
  }
  return (rfidInfo_t*)NULL;
}
#else
static Status_t register_signalStrength( uint8 modbus_id, int8 rssi)
{
  signalStrength_t *pNewItem;
  signalStrength_t *pLoop;
  
  pNewItem = osal_mem_alloc( sizeof( signalStrength_t));
  if( pNewItem == NULL)
  {
    return (ZMemError);
  }
  
  pNewItem->next = (signalStrength_t *)NULL;
  pNewItem->modbus_id = modbus_id;
  pNewItem->rssi = rssi;
  pNewItem->delay_flag = TRUE;
  
  if( pSignalStren == NULL)
  {
    pSignalStren = pNewItem;
  }
  else
  {
    pLoop = pSignalStren;
    while( pLoop->next != NULL)
    {
      pLoop = pLoop->next;
    }
    pLoop->next = pNewItem;
  }
  
  return SUCCESS;
}

//*********************************************
static signalStrength_t *findSignalStrength( uint8 modbus_id)
{
  signalStrength_t *pLoop = pSignalStren;
  
  while( pLoop != NULL)
  {
    if( modbus_id == pLoop->modbus_id)
    {
      return (pLoop);
    }
    pLoop = pLoop->next;
  }
  
  return ( (signalStrength_t*)NULL);
}

#endif
#if 0
#if defined ( GATE_PROTOCAL)
ZStatus_t GenericApp_SendGateProtocaltoCoord( uint16 len, uint8 *buf)
{
  uint8 stat;
  afAddrType_t destAddr;
  
  destAddr.addrMode = afAddr16Bit;
  destAddr.addr.shortAddr = 0;                
  destAddr.endPoint = 10;
  
  stat = AF_DataRequest( &destAddr, &GenericApp_epDesc,
                 GATE_PROTOCAL_CLUSTERID,
                 len,
                 buf,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
  return stat;
}

#endif
#endif
// 发送GATE协议命令
/*********************************************************************
 */






      /*
      time_reduce_flag++;

      if( time_reduce_flag == 1)
      {
        first_time_record = modbusId_list->timeRecord;
        first_time_record_hi = temp_rec_hi;
        first_time_record_lo = temp_rec_lo;
      }
      else if( time_reduce_flag == 2)
      {
        time_difference = modbusId_list->timeRecord - first_time_record;
        if( temp_rec_lo < first_time_record_lo )
        {
          time_diff_lo = ( temp_rec_lo + 1000000) - first_time_record_lo;
          time_diff_hi = temp_rec_hi - first_time_record_hi - 1;
        }
        else
        {
          time_diff_lo = temp_rec_lo - first_time_record_lo;
          time_diff_hi = temp_rec_hi - first_time_record_hi;
        }
        uint16 diff_hi = time_difference / 100000;
        uint16 diff_lo = ( time_difference % 100000)*10/43;
        //mmc_write_event( modbusId_list[i].modbusId, (uint8 *)&time_difference, SD_TIME_RESULT);
        
        afAddrType_t timeAddr;
        uint8 TransID;
        uint8 rfid_sent[7] = {11,255,11, 0,0,0,0};
        
        rfid_sent[3] = LO_UINT16( diff_hi);
        rfid_sent[4] = HI_UINT16( diff_hi);
        rfid_sent[5] = LO_UINT16( diff_lo);
        rfid_sent[6] = HI_UINT16( diff_lo);
        
        TransID = 0;
        timeAddr.addrMode = afAddrBroadcast;
        timeAddr.endPoint = AF_BROADCAST_ENDPOINT;
        timeAddr.addr.shortAddr = 0xFFFF;
        
        AF_DataRequest( &timeAddr, &GenericApp_epDesc,
                                     GATE_PROTOCAL_CLUSTERID,
                                     7,
                                     rfid_sent,
                                     &TransID,
                                     AF_DISCV_ROUTE,
                                     AF_DEFAULT_RADIUS );
        time_reduce_flag = 0;
      }*/
         /*
        for( i = 0; i < modbusId_index; i++)
        {
          if( pkt->cmd.Data[4] == modbusId_list[i].modbusId)     
          {
            if( pkt->srcAddr.addr.shortAddr != modbusId_list[i].shortAddr)
            {
              change_modbusId[0] = modbusId_list[i].modbusId;
              change_modbusId[5] = modbusId_list[i].modbusId - 1;
              
              initCRC16();
              for( j = 0; j < 6; j++)
              {
                CRC16_byte( change_modbusId[j]);
              }
              change_modbusId[6] = CRChi;
              change_modbusId[7] = CRClo;
              
              AF_DataRequest( &pkt->srcAddr, &GenericApp_epDesc,
                 MODBUS_PROTOCAL_CLUSTERID,
                 8,
                 change_modbusId,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );
            }
            store_ModbusId = FALSE;
          }
        }
        if( store_ModbusId == TRUE)
        {
          for( i = 0; i < modbusId_index; i++)
          {
            if( (modbusId_list[i].shortAddr == pkt->srcAddr.addr.shortAddr) && 
               modbusId_list[i].modbusId != pkt->cmd.Data[4])
            {
              modbusId_list[i].modbusId = pkt->cmd.Data[4];
              modbusId_exist = TRUE;
            }
          }
          if( modbusId_exist == FALSE)
          {
            modbusId_list[modbusId_index].modbusId = pkt->cmd.Data[4];
            modbusId_list[modbusId_index].shortAddr = pkt->srcAddr.addr.shortAddr;
            
            modbusId_index++;
          }
        }
      */     