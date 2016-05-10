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
#include "MT_UART.h"
#include "OSAL_Nv.h"

#if defined ( MODBUS_SUPPLY)
  #include "modbus.h"
#endif
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

#define RSSI_NODE_LEAVE_NUM   2
boatMonitor_t* pBoatMonitor;
uint8 numMonitor;
uint8 allocModbusId = 23;
uint8 change_modbusId[8] = {0xff, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00};
/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
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

Tx_data_t TxBuffer;

#if defined (START_ROUTER)
  uint8 reg_len;
  uint8 len_check_flag = 0;
  #ifdef PANID_DISPLAY
    bool panId_send = FALSE;
    uint8 panId_send_hi, panId_send_lo;
  #endif
  bool ack_exist = TRUE;
#else
  uint16 update_addr;
  
#endif
  
uint16 ad_power=0;
uint16 detectPin1 = 0;
uint16 detectPin2 = 0;
uint8 water_switch = 0;
uint8 water_id = 0;
int8 water_rssi = 0;
//uint8 leaveTimeCounter = 0;

uint8 modbus_set_baudrate = 0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void response_scan_cmd(void);
/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

afStatus_t zb_SendDataRequest( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius );

static boatMonitor_t* findBoatMonitorById(uint8 modbusId);
static boatMonitor_t* findBoatMonitorByAddr(uint16 shortAddr);
static Status_t registerBoatMonitor(uint8 modbusId, uint16 shortAddr, uint16 adPower,
                                    uint16 detectPin1, uint16 detectPin2, uint8 waterSwitch, int8 rssi);
static void deleteBoatMonitor( uint16 shortAddr);
static uint16 checkNodeAlive( void);
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
  pBoatMonitor = (boatMonitor_t*)NULL;
  
  MT_UartInit();
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
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  if(SUCCESS != osal_nv_read(ZCD_NV_SERIAL_NUM,0, sizeof(ttt), ttt))
  {
    osal_nv_item_init(ZCD_NV_SERIAL_NUM,sizeof(ttt), ttt);
    osal_nv_write(ZCD_NV_SERIAL_NUM, 0,sizeof(ttt), ttt);
  }
  if(SUCCESS != osal_nv_read(ZCD_NV_MODBUS_ID,0, sizeof(modbus_id), &modbus_id))
  {
    osal_nv_item_init(ZCD_NV_MODBUS_ID, sizeof(modbus_id), &modbus_id);
    osal_nv_write(ZCD_NV_MODBUS_ID, 0,sizeof(modbus_id), &modbus_id);
  }
  
  P0SEL &= ~BV(6);
  P0DIR |= BV(6);
  P0_6 = 0;
  
  P1SEL &= ~BV(0);
  P1DIR |= BV(0);
  P1_0 = 0;
  
  P2DIR |= BV(1)|BV(2);
  P2_1 = 1;
  P2_2 = 1;
  
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if !defined START_ROUTER
  HAL_TURN_ON_LED1();
  osal_set_event(GenericApp_TaskID, TX_MSG_EVENT);    // 用轮询的方法，发送串口消息
#endif
}

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
    uint16 tempAddr;
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
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
#if defined (START_ROUTER)
            osal_set_event( GenericApp_TaskID, ACK_CHECK);
#endif
//            osal_set_event( GenericApp_TaskID,DETECT_WATER_SWITCH);
            osal_start_timerEx(GenericApp_TaskID, LEAVE_TIME_CHECK, 3000);
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
  
#if defined ( START_ROUTER)
  if ( events & ACK_CHECK)
  {
    uint8 ack_byte = 0;
    if( ack_exist == TRUE)
    {
      zb_SendDataRequest( 0, ACK_CMD_ID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);
    }
    else
    {
      restore_factory_setting();
    }
    ack_exist = FALSE;
    osal_start_timerEx( GenericApp_TaskID, ACK_CHECK, 10000);   // Every minute check ack, if no receive, restart to join a new network
    return ( events ^ ACK_CHECK);
  }
#endif 
  
/*  if( events & DETECT_WATER_SWITCH)
  {
    uint8 tempWaterSwitch=0;
//    zb_SendDataRequest( 0xFFFF, ASK_WATER_SWITCH, 1, &tempWaterSwitch, 
//                               0, AF_ACK_REQUEST, 0);
    
    if(leaveTimeCounter!= 0)
    {
      ad_power = 754;
      water_id = 0;
      water_rssi = 0;
    }
    leaveTimeCounter=1;
    osal_start_timerEx( GenericApp_TaskID, DETECT_WATER_SWITCH, 3000);
    return ( events ^ ASK_WATER_SWITCH);
  }*/
  
  
  if( events & TX_MSG_EVENT)
  {
#if defined START_ROUTER
    uint16 coord_addr = 0;
    if( TxBuffer.buf[0] == 0xff)        // 用T3000扫描时要回复
    {
      // When the first byte is 0xff, it's a scan command of T3000 software to scan all the nodes.
      zb_SendDataRequest( coord_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                               0, AF_ACK_REQUEST, 0);
      TxBuffer.size = 0;
    }
    else
    {
#ifdef PANID_DISPLAY
      if( panId_send == TRUE)
      {
        TxBuffer.buf[panId_send_hi] = HI_UINT16(_NIB.nwkPanId);
        TxBuffer.buf[panId_send_lo] = LO_UINT16(_NIB.nwkPanId);
        
        panId_send = FALSE;
      }
#endif
      zb_SendDataRequest( coord_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                               0, AF_ACK_REQUEST, 0);
      TxBuffer.size = 0;
    }
#else
    if( TxBuffer.size > 5)
    {
      // the length of scan message is 5
     if( (TxBuffer.buf[0] == 0xff) && ( TxBuffer.buf[1] == 0x19))
     {
       response_scan_cmd();
       // send a broadcast message
       zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, 6, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
       TxBuffer.size = 0;
     }
     else if( (TxBuffer.buf[1] == 3) || ( TxBuffer.buf[1] == 6) || (TxBuffer.buf[1] == 16))
     {
       // the second byte of read command is 0x03, and write command is 0x06
       if( TxBuffer.size > 7)
       {
#if 1//defined ( USB_DONGLE)
          if( (TxBuffer.buf[0] == 0xff) || ( TxBuffer.buf[0] == modbus_id))
            modbus_uart_data_process( TxBuffer.buf, TxBuffer.size);
          else
#endif
          {
             // the length of read command is 8
             // send a broadcast message
             zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                                 0, AF_ACK_REQUEST, 0);
          }
          TxBuffer.size = 0;
       }
     }
     else if( (TxBuffer.buf[1] == 0x10) && ( TxBuffer.buf[4] == 0x00) && ( TxBuffer.buf[5] == 0x80))
     {
       if( TxBuffer.size > 5)
       {
         if( TxBuffer.size >= (TxBuffer.buf[5] + 9))
         {
           zb_SendDataRequest( update_addr, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
           TxBuffer.size = 0;
         }
       }
     }
     else
     {
       zb_SendDataRequest( 0xFFFF, GENERICAPP_CLUSTERID, TxBuffer.size, TxBuffer.buf, 
                             0, AF_ACK_REQUEST, 0);
       TxBuffer.size = 0;
     }
    }
   // every 25 ms to check if there are uart msg
   osal_start_timerEx( GenericApp_TaskID, TX_MSG_EVENT, 25);    // LJ 每50毫秒轮询一次检测BUFFER里是否有消息要发出
#endif
   return ( events ^ TX_MSG_EVENT);
  }
  
  if( events & LEAVE_TIME_CHECK)
  {
    //leaveTimeCounter = 0;
    tempAddr = checkNodeAlive();
    if( tempAddr != 0xffff)
      deleteBoatMonitor(tempAddr);
    
    osal_start_timerEx(GenericApp_TaskID, LEAVE_TIME_CHECK, 2000);
    return ( events ^ LEAVE_TIME_CHECK);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn          zb_SendDataRequest
 *
 * @brief       The function initiates transmission of data
 *              to a peer device
 * 
 * @return      none
 */
afStatus_t zb_SendDataRequest( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
{
  afStatus_t status;
  afAddrType_t dstAddr;

  txOptions |= AF_DISCV_ROUTE;

  // Set the destination address
  if (destination == INVALID_NODE_ADDR)
  {
    // Binding
    dstAddr.addrMode = afAddrNotPresent;
  }
  /*
  else if( destination == MAC_SHORT_ADDR_BROADCAST)
  {
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddrBroadcast;
  }*/
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = GenericApp_epDesc.simpleDesc->EndPoint;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &GenericApp_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  return status;
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

static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  
      P2_2 = ~P2_2;
#if ! defined (START_ROUTER)
  uint8 ack_byte = 1;
#endif
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:

    update_addr = pkt->srcAddr.addr.shortAddr;
      // send the received msg to uart
    
  #if defined ( USB_DONGLE)
    HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
  #else
    if(pkt->cmd.Data[0] != 0)
//      send_str_Uart( pkt->cmd.Data, pkt->cmd.DataLength, 0);
    {
      
      P1_0 = 1;
      //MicroWait(5);
      send_str_Uart( pkt->cmd.Data, pkt->cmd.DataLength, 0);
      //HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
      P1_0 = 0;
      //MicroWait(5);
    }
  #endif

    break;
    
    case ACK_CMD_ID:
      if( pkt->cmd.Data[0] == 0)
        zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, ACK_CMD_ID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);


      break;
      
    case RSP_WATER_SWITCH:
      ad_power = BUILD_UINT16( pkt->cmd.Data[0],pkt->cmd.Data[1]);
      water_switch = pkt->cmd.Data[2];
      water_id = pkt->cmd.Data[3];
      water_rssi = pkt->rssi;
      detectPin1 = BUILD_UINT16( pkt->cmd.Data[4],pkt->cmd.Data[5]);
      detectPin2 = BUILD_UINT16( pkt->cmd.Data[6],pkt->cmd.Data[7]);
      //leaveTimeCounter = 1;
      boatMonitor_t* pTemp;
      pTemp = findBoatMonitorByAddr( pkt->srcAddr.addr.shortAddr);
      if( pTemp != NULL)
      {
        pTemp->modbus_id = water_id;
        pTemp->ad_power = ad_power;
        pTemp->detectPin1 = detectPin1;
        pTemp->detectPin2 = detectPin2;
        pTemp->water_switch = water_switch;
        pTemp->rssi = water_rssi;
        pTemp->leaveTimeout = 0;
      }
      else
      {
        /*water_id = allocModbusId;
        change_modbusId[5] = water_id;
        AF_DataRequest( &pkt->srcAddr, &GenericApp_epDesc,
                 GENERICAPP_CLUSTERID,
                 8,
                 change_modbusId,
                 &GenericApp_TransID,
                 AF_DISCV_ROUTE,
                 AF_DEFAULT_RADIUS );*/
        registerBoatMonitor(water_id, pkt->srcAddr.addr.shortAddr, ad_power, detectPin1,
                            detectPin2, water_switch, water_rssi);
        numMonitor ++;
        //allocModbusId++;
      }
      
      //osal_start_timerEx(GenericApp_TaskID, LEAVE_TIME_CHECK, 10000);
      //MicroWait(400);
      break;
   }
}

static Status_t registerBoatMonitor(uint8 modbusId, uint16 shortAddr, uint16 adPower,
                                    uint16 detectPin1, uint16 detectPin2, uint8 waterSwitch, int8 rssi)
{
  boatMonitor_t *pNewItem;
  boatMonitor_t *pLoop;
  
  pNewItem = osal_mem_alloc( sizeof(boatMonitor_t));
  if(pNewItem == NULL)
    return ZMemError;
  
  pNewItem->next = (boatMonitor_t*)NULL;
  pNewItem->modbus_id = modbusId;
  pNewItem->shortAddr = shortAddr;
  pNewItem->ad_power = adPower;
  pNewItem->detectPin1 = detectPin1;
  pNewItem->detectPin2 = detectPin2;
  pNewItem->water_switch = waterSwitch;
  pNewItem->rssi = rssi;
  pNewItem->leaveTimeout = 0;
  
  if (pBoatMonitor == NULL)
    pBoatMonitor = pNewItem;
  else
  {
    pLoop = pBoatMonitor;
    while(pLoop->next != NULL)
    {
      pLoop = pLoop->next;
    }
    pLoop->next = pNewItem;
  }
  
  return SUCCESS;
}

static boatMonitor_t* findBoatMonitorByAddr(uint16 shortAddr)
{
  boatMonitor_t* pLoop = pBoatMonitor;
  
  while(pLoop != NULL)
  {
    if( shortAddr == pLoop->shortAddr)
    {
      return pLoop;
    }
    pLoop = pLoop->next;
  }
  
  return (boatMonitor_t *)NULL;
}

static boatMonitor_t* findBoatMonitorById(uint8 modbusId)
{
  boatMonitor_t* pLoop = pBoatMonitor;
  
  while(pLoop != NULL)
  {
    if( modbusId == pLoop->modbus_id)
    {
      return pLoop;
    }
    pLoop = pLoop->next;
  }
  return (boatMonitor_t*)NULL;
}

static void deleteBoatMonitor( uint16 shortAddr)
{
  boatMonitor_t *pLoop, *pNext;
  
  pLoop = pBoatMonitor;
  pNext = pBoatMonitor->next;
  
  if( pLoop != NULL)
  {
    if(pLoop->shortAddr == shortAddr)
    {
      numMonitor--;
      if(pNext != NULL)
        pBoatMonitor = pNext;
      else
        pBoatMonitor = NULL;
      osal_mem_free( pLoop);
    }
    else
    {
      while( pNext != NULL)
      {
        if(pNext->shortAddr == shortAddr)
        {
          numMonitor--;
          pLoop->next = pNext->next;
          osal_mem_free(pNext);
          return;
        }
        pLoop = pNext;
        pNext = pLoop->next;
      }
    }
  }
}

//*********************************************
static uint16 checkNodeAlive( void)
{
  boatMonitor_t *pLoop = pBoatMonitor;
  uint16 shortAddr = 0xffff;
  
  while(pLoop != NULL)
  {
    pLoop->leaveTimeout++;
    if(pLoop->leaveTimeout >= RSSI_NODE_LEAVE_NUM)
    {
      shortAddr = pBoatMonitor->shortAddr;
      return shortAddr;
    }
    pLoop = pLoop->next;
  }
  return shortAddr;
}
/*********************************************************************
 */
