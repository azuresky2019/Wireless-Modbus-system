/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "ZDApp.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

#include "AF.h"
#include "hal_uart.h"

#include "temcoAPI.h"
#include "OnBoard.h"
#include "modbus.h"
#include "OSAL_nv.h"
#include "hal_led.h"
#include "hal_adc.h"
/*********************************************************************
 * CONSTANTS
 */
#define TYPE_ASSERT_TIMEOUT     6

#define ACK_CHECK_TIMEOUT        10000     // 10000 means 10 secends
#define RSSI_REQ_TIMEOUT         10000

#define RSSI_NODE_LEAVE_NUM   2
#define ACK_TIMEOUT_NUM       2

#define WATER_SWITCH_CHN  HAL_ADC_CHANNEL_0
#define POWER_DETECT_CHN  HAL_ADC_CHANNEL_0

const int16 limit[10][2] = { { -400, 1500 }, { -400, 3020 },
                            { -400, 1200 }, { -400, 2480 },
                            { -400, 1200 }, { -400, 2480 },
                            { -400, 1200 }, { -400, 2480 },
                            { -500, 1100 }, { -580, 2300 }
                      };

const uint16 def_tab[5][17] = {
 /* 3k termistor YSI44005 -40 to 150 Deg.C or -40 to 302 Deg.F */
	{ 233*4,  211*4, 179*4, 141*4, 103*4, 71*4, 48*4, 32*4,
		21*4, 14*4, 10*4, 7*4, 5*4, 4*4, 3*4, 2*4, 1*4 },

 /* 10k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */
	{ 246*4, 238*4, 227*4, 211*4, 191*4, 167*4, 141*4, 115*4,
	 92*4, 72*4, 55*4, 42*4, 33*4, 25*4, 19*4, 15*4, 12*4 },

 /* 3k termistor GREYSTONE -40 to 120 Deg.C or -40 to 248 Deg.F */
	{ 233*4, 215*4, 190*4, 160*4, 127*4, 96*4, 70*4, 50*4,
		35*4, 25*4, 18*4, 13*4, 9*4, 7*4, 5*4, 4*4, 3*4 },

 /* 10k termistor KM -40 to 120 Deg.C or -40 to 248 Deg.F */
	{ 246*4, 238*4, 227*4, 211*4, 191*4, 167*4, 141*4, 115*4,
		92*4, 72*4, 55*4, 42*4, 33*4, 25*4, 19*4, 15*4, 12*4 },

 /* 3k termistor AK -40 to 150 Deg.C or -40 to 302 Deg.F */
	{ 246*4, 238*4, 227*4, 211*4, 191*4, 167*4, 141*4, 115*4,
		92*4, 72*4, 55*4, 42*4, 33*4, 25*4, 19*4, 15*4, 12*4 }
};

const int16 tab_int[10] = { 119, 214, 100, 180, 100, 180,100, 180, 100, 180 };

typedef enum { not_used_input, Y3K_40_150DegC, Y3K_40_300DegF, R10K_40_120DegC,
 R10K_40_250DegF, R3K_40_150DegC, R3K_40_300DegF, KM10K_40_120DegC,
 KM10K_40_250DegF, A10K_50_110DegC, A10K_60_200DegF, V0_5, I0_100Amps,
 I0_20ma, I0_20psi, N0_2_32counts, N0_3000FPM_0_10V, P0_100_0_5V,
 P0_100_4_20ma/*, P0_255p_min*/, V0_10_IN, table1, table2, table3, table4,
 table5, HI_spd_count = 100 } Analog_input_range_equate;

uint8 globalTempSensor = R10K_40_120DegC;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
const cId_t temcoClusterList[TEMCO_MAX_CLUSTERS] =
{
  TEMCO_CLUSTERID,
  
};

const SimpleDescriptionFormat_t temcoSimpleDesc =
{
  TEMCO_ENDPOINT,              //  int Endpoint;
  TEMCO_PROFID,                //  uint16 AppProfId[2];
  TEMCO_DEVICEID,              //  uint16 AppDeviceId[2];
  TEMCO_DEVICE_VERSION,        //  int   AppDevVer:4;
  TEMCO_FLAGS,                 //  int   AppFlags:4;
  TEMCO_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)temcoClusterList,  //  byte *pAppInClusterList;
  TEMCO_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)temcoClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t temco_epDesc;
bool ack_exist = TRUE;
uint8 ack_count = 0;
uint8 ask_modbus_id[8] = { 0xff, 0x03, 0x00, 0x06, 0x00, 0x02, 0x31, 0xd4};//0x01, 0x71, 0xd5};
uint8 tstat_id = 0;
uint8 product_id = 0;
uint8 type_assert = 0;
signalStrength_t *pSignalStren;
uint8 numSignalStren = 0;

uint16 ad_power;
/*********************************************************************
 * LOCAL VARIABLES
 */
byte temcoAPP_TaskID;
byte temcoApp_TransID;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void temcoApp_MessageMSGCB(afIncomingMSGPacket_t *pkt);
static Status_t register_signalStrength( uint8 modbus_id, int8 rssi);
static signalStrength_t *findSignalStrength( uint8 modbus_id);
static uint8 checkNodeAlive( void);
static void deleteSignalStrength( uint8 modbus_id);
static void sendAllSignalStren( void);
static void restart_to_other_type(void);
static uint16 get_input_value_by_range( int range, uint16 raw );
/*********************************************************************
 * @fn      temcoApp_Init
 */
void temcoApp_Init(uint8 task_id)
{
  temcoAPP_TaskID = task_id;
  
  temco_epDesc.endPoint = TEMCO_ENDPOINT;
  temco_epDesc.task_id = &temcoAPP_TaskID;
  temco_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&temcoSimpleDesc;
  temco_epDesc.latencyReq = noLatencyReqs;
  
  // register the endpoint description with the AF
  afRegister( &temco_epDesc);
#if 0
  if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
  {
     tstat_id = 254;        // NC's modbus id always is 9
  }
#endif
  P0SEL &= ~0x02;
  P0DIR &= ~0x02;
  
  P2DIR |= BV(1)|BV(2);
  P2_1 = 1;
  P2_2 = 1;
  
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
  
  osal_set_event( temcoAPP_TaskID, ASK_MODBUS_ID);
}


uint8 savedSwitchStatus = 0;

/*********************************************************************
 * @fn      temcoApp_ProcessEvent
 */
uint16 temcoApp_ProcessEvent(uint8 task_id, uint16 events)
{
  osal_event_hdr_t *pMsg;
  afIncomingMSGPacket_t *pMSGpkt;
  
  if( events & SYS_EVENT_MSG)
  {
    pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    while(pMsg)
    {
      switch( pMsg->event)
      {
        case AF_INCOMING_MSG_CMD:
          pMSGpkt = (afIncomingMSGPacket_t *)pMsg;
          temcoApp_MessageMSGCB(pMSGpkt);
          break;
          
        case ZDO_STATE_CHANGE:
          if (pMsg->status == DEV_END_DEVICE ||
              pMsg->status == DEV_ROUTER 
              ||pMsg->status == DEV_ZB_COORD )
          {
//            osal_set_event( temcoAPP_TaskID, ACK_CHECK);
//            HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );
//            HalLedBlink ( HAL_LED_1, 0, 50, 500 );
            osal_start_timerEx( temcoAPP_TaskID, DETECT_WATER_SWITCH, 1000);
            osal_start_timerEx( temcoAPP_TaskID, BREATHE_LOOP, 1000);
          }
          break;
          
        default:
          
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *) pMsg );
      
      // Next
      pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    }
    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if (events & ACK_CHECK)
  {
    uint8 ack_byte = 0;
    
    if( ack_count > ACK_TIMEOUT_NUM)
    {
      ack_count = 0;
      if( ack_exist == FALSE)
      {
        restore_factory_setting();
      }
    }
    if( ack_exist == TRUE)
    {
      zb_SendDataRequest( 0, ACK_CMD_CLUSTERID, 1, &ack_byte,
                           0, AF_ACK_REQUEST, 0);
      ack_exist = FALSE;
    }
    ack_count ++;
    osal_start_timerEx( temcoAPP_TaskID, ACK_CHECK, ACK_CHECK_TIMEOUT);   // Every minute check ack, if no receive, restart to join a new network
    return ( events ^ ACK_CHECK);
  }
  
  // Send the command to check TSTAT modbus id
  if( events & ASK_MODBUS_ID)
  {
    uint8 rssi_byte = 0;
    uint8 deleteId;

    if( type_assert >= TYPE_ASSERT_TIMEOUT)  // Decide which type to start up
    {
#if 0
      if( (product_id == 0) || (product_id == 100))   
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_ROUTER)
        {
          zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
          restart_to_other_type();
        }
      }
      else
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
        {
          zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
          restart_to_other_type();
        }
      }
#endif
      // To read register6 of TSTAT
      if( tstat_id != 0)
      {
        //if( zgDeviceLogicalType != ZG_DEVICETYPE_COORDINATOR)
        // Send a command asking other nodes' modbus id, the response message carry rssi 
        zb_SendDataRequest( 0xffff, RSSI_REQ_CLUSTERID, 1, &rssi_byte,
                             0, AF_ACK_REQUEST, 0);
        
        deleteId = checkNodeAlive();   // check if there are any nodes not alive
        if( deleteId != 255)
          deleteSignalStrength(deleteId);        // delete the dead id
      }
    }
    else
    {
      type_assert ++;
      HalUARTWrite ( 0, ask_modbus_id, 8 );
    }
    
    // if not received, send again X seconds later
    osal_start_timerEx( temcoAPP_TaskID, ASK_MODBUS_ID, RSSI_REQ_TIMEOUT);
    return ( events ^ ASK_MODBUS_ID);
  }
  uint8 tempWater[8];
  if( events & DETECT_WATER_SWITCH)
  {
    //P1_0 = ~P1_0;
    //MicroWait(1000);
    //P1_0 = ~P1_0;
    modbus_id = LO_UINT16(NLME_GetShortAddr())+2;
    ad_power = HalAdcRead( HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_12);
    tempWater[4] = LO_UINT16(ad_power);
    tempWater[5] = HI_UINT16(ad_power);
    ad_power = HalAdcRead( HAL_ADC_CHANNEL_5, HAL_ADC_RESOLUTION_12);
    tempWater[6] = LO_UINT16(ad_power);
    tempWater[7] = HI_UINT16(ad_power);
    ad_power = HalAdcRead( POWER_DETECT_CHN, HAL_ADC_RESOLUTION_12);
    //ad_power = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_14);
    tempWater[0] = LO_UINT16(ad_power);
    tempWater[1] = HI_UINT16(ad_power);
    tempWater[2] = ~P0_1;
    tempWater[3] = modbus_id;
    zb_SendDataRequest( 0, RSP_WATER_SWITCH, 8, tempWater, //暂时用23、24区分两个设备
                               0, AF_ACK_REQUEST, 0);
    
    osal_start_timerEx( temcoAPP_TaskID, DETECT_WATER_SWITCH, 8000);
    return ( events ^ DETECT_WATER_SWITCH);
  }
  
  if( events & BREATHE_LOOP)
  {
    P1_0 = ~P1_0;
    if( savedSwitchStatus != (~P0_1))
    {
      //osal_stop_timerEx( temcoAPP_TaskID, DETECT_WATER_SWITCH);
      ad_power = HalAdcRead( HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_12);
      ad_power = (uint32)ad_power*552/1000;
      ad_power = get_input_value_by_range(globalTempSensor, ad_power);
      tempWater[4] = LO_UINT16(ad_power);
      tempWater[5] = HI_UINT16(ad_power);
      ad_power = HalAdcRead( HAL_ADC_CHANNEL_5, HAL_ADC_RESOLUTION_12);
      ad_power = (uint32)ad_power*552/1000;
      ad_power = get_input_value_by_range(globalTempSensor, ad_power);
      tempWater[6] = LO_UINT16(ad_power);
      tempWater[7] = HI_UINT16(ad_power);
      ad_power = HalAdcRead( POWER_DETECT_CHN, HAL_ADC_RESOLUTION_12);
      //ad_power = HalAdcRead( HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_14);
      tempWater[0] = LO_UINT16(ad_power);
      tempWater[1] = HI_UINT16(ad_power);
      tempWater[2] = ~P0_1;
      tempWater[3] = modbus_id;
      zb_SendDataRequest( 0, RSP_WATER_SWITCH, 8, tempWater, //暂时用23、24区分两个设备
                                 0, AF_ACK_REQUEST, 0);
      savedSwitchStatus = ~P0_1;
    }
    osal_start_timerEx( temcoAPP_TaskID, BREATHE_LOOP, 800);
  }
  
  // Discard unknown events
  return 0;
}

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
  
  else if( destination == MAC_SHORT_ADDR_BROADCAST)
  {
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddrBroadcast;
  }
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = temco_epDesc.simpleDesc->EndPoint;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &temco_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  return status;
}


/*********************************************************************
 * @fn      temcoApp_MessageMSGCB
 */
static void temcoApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  switch(pkt->clusterId)
  {
    case TEMCO_CLUSTERID:
      if( (zgDeviceLogicalType == ZG_DEVICETYPE_ROUTER) || (zgDeviceLogicalType == ZG_DEVICETYPE_ENDDEVICE))  // Prepare to insert modbus message into TSTAT's
      {
        if( (pkt->cmd.Data[1] == MODBUS_SINGLE_READ) && (pkt->cmd.DataLength == 8))
        {
#if 0
          firstAddr = pkt->cmd.Data[3];
          uint8 length = pkt->cmd.Data[5];
          
          if( (firstAddr<21)&&((firstAddr+length)>21))
          {
            modbusDataLength = firstAddr+length-21;
            modbusStartAddr = 21;
          }
          else if( (firstAddr>=21) && (firstAddr <100))
          {
            modbusStartAddr = firstAddr;
            if(length <= (101-firstAddr))
              modbusDataLength = length;
            else
              modbusDataLength = 101-firstAddr;
          }
          else
          {
            modbusStartAddr = 0;
            modbusDataLength = 0;
          }
#endif
          modbus_uart_data_process(pkt->cmd.Data, pkt->cmd.DataLength);
        }
        else if( pkt->cmd.Data[1] == MODBUS_SINGLE_WRITE)
        {
          modbus_single_write(pkt->cmd.Data, pkt->cmd.DataLength);
        }
        else if( pkt->cmd.Data[1] == MODBUS_MULTI_WRITE)
        {
          modbus_multi_write(pkt->cmd.Data, pkt->cmd.DataLength);
        }
      }
      HalUARTWrite ( 0, pkt->cmd.Data, pkt->cmd.DataLength );
      break;
      
    case ACK_CMD_CLUSTERID:  // 7/16取消修改 此处改为定时发广播，收到任何节点消息，不论对方类型，都不需要重启
      if(zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
      {
        uint8 ack_byte = 1;
        zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, ACK_CMD_CLUSTERID, 1, &ack_byte, 
                               0, AF_ACK_REQUEST, 0);
      }
      else
      {
        if(pkt->cmd.Data[0] == 1)
          ack_exist = TRUE;
      }
      break;
      
    case RSSI_REQ_CLUSTERID:
      if( pkt->cmd.Data[0] == 0)
      {
        if( zgDeviceLogicalType == ZG_DEVICETYPE_COORDINATOR)
        {
          zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, RSSI_RSP_CLUSTERID, 1, &tstat_id, 
                                 0, AF_ACK_REQUEST, 0);
        }
        else
        {
          if( tstat_id != 0)
            zb_SendDataRequest( pkt->srcAddr.addr.shortAddr, RSSI_RSP_CLUSTERID, 1, &tstat_id, 
                                   0, AF_ACK_REQUEST, 0);
        }
      }
      break;
    case RSSI_RSP_CLUSTERID:
      if( pkt->cmd.Data[0] != 0)
      {
        
        signalStrength_t *pInSignal;
        pInSignal = findSignalStrength( pkt->cmd.Data[0]);
        if( pInSignal != NULL)
        {
          pInSignal->rssi = pkt->rssi;
          pInSignal->leaveTime = 0;
        }
        else
        {
          register_signalStrength( pkt->cmd.Data[0], pkt->rssi);
          numSignalStren++;
        }
      }
      break;
      
  case ASK_WATER_SWITCH:
    if( pkt->cmd.Data[0] == 0)
    {
      uint8 tempWater[4];
      tempWater[0] = LO_UINT16(ad_power);
      tempWater[1] = HI_UINT16(ad_power);
      tempWater[2] = P0_1;
      tempWater[3] = modbus_id;
      zb_SendDataRequest( 0, RSP_WATER_SWITCH, 4, tempWater, 
                                 0, AF_ACK_REQUEST, 0);
    }
    break;
    default:
      break;
  }
}

//*********************************************
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
  pNewItem->leaveTime = 0;
  
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
//*********************************************
static void deleteSignalStrength( uint8 modbus_id)
{
  signalStrength_t *pLoop, *pNext;
  
  pLoop = pSignalStren;
  pNext = pSignalStren->next;
  
  if( pLoop != NULL)
  {
    if(pLoop->modbus_id == modbus_id)
    {
      numSignalStren--;
      if(pNext != NULL)
        pSignalStren = pNext;
      else
        pSignalStren = NULL;
      osal_mem_free( pLoop);
    }
    else
    {
      while( pNext != NULL)
      {
        if(pNext->modbus_id == modbus_id)
        {
          numSignalStren--;
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
static uint8 checkNodeAlive( void)
{
  signalStrength_t *pLoop = pSignalStren;
  uint8 modbus_id = 255;
  
  while(pLoop != NULL)
  {
    pLoop->leaveTime++;
    if(pLoop->leaveTime >= RSSI_NODE_LEAVE_NUM)
    {
      modbus_id = pSignalStren->modbus_id;
      return modbus_id;
    }
    pLoop = pLoop->next;
  }
  return modbus_id;
}
//*********************************************
static void sendAllSignalStren( void)
{
  signalStrength_t *pLoop = pSignalStren;
  while( pLoop != NULL)
  {
    send_char_Uart( pLoop->modbus_id, 0);
    send_char_Uart( pLoop->rssi, 0);
    pLoop = pLoop->next;
  }
}
//*********************************************
static void restart_to_other_type(void)
{
  NLME_InitNV();
  NLME_SetDefaultNV();
  osal_nv_write( ZCD_NV_LOGICAL_TYPE, 0, sizeof(zgDeviceLogicalType), &zgDeviceLogicalType);
  SystemReset();
}

#define MIDDLE_RANGE     8
#define NO_TABLE_RANGES 16
static uint16 get_input_value_by_range( int range, uint16 raw )
{
	int index;
	long val;
	int work_var;
	int ran_in;
	int delta = MIDDLE_RANGE;
	uint16 *def_tbl;
	uint8 end = 0;
	range--;
	ran_in = range;
	range >>= 1;
	def_tbl = ( uint16 * )&def_tab[range];

	if( raw <= def_tbl[NO_TABLE_RANGES] )
		return limit[ran_in][1];
	if( raw >= def_tbl[0] )
		return limit[ran_in][0];
	index = MIDDLE_RANGE;

	while( !end )
	{
		if( ( raw >= def_tbl[index] ) && ( raw <= def_tbl[index-1] ) )
		{
			index--;
			delta = def_tbl[index] - def_tbl[index+1];
			if( delta )
			{
				work_var = (int)( ( def_tbl[index] - raw ) * 100 );
				work_var /= delta;
				work_var += ( index * 100 );
				val = tab_int[ran_in];
				val *= work_var;
				val /= 100;
				val += limit[ran_in][0];
			}
			return val;
		}
		else
		{
			if( !delta )
				end = 1;
			delta /= 2;
			if( raw < def_tbl[index] )
				index += delta;
			else
				index -= delta;
			if( index <= 0 )
				return limit[ran_in][0];
			if( index >= NO_TABLE_RANGES )
				return limit[ran_in][1];
		}
	}
        return 0;
}
