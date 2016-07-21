/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2012-02-12 15:58:41 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29216 $

  Description:    This file contains the Generic Application definitions.


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
**************************************************************************************************/

#ifndef GENERICAPP_H
#define GENERICAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define GENERICAPP_ENDPOINT           10

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#if defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD) && defined ( GATE_PROTOCAL)
#define GENERICAPP_MAX_CLUSTERS       4
#elif defined ( CC2530_TIME_SYNC_ROUTER) || defined ( CC2531_TIME_SYNC_COORD)
#define GENERICAPP_MAX_CLUSTERS       2
#elif defined ( GATE_PROTOCAL) && defined ( MODBUS_SUPPLY)
#define GENERICAPP_MAX_CLUSTERS       3
#else
#define GENERICAPP_MAX_CLUSTERS       1
#endif
#define GENERICAPP_CLUSTERID          1

// Send Message Timeout
#define GENERICAPP_SEND_MSG_TIMEOUT   5000     // Every 5 seconds
  
#if defined ( CC2530_TIME_SYNC_ROUTER) || defined (CC2531_TIME_SYNC_COORD)
// 时间同步命令
  #define SET_COORD_TIME                      0
  #define TIME_SYNC_REQ                       1
  #define TIME_SYNC_RSP                       2
    
  #define TIME_SYNC_CLUSTERID                 0x0002
  #define GENERICAPP_SEND_TIME_SYNC_EVT       0x0010
  #define GENERICAPP_SEND_TIME_SYNC_TIMEOUT   30000
#endif
 
#if defined ( GATE_PROTOCAL)
  #define GATE_PROTOCAL_CLUSTERID               0x0003
#if defined ( JUMP_MACHINE)
  #define CONFIRM_RFID_RSP                0x0020
  #define MSG_PROGRESS_EVT                0x0040
#endif
  
#define MSG_ACK                   0
#define MSG_SET_LAMP              1
#define MSG_SLEEP                 2
#define MSG_RQ_STATUS             3
#define MSG_UPGRADE               4
#define MSG_WRITE_E2              5
#define MSG_RQ_READ_E2            6
#define MSG_RESET                 7
#define MSG_CONFIG                8
#define MSG_CHRONOS               9
#define MSG_LEAVE_NETWORK         10
#define MSG_ATHLETE_NAME          13
#endif
  
#if defined ( MODBUS_SUPPLY)
  #define MODBUS_PROTOCAL_CLUSTERID     0x0004
#endif
  
#define BATTERY_STAT1                 P2_0
#define BATTERY_STAT2                 P1_6
#define BATTERY_PG                    P1_7
  
// Application Events (OSAL) - These are bit weighted definitions.
#define GENERICAPP_SEND_MSG_EVT       0x0001

#if defined ( RFID_SUPPLY)  
  #define RFID_DELAY_FLAG               0x0020
  #define BATTERY_MANAGEMENT            0x0040
  #define GATE_JUMP_RECORD              0xc300
  //#define GENERIC_WRITE_MMC             0x0080
  #define SEND_RSSI_EVT                 0x0400
  //#define WRITE_MMC_TIMEOUT             1000
#else
  #define GET_MODBUS_ADDRESS            0x0080
  
  #define SEND_RSSI_ID                  0x0200
  #define COMFIRM_RFID_NAME             0x0400
  #define ASK_RSSI_REQ                  0x0800
#endif

#define JUMP_HEIGHT_SCROE             0xc400  
#define JUMP_HEIGHT_RECORD            0xc300
#define GATE_TIME_RECORD              0xc200
#define GATE_RFID_RECORD              0xc100
#define ASK_RSSI_CLUSTER              0xd100
#define RSP_RSSI_CLUSTER              0xd200
  
/*********************************************************************
 * GLOBAL VARIABLES
 */
  
#if defined ( RFID_SUPPLY)
//#define MAX_GATE_NUM    10
typedef struct modbusId
{
  struct modbusId *next;
  uint16 shortAddr;
  uint8 modbusId;
  uint32 timeRecord;
  uint8 distance;
  uint8 funcId;
} modbusId_t;

typedef struct rfidInfo
{
  struct rfidInfo *next;
  uint32 rfidNum;
  uint8 *rfidName;
}rfidInfo_t;
#if 0
  uint32 rfid_num;
 // uint32 rfid_name;
  uint16 boardHeight;
  uint8 userId_stat;
  uint16 challengeHeight;
  uint8 modeSelection;
#endif

  extern uint8 mmc_modbusId;
  extern uint8 mmc_event;
  extern uint8 modbusId_index;
  extern uint8 rfidInfo_index;
  //extern modbusId_t modbusId_list[MAX_GATE_NUM];
  extern modbusId_t *pModbusId;
  extern rfidInfo_t* pRfidInfo;
  extern uint32 rfid_num;
  // extern uint32 rfid_name;
  extern uint16 time_diff_hi ;
  extern uint16 time_diff_lo ;
  
  extern uint32 time_difference;
  
  extern Status_t registerRfidInfo( uint32 rfidNum);
  extern rfidInfo_t* findRfidInfo( uint32 rfidNum);

#else
// signal strength list
  typedef struct signalStrength
  {
    struct signalStrength *next;
    uint8 modbus_id;
    int8 rssi;
    bool delay_flag;
  } signalStrength_t;
  extern bool confirmRfidName;
#endif


#if defined ( JUMP_MACHINE)
typedef struct
{
  uint8 buf[512];
  uint16 size;
} Tx_data_t;

extern Tx_data_t TxBuffer;
  extern bool confirm_rfid;
#endif
/*********************************************************************
 * MACROS
 */
extern byte GenericApp_TaskID;
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
