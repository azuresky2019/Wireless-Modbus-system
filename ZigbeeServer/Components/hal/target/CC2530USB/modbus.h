#if defined ( MODBUS_SUPPLY)

#ifndef MODBUS_H
#define MODBUS_H
/*********************************************************************
* INCLUDE
 */
#include "hal_types.h"

/*********************************************************************
* CONTANTS
 */
#define CRC_NO  0
#define CRC_YES 1

#define MODBUS_SINGLE_READ      0x03
#define MODBUS_SINGLE_WRITE     0x06
#define MODBUS_MULTI_WRITE      0x10

/*********************************************************************
* MACROS
 */
#define MODBUS_GATE_DISTANCE_1  (MODBUS_GATE_ADDRESS_1+modbusId_index)//= 61,
#define MODBUS_GATE_TIME_1_SEC  (MODBUS_GATE_DISTANCE_1+modbusId_index)
#define MODBUS_GATE_TIME_1_MICROSEC  (MODBUS_GATE_TIME_1_SEC+modbusId_index)
#define MODBUS_GATE_FUNC_1_ID  (MODBUS_GATE_TIME_1_MICROSEC+modbusId_index)
#define MODBUS_RFID_NUM  (MODBUS_GATE_FUNC_1_ID+modbusId_index)
#define MODBUS_RFID_ID_START  (MODBUS_RFID_NUM+1)
#define MODBUS_RFID_ID_END  (MODBUS_RFID_ID_START+(rfidInfo_index*4))
#define MODBUS_RFID_NAME   (MODBUS_RFID_ID_END+1)
#define MODBUS_RFID_END   (MODBUS_RFID_NAME + (rfidInfo_index * 10))
/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 CRClo, CRChi;
extern uint8 modbus_id;
extern uint8 modbus_jump_height[2];
extern uint16 modbus_jump_record ;
extern bool restroe_factory_setting;

#if defined ( RSSI_DISPLAY)
  extern uint8 rssi_send_hi, rssi_send_lo;
  extern bool panId_send ;
  extern int8 modbus_rssi;
  extern void calcCRC16( uint8* data, uint8 len);
#endif
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
extern void modbus_uart_data_process( uint8 *data_buffer, uint8 len);
extern void initCRC16( void);
extern void CRC16_byte( uint8 ch);
/*********************************************************************
 * LOCAL VARIABLES
 */
enum {
  MODBUS_SERIALNUMBER_LOWORD  = 0,             
  MODBUS_SERIALNUMBER_HIWORD  = 2,
  MODBUS_FIRMWARE_VERSION_NUMBER_LO = 4,
  MODBUS_FIRMWARE_VERSION_NUMBER_HI,
  MODBUS_ADDRESS                = 6,
  MODBUS_HARDWARE_REV,
  
  MODBUS_SYS_HOUR,
  MODBUS_SYS_MINUTES,
  MODBUS_SYS_SECONDS,
  MODBUS_SYS_MONTH,
  MODBUS_SYS_DAY,
  MODBUS_SYS_YEAR,
  
  MODBUS_SYS_RESTORE = 14,
  
  MODBUS_PANID = 21,
  
  MODBUS_DEVICE_ID  = 22,
  MODBUS_GROUP_ID,
  MODBUS_JOB_ID,
  MODBUS_USER_ID,
  
  MODBUS_CHARGING_STAT,
  MODBUS_LEFT_QUANTITY,
  MODBUS_LEFT_HOURS,
#if defined ( JUMP_MACHINE_DONGLE)
  MODBUS_JUMP_HEIGHT,
  MODBUS_JUMP_RECORD,

  MODBUS_GATE_BOARD_HEIGHT_1,
  MODBUS_GATE_USER_STAT_1,
  MODBUS_GATE_CHALLENGE_HEIGHT_1,
  MODBUS_GATE_MODE_SELECTION_1,
#endif
  MODBUS_GATE_NUMS = 40,
  
  MODBUS_GATE_ADDRESS_1,
//  MODBUS_GATE_ADDRESS_10 = MODBUS_GATE_ADDRESS_1+modbusId_index,
//  MODBUS_GATE_DISTANCE_10 = 80,
  
//  MODBUS_GATE_TIME_1_SEC = 81,
//  MODBUS_GATE_TIME_10_SEC = 100,
  
//  MODBUS_GATE_TIME_1_MICROSEC = 101,
//  MODBUS_GATE_TIME_10_MICROSEC = 120,
  
//  MODBUS_GATE_FUNC_1_ID = 121,
//  MODBUS_GATE_FUNC_10_ID = 140,
  
//  MODBUS_GATE_TIME_RESULT_SEC,
//  MODBUS_GATE_TIME_RESULT_MILLISEC,
//  MODBUS_GATE_TIME_RESULT_MICROSEC,
  
//  MODBUS_GATE_NAME_1_HI,
//  MODBUS_GATE_NAME_1_LO,  
//  MODBUS_GATE_RFID_1_HI,
//  MODBUS_GATE_RFID_1_LO,
  

};

	



#endif // MODBUS_H

#endif // MODBUS_SUPPLY