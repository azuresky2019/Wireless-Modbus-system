#if defined ( MODBUS_SUPPLY)

#ifndef MODBUS_H
#define MODBUS_H
/*********************************************************************
* INCLUDE
 */

#include "GenericApp.h"
/*********************************************************************
* CONTANTS
 */
#define CRC_NO  0
#define CRC_YES 1

#define MODBUS_SINGLE_READ      0x03
#define MODBUS_SINGLE_WRITE     0x06
#define MODBUS_MULTI_WRITE      0x10
/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 CRClo, CRChi;
extern uint8 modbus_id;
extern uint8 ttt[6];
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
extern void modbus_uart_data_process( uint8 *data_buffer, uint8 len);
/*********************************************************************
 * LOCAL VARIABLES
 */
enum {
  MODBUS_SERIALNUMBER_LOWORD  = 0,             
  MODBUS_SERIALNUMBER_HIWORD  = 2,
  MODBUS_FIRMWARE_VERSION_NUMBER_LO = 4,
  MODBUS_FIRMWARE_VERSION_NUMBER_HI,
  MODBUS_ADDRESS                = 6,
  MODBUS_PRODUCT_ID,
  
  MODBUS_SYS_HOUR = 8,
  MODBUS_SYS_MINUTES,
  MODBUS_SYS_SECONDS,
  MODBUS_SYS_MONTH,
  MODBUS_SYS_DAY,
  MODBUS_SYS_YEAR,
  
  MODBUS_WATER_SWITCH,
  MODBUS_DETECT_POWER,
  MODBUS_WATER_ID,
  MODBUS_WATER_RSSI,
  
  MODBUS_WATER_SWITCH1,
  MODBUS_DETECT_POWER1,
  MODBUS_WATER_ID1,
  
  MODBUS_BAUDRATE = 21,
  MODBUS_PANID = 22,
  MODBUS_DEVICE_TYPE,
  MODBUS_CHANNEL_LIST_HI,
  MODBUS_CHANNEL_LIST_LO,
  MODBUS_SOFTWARE_REV,
  MODBUS_EXTENDED_ADDR_HI,
  MODBUS_EXTENDED_ADDR_LO = 34,
  MODBUS_FACTORY_RESTORE,
  MODBUS_SECURITY_KEY_START = 36,
  MODBUS_SECURITY_KEY_END = 51,
  
  MODBUS_MONITOR_NUM,
};

#define MODBUS_FIRST_MONITOR_ID       (MODBUS_MONITOR_NUM+1)
#define MODBUS_LAST_MONITOR_ID        (MODBUS_FIRST_MONITOR_ID+numMonitor-1)
#define MODBUS_FIRST_DETECT_POWER     (MODBUS_LAST_MONITOR_ID+1)
#define MODBUS_LAST_DETECT_POWER      (MODBUS_FIRST_DETECT_POWER+numMonitor-1)
#define MODBUS_FIRST_WATER_SWITCH     (MODBUS_LAST_DETECT_POWER+1)
#define MODBUS_LAST_WATER_SWITCH      (MODBUS_FIRST_WATER_SWITCH+numMonitor-1)
#define MODBUS_FIRST_DETECT_PIN1      (MODBUS_LAST_WATER_SWITCH+1)
#define MODBUS_LAST_DETECT_PIN1       (MODBUS_FIRST_DETECT_PIN1+numMonitor-1)
#define MODBUS_FIRST_DETECT_PIN2      (MODBUS_LAST_DETECT_PIN1+1)
#define MODBUS_LAST_DETECT_PIN2       (MODBUS_FIRST_DETECT_PIN2+numMonitor-1)
#define MODBUS_FIRST_RSSI             (MODBUS_LAST_DETECT_PIN2+1)
#define MODBUS_LAST_RSSI              (MODBUS_FIRST_RSSI+numMonitor-1)



#endif // MODBUS_H

#endif // MODBUS_SUPPLY