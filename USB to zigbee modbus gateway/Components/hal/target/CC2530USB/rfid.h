#if 1
#if defined ( RFID_SUPPLY)

#ifndef RFID_H
#define RFID_H

/******************************************************************************
 * INCLUDE
 */
#include "comdef.h"

/******************************************************************************
 * CONTANTS
 */
#define RFID_LOG_IN     0

/******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint8 rfid_lock_flag;

/******************************************************************************
 * GLOBAL FUNCTIONS
 */
extern void RFID_init(void);
extern void RFID_filter(void);
extern void Store_events_to_SD( uint8 *data, uint16 len, uint8 event_id);


#endif    // RFID_H

#endif
#endif