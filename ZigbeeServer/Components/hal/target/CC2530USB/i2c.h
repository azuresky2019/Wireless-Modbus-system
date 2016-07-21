#if 0
#ifndef I2C_H
#define I2C_H

/********************************************************************
 * INCLUDE
 */
#include "hal_board.h"

/********************************************************************
 * MACROS
 */
#define I2C_CLK   P0_1
#define I2C_DAT   P0_0

#define I2C_DATA_BIT  0

/********************************************************************
 * GLOBAL FUNCTIONS
 */
//extern void delay_us(uint16 s);
extern void I2c_init( void);

/********************************************************************
 * LOCAL FUNCTIONS
 */
void delay_us(uint16 s);
void I2c_Start( void );
void I2c_Stop( void );
bool I2c_Ack( void );	
uint8 I2c_Read_Byte( void );
void I2c_Write_Byte( uint8 SeDAT );
void Write_Data(uint8 Addr,uint8 Value);
uint8 Read_Data(uint8 Addr);
//----------------------------------------------------

#endif // I2C_H
#endif