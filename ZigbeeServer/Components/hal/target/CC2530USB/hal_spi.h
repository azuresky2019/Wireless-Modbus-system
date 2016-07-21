#if defined ( SD_SUPPLY)
#ifndef HAL_SPI_H
#define HAL_SPI_H


#include "hal_board.h"
/*****************************************************************************************
 *
 */
/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20
#if 0
#define HAL_SPI_CS_PORT 0
#define HAL_SPI_CS_PIN  2

#define HAL_SPI_CLK_PORT 0
#define HAL_SPI_CLK_PIN  3

#define HAL_SPI_MOSI_PORT 0
#define HAL_SPI_MOSI_PIN  4

#define HAL_SPI_MISO_PORT 0
#define HAL_SPI_MISO_PIN  5

#else
#define HAL_SPI_CS_PORT 0
#define HAL_SPI_CS_PIN  4

#define HAL_SPI_CLK_PORT 0
#define HAL_SPI_CLK_PIN  5

#define HAL_SPI_MOSI_PORT 0
#define HAL_SPI_MOSI_PIN  3

#define HAL_SPI_MISO_PORT 0
#define HAL_SPI_MISO_PIN  2
#endif
/*****************************************************************************************
 *
 */

#define HAL_IO_SET(port, pin, val)                    HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)               st( P##port##_##pin## = val; )

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)          HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)     st( P##port##SEL &= ~BV(pin); \
                                                          P##port##_##pin## = val; \
                                                          P##port##DIR |= BV(pin); )

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)           HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)      st( P##port##SEL |= BV(pin); )

/* 清除接收和发送标志, 写入缓存区x, 等待发送 */
#if 0
#define SPI_TX(x)                                     { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define SPI_WAIT_TXRDY()                              { while(!(U1CSR & BV(1))); }

#define SPI_RX(x)			              { while ( !(U1CSR & BV(0))); x = U1DBUF;}
#define SPI_WAIT_RXRDY()			      { while ( !(U1CSR & BV(2)));}

#else
#define SPI_TX(x)                                     { U0CSR &= ~0x02; U0DBUF = x; while( !(U0CSR & BV(1)) ); }
#define SPI_WAIT_TXRDY()                              { while(!(U0CSR & BV(1))); }

#define SPI_RX(x)			              { U0CSR &= ~0x02; U0DBUF = 0x00; while( !(U0CSR & BV(1))); x = U0DBUF; }
#define SPI_WAIT_RXRDY()			      { while ( !(U0CSR & BV(2)));}
#endif

/* SPI 片选开关 */
#define SPI_BEGIN()     HAL_IO_SET(HAL_SPI_CS_PORT,  HAL_SPI_CS_PIN,  0); /* chip select */
#define SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_SPI_CS_PORT,  HAL_SPI_CS_PIN,  1); /* chip select */         \
}

/*
#define DO_WRITE()        HAL_IO_SET(HAL_SPI_MODE_PORT,  HAL_SPI_MODE_PIN,  1);
#define DO_CONTROL()      HAL_IO_SET(HAL_SPI_MODE_PORT,  HAL_SPI_MODE_PIN,  0);
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \

#define ACTIVATE_RESET()  HAL_IO_SET(HAL_SPI_RESET_PORT, HAL_SPI_RESET_PIN, 0);
#define RELEASE_RESET()   HAL_IO_SET(HAL_SPI_RESET_PORT, HAL_SPI_RESET_PIN, 1);
*/

/**********************************************************************************
 *                      GLOBAL FUNCTION API
 */

extern void hal_spi_init( void);
extern void hal_spi_write_byte( uint8 data);
extern uint8 hal_spi_read_byte( void);

#endif // HAL_SPI_H
#endif // SD_SUPPLY