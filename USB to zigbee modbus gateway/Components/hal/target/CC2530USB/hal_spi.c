/*
  //spi
  P0.2 - CS
  P0.3 - CLK
  P0.4 - MOSI
  P0.5 - MISO
*/
#if defined ( SD_SUPPLY)

#include "hal_types.h"
#include "hal_spi.h"

/*****************************************************************************************
 * IO口配置
 */

static void hal_ConfigIO(void)
{
  HAL_CONFIG_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);
}

/*****************************************************************************************
 * SPI配置
 */

static void hal_ConfigSPI(void)
{
   // SPI外围配置
  
  uint8 baud_exponent;
  uint8 baud_mantissa;
  
  // 设置SPI为UART 1, alternative1
  PERCFG &= ~0x01;
//  PERCFG |= 0x02;
  
  // 配置时钟， 主出和主入
  HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_MOSI_PORT, HAL_SPI_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_SPI_MISO_PORT, HAL_SPI_MISO_PIN);
  
  
  // 设置SPI速度到3.25MHz
  
  baud_exponent = 17;
  baud_mantissa = 0;
  
  // 配置SPI
#if 0
  U1UCR  = 0x80;	   // IDLE, 8-N-1
  U1CSR  = 0x00;	   // SPI模式 主
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
  U1BAUD = baud_mantissa;
#else
  U0UCR  = 0x80;	   // IDLE, 8-N-1
  U0CSR  = 0x00;	   // SPI模式 主
  U0GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
  U0BAUD = baud_mantissa;
#endif
}

/*****************************************************************************************
  * SPI初始化
  */
void hal_spi_init( void)
{
  hal_ConfigIO();
  hal_ConfigSPI();
}

 /*****************************************************************************************
  * SPI数据写入
  */

void hal_spi_write_byte( uint8 data)
{
  SPI_BEGIN();
  SPI_TX(data);
  SPI_WAIT_TXRDY();
  SPI_END();
}

 /*****************************************************************************************
  * SPI数据读取
  */
uint8 spi_data;

uint8 hal_spi_read_byte( void)
{
  spi_data = 0xff;
  SPI_BEGIN();
  SPI_RX(spi_data);
  SPI_END();
  return spi_data;
}


#endif  // SD_SUPPLY