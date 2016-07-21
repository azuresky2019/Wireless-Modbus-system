#if 0   //20140626 打开虚拟I2C，会影响串口读取MODBUS时 刷RFID卡， 具体原因暂没时间查明

/*************************************************************
 * INCLUDE
 */
#include "I2C.h"
#include "OnBoard.h"



/*************************************************************
 * CONTANTS
 */
#define I2C_DELAY_TIME    2

void delay_us(uint16 s)
{
  uint16 i;
  for(i=0; i<s; i++);
}
/*************************************************************
 * @fn        I2c_init
 *
 * @brief     Initialize simulate I2C
 * 
 * @param     none
 *
 * @return    none
 */
void I2c_init( void)
{
  P0SEL &= ~( BV(1) | BV(0));
  P0DIR |= BV(1) | BV(0);
}

/*************************************************************
 * @fn        I2c_Start
 *
 * @brief     This func for start
 *            when SCL is '1' and SDA from '1' to '0'
 * 
 * @param     none
 *
 * @return    none
 */
void I2c_Start( void )
{
#if 0
  I2C_CLK = 1;
//    MicroWait(I2C_DELAY_TIME);
  I2C_DAT = 1;
  MicroWait(I2C_DELAY_TIME);
  I2C_DAT = 0;
  MicroWait(I2C_DELAY_TIME);
  I2C_CLK = 0;	
#else
    I2C_DAT = 1;
    I2C_CLK = 1; 
    delay_us(I2C_DELAY_TIME);
    I2C_DAT = 0;
    delay_us(I2C_DELAY_TIME);
//    I2C_DAT = 0;// delay_us(2);
    I2C_CLK = 0;
    delay_us(I2C_DELAY_TIME);
#endif
}

/*************************************************************
 * @fn        I2c_Stop
 *
 * @brief     This func for stop
 *            when SCL is '1' and SDA from '0' to '1'
 * 
 * @param     none
 *
 * @return    none
 */
void I2c_Stop( void )//when SCL is'1' and SDA from '0'to '1'
{
#if 0
  I2C_CLK = 1;
  I2C_DAT = 0;
  MicroWait(I2C_DELAY_TIME);
  I2C_CLK = 1;
  MicroWait(I2C_DELAY_TIME);
  I2C_DAT = 1;
#else
  I2C_DAT = 0;
  I2C_CLK = 1;
  delay_us(I2C_DELAY_TIME);
  I2C_DAT = 1;
  delay_us(I2C_DELAY_TIME);
  I2C_CLK = 0;
  delay_us(I2C_DELAY_TIME);
#endif
}

/*************************************************************
 * @fn        I2c_Ack
 *
 * @brief     This func for ACK
 *            after send 8byte,need set SDA high,wait the slaver pulldown the SDA
 * 
 * @param     none
 *
 * @return    bool
 */
bool I2c_Ack( void )	
{
#if 0
  uint8 Ack;
  P0DIR &= ~BV(0);      // Make i2c data wire to be input
  while(I2C_DAT)
  {
  //   Waiting for acknowledge low
  }
  MicroWait(I2C_DELAY_TIME);
  I2C_CLK = 1;
  MicroWait(I2C_DELAY_TIME);
  I2C_CLK = 0;
  Ack = FALSE;
  P0DIR |= BV(0);     // Restore i2c data wire direction
  return Ack;
#else
   P0DIR &= ~BV(I2C_DATA_BIT);
   I2C_DAT = 1;
   I2C_CLK = 1;
   delay_us(I2C_DELAY_TIME);
   I2C_CLK = 0;
   delay_us(I2C_DELAY_TIME);
   I2C_CLK = 0;
   P0DIR |= BV(I2C_DATA_BIT); 
   if( I2C_DAT == 1)
     return FALSE;
   return TRUE;
#endif
}

/*************************************************************
 * @fn        I2c_Read_Byte
 *
 * @brief     This func for read byte
 *            Make i2c data wire to be input
 * 
 * @param     none
 *
 * @return    uint8 
 */
uint8 I2c_Read_Byte( void )
{
  uint8 Num, Recv = 0;
  I2C_DAT = 1;
  P0DIR &= ~BV(I2C_DATA_BIT);    // Make i2c data wire to be input
  for(Num = 0; Num < 8; Num++)
  {
    I2C_CLK = 0; 
    delay_us(I2C_DELAY_TIME);
    I2C_CLK = 1;
    Recv = (Recv << 1) | I2C_DAT;// delay_us(2);
   // I2C_CLK = 1; 
    delay_us(I2C_DELAY_TIME);
  }
  P0DIR |= BV(I2C_DATA_BIT);     // Restore i2c data wire direction
  return Recv;
}

/*************************************************************
 * @fn        I2c_Write_Byte
 *
 * @brief     This func for write byte
 *            One times send 8bit, from high to low
 * 
 * @param     uint8 - The byte want to be writen
 *
 * @return    none 
 */
void I2c_Write_Byte( uint8 SeDAT )
{
  uint8 Num;
  for(Num = 0; Num < 8; Num++)
  {
    I2C_CLK = 0; 
    if ( SeDAT & 0X80 )
      I2C_DAT = 1;
    else
      I2C_DAT = 0;
    delay_us(I2C_DELAY_TIME);
    I2C_CLK = 1; 
    delay_us(I2C_DELAY_TIME);
    SeDAT <<= 1;
  }
  I2C_CLK = 0;
 // delay_us(1);
  I2C_DAT = 1;
}

/*************************************************************
 * @fn        Write_Data
 *
 * @brief     This func for write words
 *            We can write 2 byte into register 
 * 
 * @param     uint8 - Address of slave device
 *            uint8 - The byte want to be writen
 *
 * @return    none 
 */
void Write_Data(uint8 Addr,uint8 Value)
{
  I2c_Start(); //make the I2C bus begin
  
  I2c_Write_Byte(0xC8);//master send slaver address
  I2c_Ack();             //we can get Ackknowledge from slaver if equal to 0,means right
 
  I2c_Write_Byte(Addr);//master send register to slaver
  I2c_Ack();            
  
  I2c_Write_Byte(Value);//master send value to slaver
  if( I2c_Ack() )            
  {                          
    I2c_Stop();
 //   return False;
  }
  //MicroWait(I2C_DELAY_TIME);
  delay_us(I2C_DELAY_TIME);
  I2c_Stop();
}

/*************************************************************
 * @fn        Write_Data
 *
 * @brief     This func for read data form slave device
 * 
 * @param     uint8 - Address of slave device
 *
 * @return    uint8 - The byte read form the Addr 
 */
uint8 Read_Data(uint8 Addr)
{
  uint8 Value;
  
  HAL_DISABLE_INTERRUPTS();
  I2c_Start(); //make the I2C bus begin
  //if((Addr != Last_Addr + 1) || (Addr & 0xFF))
  //{
  I2c_Write_Byte(0xC8 );  // write address 1100100 and make the R/w with 0
  I2c_Ack();
    
  I2c_Write_Byte(Addr);// write register to slaver
  I2c_Ack();

  I2c_Start(); //repeat make the I2C bus begin

  I2c_Write_Byte(0xC9);// write address 1100100 and make the R/w with 1
  I2c_Ack();
  
  Value = I2c_Read_Byte(); // we recive the content from slaver 
  I2c_Stop();
  
  HAL_ENABLE_INTERRUPTS(); 
  return Value;
}

#endif