#if defined ( SD_SUPPLY)

#include "hal_spi.h"
#include "hal_sd.h"
#include "OnBoard.h"

#if SD_CMD_COMPLEX

sd_struct sds;

#else
typedef struct STORE
{
   uint8 dat[256];
} hal_SD_buffer_t;

hal_SD_buffer_t sectorBuffer;


uint16 readPos = 0;
uint8 sectorPos = 0;
uint8 LBA_Opened = 0;
uint8 Init_Flag;
#endif


#if SD_CRC

//X16 + X12 + X5 + 1 余式表
const uint16 CRCTable[256]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
                            0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
                            0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
                            0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
                            0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
                            0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
                            0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
                            0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
                            0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
                            0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
                            0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
                            0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
                            0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
                            0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
                            0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
                            0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
                            0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
                            0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
                            0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
                            0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
                            0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
                            0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
                            0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
                            0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
                            0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
                            0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
                            0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
                            0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
                            0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
                            0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
                            0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
                            0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
						   };
/*****************************************************************************************

 * 计算16位校验
 */



uint16 halGetCRC16( uint8 *ptr, uint16 len)
{
  uint16 i;
  uint16 ret = 0;

  for( i = 0; i < len; i++)
    ret = ( ret << 8) ^ (uint16)CRCTable[(ret >> 8) ^ *ptr++];

  return ret;
}

/*****************************************************************************************

 * 获得SD命令校验码
 */

uint8 halSDGetCRC( uint8 cmd, uint8 *param)
{
  uint8 i,j;
  uint8 reg = 0;
  uint8 array[5];

  array[0] = cmd;

  for( i = 1; i< 5; i++)
    array[i] = param[4 - 1];

  for( i = 0; i < 5; i++)
  {
    for( j = 0; j < 8; j++)
    {
      reg <<= 1;
      reg ^= ((((array[i] << j) ^ reg) & 0x80) ? 0x9 : 0);
    }
  }
  return (( reg << 1) + 0x01);
}
#endif

#if SD_CMD_COMPLEX

/*****************************************************************************************

 * 初始化SD卡
 */
uint8 halSD_Init(void)
{
  uint8 recbuf[4], ret;
  
  hal_spi_init();
  
  SPI_BEGIN();
  halSD_SPIDelay(25);
  SPI_END();
  
  halSD_SPIDelay(2);
  
  Onboard_wait(5);

  ret = halSD_ResetSD();
  if( ret != SD_NO_ERR)
    return ret;
  
  ret = halSD_ActiveInit();
  if( ret != SD_NO_ERR)
    return ret;
  
  ret = halSD_ReadOCR(4, recbuf);
  if( ret != SD_NO_ERR)
    return ret;
  
  ret = halSD_SetBlockLen( SD_BLOCKSIZE);
  if( ret != SD_NO_ERR)
    return ret;
  
  return (halSD_GetCardInfo());
}

/*****************************************************************************************

 * 获得SD卡的信息
 */
uint8 halSD_GetCardInfo( void)
{
  uint32 tmp;
  uint8 csdbuf[16], ret;
  
  ret = halSD_ReadCSD(16, csdbuf);
  if( ret != SD_NO_ERR)
    return ret;
  
  // 计算块的最大长度
  sds.block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);
  
  // 计算卡中块的个数
  sds.block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
	      	 (csdbuf[C_SIZE_POS2] << 2) +
	 	((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;
  
  tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +   
	((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;
  
  // 获得卡中块的数量
  sds.block_num = sds.block_num*(1<< tmp);
  
  sds.erase_unit = ((csdbuf[SECTOR_SIZE_POS1] & SECTOR_SIZE_MSK1) << 1) +  			
	                 ((csdbuf[SECTOR_SIZE_POS2] & SECTOR_SIZE_MSK2) >> 7) + 1;
  
  return SD_NO_ERR;
}

/*****************************************************************************************

 * 激活卡进入初始化过程
 */
uint8 halSD_ActiveInit(void)
{
  uint8 param[5] = {0,0,0,0,0xff}, resp[5], ret;// cmd[6] = {0x69, 0,0,0,0,0xff};
  uint32 i = 0;
 
  do
  {
    ret = hal_send_cmd_complex( CMD1, param, CMD1_R, resp);
    if( ret!= SD_NO_ERR)
      return ret;
    i++;
  }
  while((( resp[0] & MSK_IDLE) == MSK_IDLE) && ( i< SD_IDLE_WAIT_MAX));
  
/*  i = 0;
  do
  {
    ret = hal_SendCmd(cmd);
    i ++;
  }while ((ret != 0) && ( i<=100)); */
  
  if( i>= SD_IDLE_WAIT_MAX)
    return ret;
           
  
  if( i >= SD_IDLE_WAIT_MAX)
    return SD_ERR_TIMEOUT_WAITIDLE;
  
  return SD_NO_ERR;
}
/*****************************************************************************************

 * 发送命令
 */
uint8 hal_SendCmd( uint8 *param)
{
  uint8 tmp;
  uint8 retry=0;
  uint8 i;

  //发送8个时钟信号
//  hal_spi_write_byte(0xFF);
  SPI_TX(0XFF);
  SPI_WAIT_TXRDY();

  //向SD卡发送6字节命令
  for (i=0;i<0x06;i++) 
  { 
    hal_spi_write_byte(param[i]);
  }
  
  //获得16位的回应
  do 
  {  //读取后8位
    tmp = hal_spi_read_byte();
    retry++;
  }
  while(((tmp&0x80)!= 0)&&(retry<100)); 
  return(tmp);
}

/*****************************************************************************************

 * 发送命令复杂函数
 */
uint8 hal_send_cmd_complex( uint8 cmd, uint8* param, uint8 resptype, uint8 *resp)
{
  int32 i, rlen;
  uint8 tmp, cmd_buf[6];

//  SPI_BEGIN();
  // 发送命令头
  //hal_spi_write_byte(( cmd & 0x3F) | 0x40);
  cmd_buf[0] = ( cmd & 0x3f) | 0x40;
  // 发送命令参数
//  for( i = 0; i < 5; i++)
//    hal_spi_write_byte( param[i]);
  for( i = 1; i < 6; i++)
    cmd_buf[i] = *param++;
  
  // 校验码，只用于第一个命令
/*  if( 0 == cmd)
    hal_spi_write_byte( 0x95);
  else
    hal_spi_write_byte( 0xff); */
  i = 0;

  switch( resptype)
  {
    case R1:
    case R1B:
      rlen = 1;
      break;
    case R2:
      rlen = 2;
      break;
    case R3:
      rlen = 5;
      break;
    default:
      hal_spi_write_byte(0xff);
//      SPI_END();
      return SD_ERR_CMD_RESPTYPE;
      break;
  }
    
  do //等待响应，响应开始位为0
  {
    tmp = hal_SendCmd(cmd_buf);
    i++;
  } while(((tmp & 0x80)!= 0)&& ( i< SD_CMD_TIMEOUT));
  
  if( i >= SD_CMD_TIMEOUT)
    return SD_ERR_CMD_TIMEOUT;  // 命令超时

  // 根据命令获取不同回复长度
  rlen = 0;
  
  for( i = rlen - 1 ; i >= 0; i--)
  {
    resp[i] = tmp;
    tmp = hal_spi_read_byte();
  }
  
//  SPI_END();
  return SD_NO_ERR;
}
/*****************************************************************************************

 * 将32位数转换为字节
 */
void halSD_PackParam( uint8 *parameter, uint32 value)
{
  parameter[3] = (uint8)(value >> 24);
  parameter[2] = (uint8)(value >> 16);
  parameter[1] = (uint8)(value >> 8);
  parameter[0] = (uint8)(value);
}

/*****************************************************************************************

 * 块命令
 */
uint8 halSD_BlockCommand( uint8 cmd, uint8 resptype, uint32 parameter)
{
  uint8 param[4],resp, ret;
  
  parameter <<= SD_BLOCKSIZE_NBITS;     // 调整地址，左移9位
  
  halSD_PackParam(param, parameter);
  
  ret = hal_send_cmd_complex(cmd, param, resptype, &resp);
  
  if( ret!= SD_NO_ERR)
    return ret;
  
  if( resp!= 0)
    return SD_ERR_CMD_RESP;
  
  return SD_NO_ERR;
}

/*****************************************************************************************

 * 复位SD卡
 */
uint8 halSD_ResetSD( void)
{
  uint8 param[5] = {0, 0, 0, 0, 0x95}, resp;
  
  return( hal_send_cmd_complex(CMD0, param, CMD0_R, &resp));
}

/*****************************************************************************************

 * 读CSD寄存器
 */

uint8 halSD_ReadCSD( uint8 csdlen, uint8 *recbuf)
{
  uint8 param[4] = {0, 0, 0, 0}, resp, ret;
  
  ret = hal_send_cmd_complex( CMD9, param, CMD9_R, &resp);
  
  if( ret != SD_NO_ERR)
    return ret;
  
  if( resp!= 0)
    return SD_ERR_CMD_RESP;
  
  return (halSD_ReadRegister( csdlen, recbuf));
}

/*****************************************************************************************

 * 停止数据传输
 */
uint8 halSD_StopTransmission(void)
{
  uint8 param[4] = {0,0,0,0}, resp;
  
  return( hal_send_cmd_complex( CMD12, param, CMD12_R, &resp));
}

/*****************************************************************************************

 * 读card status寄存器
 */
uint8 halSD_ReadCard_Status( uint8 len, uint8 *buffer)
{
  uint8 param[4] = {0,0,0,0};
  
  return ( hal_send_cmd_complex( CMD13, param, CMD13_R, buffer));
}

/*****************************************************************************************

 * 设置一个块的长度
 */
uint8 halSD_SetBlockLen( uint32 length)
{
  uint8 param[5], resp, ret;
  
  halSD_PackParam( param, length);
  
  param[4] = 0xff;
  
  ret = hal_send_cmd_complex( CMD16, param, CMD16_R, &resp);
  
  if( ret!=SD_NO_ERR)
    return ret;
  
  if(resp != 0)
    return SD_ERR_CMD_RESP;
  
  return SD_NO_ERR;
}


/*****************************************************************************************

 * 读单块命令
 */
uint8 halSD_ReadSingleBlock( uint32 blockaddr)
{
  return (halSD_BlockCommand( CMD17, CMD17_R, blockaddr));
}

/*****************************************************************************************

 * 读多块命令
 */
uint8 halSD_ReadMultipleBlock( uint32 blockaddr)
{
  return (halSD_BlockCommand( CMD18, CMD18_R, blockaddr));
}
/*****************************************************************************************

 * 写单块命令
 */
uint8 halSD_WriteSingleBlock( uint32 blockaddr)
{
  return (halSD_BlockCommand( CMD24, CMD24_R, blockaddr));
}

/*****************************************************************************************

 * 写多块命令
 */
uint8 halSD_WriteMultipleBlock( uint32 blockaddr)
{
  return (halSD_BlockCommand( CMD25, CMD25_R, blockaddr));
}
/*****************************************************************************************

 * 设置擦除起始地址
 */
uint8 halSD_EraseStartBlock(uint32 startblock)
{
  return (halSD_BlockCommand(CMD32, CMD32_R, startblock));
}

/*****************************************************************************************

 * 设置擦除终止地址
 */
uint8 halSD_EraseEndBlock(uint32 startblock)
{
  return (halSD_BlockCommand(CMD33, CMD33_R, startblock));
}
/*****************************************************************************************

 * 擦除已选中的块
 */
uint8 halSD_EraseSelectedBlock(void)
{
  uint8 param[4], resp, tmp;
  
  halSD_PackParam( param, 0 );
  
  tmp = hal_send_cmd_complex( CMD38, param, CMD38_R, &resp);
  
  if(tmp != SD_NO_ERR)
    return tmp;
  
  if( halSD_WaitBusy( SD_WAIT_ERASE) != SD_NO_ERR)
    return SD_ERR_TIMEOUT_ERASE;
  else
    return SD_NO_ERR;
  
}
/*****************************************************************************************

 * 读操作条件寄存器OCR
 */
uint8 halSD_ReadOCR(uint8 ocrlen, uint8 *recbuf)
{
  uint8 param[5] = {0,0,0,0, 0xff},resp[5],tmp;
  
  tmp = hal_send_cmd_complex(CMD58, param, CMD58_R, resp);
  if (tmp != SD_NO_ERR)			
      return tmp;		 										
                                                                                              
  if (resp[0] != 0)
      return SD_ERR_CMD_RESP;			 				
  
  for (tmp = 0; tmp < 4; tmp++)
      recbuf[tmp] = resp[tmp + 1];						
  
  return SD_NO_ERR;
}
/*****************************************************************************************

 * 读SD卡的 SD_Status 寄存器
 */
uint8 halSD_ReadSD_Status(uint8 sdslen, uint8 *recbuf)
{
  uint8 param[4] = {0,0,0,0},resp[2],ret;
  
  ret = hal_send_cmd_complex(CMD55, param, CMD55_R, resp);			
  if (ret != SD_NO_ERR)
    return ret;											
                                                                                               
  if (resp[0] != 0)
    return SD_ERR_CMD_RESP;								
  
  ret = hal_send_cmd_complex(ACMD13, param, ACMD13_R, resp);		
  if (ret != SD_NO_ERR)
    return ret;											
                                                                                              
  if ((resp[0] != 0) || (resp[1] != 0))
    return SD_ERR_CMD_RESP;								
      
  return (halSD_ReadBlockData(sdslen, recbuf));				
}
/*****************************************************************************************

 * 读SD卡的 SCR 寄存器
 */

uint8 halSD_ReadSCR(uint8 scrlen, uint8 *recbuf)
{
  uint8 param[4] = {0,0,0,0},resp,ret;
  
  ret = hal_send_cmd_complex(CMD55, param, CMD55_R, &resp);					
  if (ret != SD_NO_ERR)											
    return ret;													
                                                                                               
  if (resp != 0)
    return SD_ERR_CMD_RESP;									
  
  ret = hal_send_cmd_complex(ACMD51, param, ACMD51_R, &resp);   			
  if (ret != SD_NO_ERR)											
    return ret;													
                                                                                                                                                      
  if (resp != 0)
    return SD_ERR_CMD_RESP;						 			
      
  return (halSD_ReadBlockData(scrlen, recbuf));	 	
}
/*****************************************************************************************

 *  从SD卡读取数据	
 */
uint8 halSD_ReadRegister( uint32 len, uint8 *recbuf)
{
  uint32 i = 0;
  uint8 resp;
  
  do
  {
    resp = hal_spi_read_byte();
    i ++;           // 等待起始令牌
  }while(( resp == 0xff) && ( i< SD_READREG_TIMEOUT));
  
  if( i>= SD_READREG_TIMEOUT)
    return SD_ERR_TIMEOUT_READ;
  
  if( resp!= SD_TOK_READ_STARTBLOCK)
  {
    recbuf[0] = resp;
    i = 1;
  }
  else
    i = 0;
  
  for(; i< len; i++)
    recbuf[i] = hal_spi_read_byte();
  
  i = hal_spi_read_byte();
  
  i = ( i<<8) + hal_spi_read_byte();
  
  hal_spi_write_byte(0xff);
  
  return SD_NO_ERR;
}

/*****************************************************************************************

 * 从SD卡中读取数据块	
 */
uint8 halSD_ReadBlockData(uint32 len, uint8 *recbuf)
{
  uint8 tmp;
  uint32 i = 0;
  
  do
  {
    tmp = hal_spi_read_byte();
    i++;
  }while((tmp == 0xFF) && (i < sds.timeout_read));
  
  if (i >= sds.timeout_read)
  {
    return SD_ERR_TIMEOUT_READ;
  }
          
  if (tmp != SD_TOK_READ_STARTBLOCK)
  {
    hal_spi_write_byte(0xFF);
    return SD_ERR_DATA_START_TOK;
  }
  
  for (i = 0; i < len; i++)
    recbuf[i] = hal_spi_read_byte();
          
  i = hal_spi_read_byte();								
  i = (i << 8) + hal_spi_read_byte();
  
  hal_spi_write_byte(0xFF); 
  
  return SD_NO_ERR;
}

/*****************************************************************************************

 * 向sd卡写数据块	
 */
uint8 halSD_WriteBlockData(uint8 bmulti, uint32 len, uint8 *sendbuf)
{
  uint16 i;
  uint8 tmp;
          
  hal_spi_write_byte(0xFF);								
  
  if (bmulti == 1)
    hal_spi_write_byte(SD_TOK_WRITE_STARTBLOCK_M);	
  else
    hal_spi_write_byte(SD_TOK_WRITE_STARTBLOCK);		
  
  for (i = 0; i < len; i++)
    hal_spi_write_byte(sendbuf[i]);	
  
    hal_spi_write_byte((i >> 8) & 0xFF);
    hal_spi_write_byte(i & 0xFF); 						
    tmp = hal_spi_read_byte();
  if ((tmp & SD_RESP_DATA_MSK) != SD_RESP_DATA_ACCETPTED)	
  {		
    hal_spi_write_byte(0xFF);		
    return SD_ERR_DATA_RESP;					
  }
  
          
  if (halSD_WaitBusy(SD_WAIT_WRITE) != SD_NO_ERR)			
    return SD_ERR_TIMEOUT_WRITE;				
  else
    return SD_NO_ERR; 					
 }
/*****************************************************************************************

 * 停止写多块	
 */
void halSD_StopMultiToken(void)
{
  hal_spi_write_byte(0xFF);
  hal_spi_write_byte(SD_TOK_STOP_MULTI);	
  hal_spi_read_byte();
}
/*****************************************************************************************

 * 查询SD卡是否忙	
 */
uint8 halSD_WaitBusy(uint8 waittype)
{
  uint32 timeout, i = 0;
  uint8 tmp;
  
  if (waittype == SD_WAIT_WRITE)
    timeout = sds.timeout_write;
  else
    timeout = sds.timeout_erase;
  
  do
  { 					        /* 等待忙结束  */
    tmp = hal_spi_read_byte();
    i++;
  }while ((tmp != 0xFF) && (i < timeout));		/* 忙时收到的值为0xFF */    
                                            
  if(i < timeout) 
    return SD_NO_ERR;			
  else 
    return SD_ERR_TIMEOUT_WAIT;	
}
/*****************************************************************************************

 *  SPI总线延时
 */
void halSD_SPIDelay(uint8 value)
{
  uint8 i;
  
  for (i = 0; i < value; i++)
  {
//    hal_spi_write_byte(0xFF);
    SPI_TX(0xFF);
    SPI_WAIT_TXRDY();
  }
}

#else   // !SD_CMD_COMPLEX
/*****************************************************************************************

 * 向SD卡中写入命令，并返回回应的第二个字节
 */
uint8 halWriteCmdSD(uint8 *CMD)
{
  uint8 tmp;
  uint8 retry=0;
  uint8 i;
  
  //禁止SD卡片选
//  SPI_END();
  
  //发送8个时钟信号
  hal_spi_write_byte(0xFF);
  
  //使能SD卡片选
//  SPI_BEGIN();
  
  //向SD卡发送6字节命令
  for (i=0;i<0x06;i++) 
  { 
    hal_spi_write_byte(CMD[i]);
  }
  
  //获得16位的回应
//  hal_spi_read_byte(); //无视 
  do 
  {  //读取后8位
    tmp = hal_spi_read_byte();
    retry++;
  }
  while(((tmp&0x80)!= 0)&&(retry<100)); 
  return(tmp);
}


/*****************************************************************************************
  * 初始化SD卡到SPI模式
  */
  
uint8 halSDInit( void)
{  
  uint8 retry,temp;
  uint8 i;
  uint8 CMD[] = {0x40,0x00,0x00,0x00,0x00,0x95};
  
  hal_spi_init();
  
  Init_Flag=1; //将初始化标志置1
  
  for(i=0;i<25;i++) 
  {
    hal_spi_write_byte(0xff); //发送至少74个时钟信号
  }
/*  
  do
  {
    hal_spi_write_byte(0x40); //发送至少74个时钟信号
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x95);
    
    asm("NOP");
    asm("NOP");
    asm("NOP");

    hal_spi_write_byte(0x41); //发送至少74个时钟信号
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0xff);
    
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    hal_spi_write_byte(0x4a); //发送至少74个时钟信号
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0x00);
    hal_spi_write_byte(0xff);
    
    temp = hal_spi_read_byte();
  }
  while( (temp&0x80)!=0); */
  //向SD卡发送CMD0
  retry=0;
  do
  { //为了能够成功写入CMD0,在这里写200次
   temp=halWriteCmdSD(CMD);
   retry++;
   if(retry==200) 
   { //超过200次
     return(INIT_CMD0_ERROR);//CMD0 Error!
   }
  } 
  while((temp&0x80)!=0);  //回应01h，停止写入
  
  //发送CMD1到SD卡
  CMD[0] = 0x41; //CMD1
  CMD[5] = 0xFF;
  retry=0;
  do
  { //为了能成功写入CMD1,写100次
   temp=halWriteCmdSD(CMD);
   retry++;
   if(retry==100) 
   { //超过100次
     return(INIT_CMD1_ERROR);//CMD1 Error!
   }
  } 
  while((temp&0x80)!=0);//回应00h停止写入
  
  Init_Flag=0; //初始化完毕，初始化标志清零
  
  SPI_END(); //片选无效
  return(0); //初始化成功
}

/*****************************************************************************************
  *   块读取SD卡的数据
  */

uint8 halReadBlockSD(uint8 *CMD, uint8 *Buffer, uint16 Size)
{
  uint8 i;
  uint8 retry, temp;

  // 发送命令给SD卡
  retry = 0;
  do{
    // 发送100次
    temp = halWriteCmdSD( CMD);
    retry++;
    if( retry == 100)
    {
      return READ_BLOCK_ERROR;
    }
  }
  while( (temp&0x80)!= 0);

  // 读取第一个字节0XFE
  while ( hal_spi_read_byte() != 0xFE)
  {
  }

  for( i = 0; i < Size; i++)
  {
    *Buffer++ = hal_spi_read_byte();
  }

  // 读取校验字,无视
  hal_spi_read_byte();
  hal_spi_read_byte();

  return 0;
  
}



/*****************************************************************************************
  * 读SD卡的CID寄存器   共16字节    返回0说明读取成功
  */
uint8 halReadCID(uint8 *Buffer)
{
  // 读取CID寄存器的命令
  uint8 CMD[] = {0x4A,0x00,0x00,0x00,0x00,0xFF};
  uint8 temp;
  temp = halReadBlockSD(CMD,Buffer,16);
  return (temp);
}


/*****************************************************************************************
  * 读SD卡的CSD寄存器   共16字节    返回0说明读取成功
  */
uint8 halReadCSD(uint8 *Buffer)
{ 
  //读取CSD寄存器的命令
  uint8 CMD[] = {0x49,0x00,0x00,0x00,0x00,0xFF};
  uint8 temp;
  temp=halReadBlockSD(CMD,Buffer,16); //read 16 bytes
  return(temp);
}

/*****************************************************************************************

  * 返回SD卡的容量，单位为M
  * u08 == C_SIZE / (2^(9-C_SIZE_MULT))
  * SD卡的名称
  */
hal_SD_volume_info_t *halGetVolumeInfoSD( void)
{   
//  uint8 i;
//  uint8 c_temp[5];
  hal_SD_volume_info_t SD_volume_Info, *vinf;
  vinf = &SD_volume_Info; 
  
  //读取CSD寄存器
  halReadCSD(sectorBuffer.dat);
  
  //获取总扇区数
  vinf->sector_count = sectorBuffer.dat[6] & 0x03;
  vinf->sector_count <<= 8;
  vinf->sector_count += sectorBuffer.dat[7];
  vinf->sector_count <<= 2;
  vinf->sector_count += (sectorBuffer.dat[8] & 0xc0) >> 6;
  
  // 获取multiplier
  vinf->sector_multiply = sectorBuffer.dat[9] & 0x03;
  vinf->sector_multiply <<= 1;
  vinf->sector_multiply += (sectorBuffer.dat[10] & 0x80) >> 7;
  
  //获取SD卡的容量
  vinf->size_MB = vinf->sector_count >> (9-vinf->sector_multiply);
  
  // 获取卡名
  halReadCID(sectorBuffer.dat);
  vinf->name[0] = sectorBuffer.dat[3];
  vinf->name[1] = sectorBuffer.dat[4];
  vinf->name[2] = sectorBuffer.dat[5];
  vinf->name[3] = sectorBuffer.dat[6];
  vinf->name[4] = sectorBuffer.dat[7];
  vinf->name[5] = 0x00; //end flag
  
  return vinf;
}



/*****************************************************************************************
  *读扇区
  */

uint8 halReadSectorSD(uint32 sector,uint8 *buffer)
{  
  uint8 retry;
  //命令16
  uint8 CMD[] = {0x51,0x00,0x00,0x00,0x00,0xFF}; 
  uint8 temp;
  
  //地址变换   由逻辑块地址转为字节地址
  sector = sector << 9; //sector = sector * 512
  
  CMD[1] = ((sector & 0xFF000000) >>24 );
  CMD[2] = ((sector & 0x00FF0000) >>16 );
  CMD[3] = ((sector & 0x0000FF00) >>8 );
  //将命令16写入SD卡
  retry=0;
  do
  {  //为了保证写入命令  一共写100次
    temp = halWriteCmdSD(CMD);
    retry++;
    if(retry==100) 
    {
      return(READ_BLOCK_ERROR); 
    }
  }
  while(temp!=0); 
    
  
  
  while (hal_spi_read_byte() != 0xfe);
  readPos=0;
  halGetDataSD(512,buffer) ;  //512字节被读出到buffer中
  return 0;
}


/*****************************************************************************************
  * 获取数据到buffer中
  */


void halGetDataSD(uint16 size,uint8 *buffer) 
{
  uint16 j;
  for (j=0; ((j<size) && (readPos < 512)); j++)
  {
    *buffer++ = hal_spi_read_byte();
    readPos++;
  }
  
  if( readPos == 512)
  {
    hal_spi_read_byte();
    hal_spi_read_byte(); // 校验
    
    readPos = 0;         // 重置扇区读取偏移量
    sectorPos++;			// 需要读取下一个扇区
    LBA_Opened = 0;		// 当一个扇区打开后设置为1
    
    SPI_END();
  }
}


/*****************************************************************************************
  * 写512个字节到SD卡的某一个扇区中去   返回0说明写入成功
  */
uint8 halWriteSectorSD(uint32 addr,uint8 *Buffer)
{  
   uint8 tmp,retry;
   uint16 i;
   //命令24
   uint8 CMD[] = {0x58,0x00,0x00,0x00,0x00,0xFF}; 
   addr = addr << 9; //addr = addr * 512
 
   CMD[1] = ((addr & 0xFF000000) >>24 );
   CMD[2] = ((addr & 0x00FF0000) >>16 );
   CMD[3] = ((addr & 0x0000FF00) >>8 );
   //写命令24到SD卡中去
   retry=0;
   do
   {  //为了可靠写入，写100次
      tmp=halWriteCmdSD(CMD);
      retry++;
      if(retry==100) 
      { 
        return(tmp); //send commamd Error!
      }
   }
   while(tmp!=0); 
   
   //在写之前先产生100个时钟信号
   for (i=0;i<100;i++)
   {
      hal_spi_read_byte();
   }
 
   //写入开始字节
   hal_spi_write_byte(0xFE); 
 
   //现在可以写入512个字节
   for (i=0;i<512;i++)
   {
      hal_spi_write_byte(*Buffer++); 
   }
   //CRC-Byte 
   hal_spi_write_byte(0xFF); //Dummy CRC
   hal_spi_write_byte(0xFF); //CRC Code
   
    
   tmp=hal_spi_read_byte();   // read response
   if((tmp & 0x1F)!=0x05) // 写入的512个字节是未被接受
   {
     SPI_END();
     return(WRITE_BLOCK_ERROR); //Error!
   }
   //等到SD卡不忙为止
//因为数据被接受后，SD卡在向储存阵列中编程数据
   while (hal_spi_read_byte()!=0xff){};
 
   //禁止SD卡
   SPI_END();
   
   return(0);//写入成功
}

/*****************************************************************************************
  * 
  */
uint8 halStartReadSectorSD( uint32 sector)
{
  uint8 retry;
  uint8 CMD[] = {0x51, 0x00, 0x00, 0x00, 0x00, 0xFF};
  uint8 temp;

  sector = sector << 9;   // sector = sector * 512

  CMD[1] = (( sector & 0xFF000000) >> 24);
  CMD[2] = (( sector & 0x00FF0000) >> 16);
  CMD[3] = (( sector & 0x0000FF00) >> 8);

  retry = 0;

  do{
    temp = halWriteCmdSD( CMD);
    retry ++;
    if (retry ==100)
    {
            return READ_BLOCK_ERROR;
    }
  }
  while( temp != 0);

  while ( hal_spi_read_byte() != 0xFE)
  {
  }

  return 0;
}



/*****************************************************************************************
  * 获取LBA数据
  */
  
void halGetData_LBA( uint32 lba, uint16 size, uint8* buffer)
{
  // 从SD卡的LBA地址获取数据
  // 如果一个新的扇区已经被读取，就从头开始读
  if( readPos == 0)
    halStartReadSectorSD( lba);
  halGetDataSD(size, buffer);
}


/*****************************************************************************************
  * 获取扇面偏移
  */
void halGetSectorOffset( uint32 lba, uint16 offset)
{
  uint8 temp[1];
  hal_LBA_close();
  while( readPos < offset)
  {
    halGetData_LBA( lba, 1, temp);
  }
}

/*****************************************************************************************
  * 关闭LBA
  */
  
void hal_LBA_close( void)
{
  uint8 temp[1];
  while ((readPos != 0x00) | (LBA_Opened = 1))
  {
    halGetDataSD( 1,temp);
  }
}

#endif    // SD_CMD_COMPLEX

#endif // SD_SUPPLY

