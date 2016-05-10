#if defined ( SD_SUPPLY)

#ifndef _SD_H_
#define _SD_H_

#define INIT_CMD0_ERROR		0x01
#define INIT_CMD1_ERROR		0x02
#define WRITE_BLOCK_ERROR	0x03
#define READ_BLOCK_ERROR	0x04

#define SD_CMD_SET		1
#define SD_CRC			0
#define SD_CMD_COMPLEX          1


typedef struct SD_VOLUME_INFO
 { //SD/SD Card info
   uint16   size_MB;
   uint8    sector_multiply;
   uint16   sector_count;
   uint8    name[6];
 } hal_SD_volume_info_t;


#if SD_CMD_SET
/*****************************************************************************************

 * 基本命令集
 */

#define R1			1
#define R1B			2
#define R2			3
#define R3			4

/* 复位SD 卡 Reset cards to idle state */
#define CMD0 0
#define CMD0_R R1

/* 读OCR寄存器 Read the OCR (MMC mode, do not use for SD cards) */
#define CMD1 1
#define CMD1_R R1

/* 读CSD寄存器 Card sends the CSD */
#define CMD9 9
#define CMD9_R R1

/* 读CID寄存器 Card sends CID */
#define CMD10 10
#define CMD10_R R1

/* 停止读多块时的数据传输 Stop a multiple block (stream) read/write operation */
#define CMD12 12
#define CMD12_R R1B

/* 读 Card_Status 寄存器 Get the addressed card's status register */
#define CMD13 13
#define CMD13_R R2

/***************************** 块读命令集 Block read commands **************************/

/* 设置块的长度 Set the block length */
#define CMD16 16
#define CMD16_R R1

/* 读单块 Read a single block */
#define CMD17 17
#define CMD17_R R1

/* 读多块,直至主机发送CMD12为止 Read multiple blocks until a CMD12 */
#define CMD18 18
#define CMD18_R R1

/***************************** 块写命令集 Block write commands *************************/
/* 写单块 Write a block of the size selected with CMD16 */
#define CMD24 24
#define CMD24_R R1

/* 写多块 Multiple block write until a CMD12 */
#define CMD25 25
#define CMD25_R R1

/* 写CSD寄存器 Program the programmable bits of the CSD */
#define CMD27 27
#define CMD27_R R1

/***************************** 写保护 Write protection *****************************/
/* Set the write protection bit of the addressed group */
#define CMD28 28
#define CMD28_R R1B

/* Clear the write protection bit of the addressed group */
#define CMD29 29
#define CMD29_R R1B

/* Ask the card for the status of the write protection bits */
#define CMD30 30
#define CMD30_R R1

/***************************** 擦除命令 Erase commands *******************************/
/* 设置擦除块的起始地址 Set the address of the first write block to be erased */
#define CMD32 32
#define CMD32_R R1

/* 设置擦除块的终止地址 Set the address of the last write block to be erased */
#define CMD33 33
#define CMD33_R R1

/* 擦除所选择的块 Erase the selected write blocks */
#define CMD38 38
#define CMD38_R R1B

/***************************** 锁卡命令 Lock Card commands ***************************/
/* 设置/复位密码或上锁/解锁卡 Set/reset the password or lock/unlock the card */
#define CMD42 42
#define CMD42_R	R1B
/* Commands from 42 to 54, not defined here */

/***************************** 应用命令 Application-specific commands ****************/
/* 禁止下一个命令为应用命令  Flag that the next command is application-specific */
#define CMD55 55
#define CMD55_R R1

/* 应用命令的通用I/O  General purpose I/O for application-specific commands */
#define CMD56 56
#define CMD56_R R1

/* 读OCR寄存器  Read the OCR (SPI mode only) */
#define CMD58 58
#define CMD58_R R3

/* 使能或禁止 CRC Turn CRC on or off */
#define CMD59 59
#define CMD59_R R1

/***************************** 应用命令 Application-specific commands ***************/
/* 获取 SD Status寄存器 Get the SD card's status */
#define ACMD13 13
#define ACMD13_R R2

/* 得到已写入卡中的块的个数 Get the number of written write blocks (Minus errors ) */
#define ACMD22 22
#define ACMD22_R R1

/* 在写之前,设置预先擦除的块的个数 Set the number of write blocks to be pre-erased before writing */
#define ACMD23 23
#define ACMD23_R R1

/* 读取OCR寄存器 Get the card's OCR (SD mode) */
#define ACMD41 41
#define ACMD41_R R1

/* 连接/断开CD/DATA[3]引脚上的上拉电阻 Connect or disconnect the 50kOhm internal pull-up on CD/DAT[3] */
#define ACMD42 42
#define ACMD42_R R1

/* 读取SCR寄存器 Get the SD configuration register */
#define ACMD51 51
#define ACMD51_R R1

#endif
/**************************************************************************
 *                     LOCAL FUNTION PROTOTYPE
 */
#if SD_CMD_COMPLEX

#define SD_CMD_TIMEOUT   	 	100

#define SD_BLOCKSIZE 			512			/* SD卡块的长度 */

#define SD_BLOCKSIZE_NBITS		9  

/* SD卡信息结构体定义 */ 
typedef struct SD_STRUCT
{	
  uint32 block_num;				/* 卡中块的数量 */
  uint32 block_len;				/* 卡的块长度(单位:字节) */
  uint32 erase_unit;				/* 一次可擦除的块个数 */
  
  uint32 timeout_read;			/* 读块超时时间(单位: 8 SPI clock) */
  uint32 timeout_write;			/* 写块超时时间(单位: 8 SPI clock) */
  uint32 timeout_erase;			/* 擦块超时时间(单位: 8 SPI clock) */
	
}sd_struct;

extern sd_struct sds;

/* 等待类型 Wait Type */
#define SD_WAIT_READ			  0x00		//读等待
#define SD_WAIT_WRITE			  0x01		//写等待
#define SD_WAIT_ERASE		 	  0x02		//擦除等待

#define SD_READREG_TIMEOUT		  8

/* CSD中一些域的字节位置(高字节在前) */
#define TAAC_POS 			1			//TACC
#define NSAC_POS			2			//NSAC

#define READ_BL_LEN_POS		        5			//READ_BL_LEN

#define C_SIZE_POS1			6			//C_SIZE upper  2-bit
#define C_SIZE_POS2			7			//C_SIZE middle 8-bit
#define C_SIZE_POS3			8			//C_SIZE lower	2-bit

#define C_SIZE_MULT_POS1	        9			//C_SIZE_MULT upper 2-bit
#define C_SIZE_MULT_POS2	        10			//C_SIZE_MULT lower 1-bit	

#define SECTOR_SIZE_POS1	        10			//SECTOR_SIZE upper 5-bit
#define SECTOR_SIZE_POS2	        11			//SECTOR_SIZE lower 2-bit

#define R2WFACTOR_POS 		        12			//R2WFACTOR_POS

//CSD中一些域的掩码
#define TAAC_MSK			0x07		//TACC 域掩码
#define NSAC_MSK			0x78		//NSAC 域掩码

#define READ_BL_LEN_MSK		        0x0F		//READ_BL_LEN 的掩码

#define C_SIZE_MSK1			0x03		//C_SIZE 高2位掩码
#define C_SIZE_MSK3			0xC0		//C_SIZE 低2位掩码

#define C_SIZE_MULT_MSK1 	        0x03		//C_SIZE_MULT 的高2位掩码
#define C_SIZE_MULT_MSK2 	        0x80		//C_SIZE_MULT 的低2位掩码

#define R2WFACTOR_MSK		        0x1C		//R2WFACTOR 掩码

#define SECTOR_SIZE_MSK1	        0x3F		//SECTOR_SIZE 的高5位
#define SECTOR_SIZE_MSK2	        0x80		//SECTOR_SIZE 的低2位

/* 定义在初始化阶段,等待SD卡退出空闲状态的重试次数 */
#define SD_IDLE_WAIT_MAX     	1000

/* R1和R2高字节错误码 R1 and upper byte of R2 error code */
#define MSK_IDLE          		  0x01
#define MSK_ERASE_RST     		  0x02
#define MSK_ILL_CMD       		  0x04
#define MSK_CRC_ERR       		  0x08
#define MSK_ERASE_SEQ_ERR 		  0x10
#define MSK_ADDR_ERR      		  0x20
#define MSK_PARAM_ERR     		  0x40

/* 数据令牌 Data Tokens */
#define SD_TOK_READ_STARTBLOCK    0xFE
#define SD_TOK_WRITE_STARTBLOCK   0xFE
#define SD_TOK_READ_STARTBLOCK_M  0xFE
#define SD_TOK_WRITE_STARTBLOCK_M 0xFC
#define SD_TOK_STOP_MULTI         0xFD

/* 数据响应令牌 Data Response Tokens */
#define SD_RESP_DATA_MSK		  0x0F		//数据响应掩码
#define SD_RESP_DATA_ACCETPTED	  0x05		//数据被接受
#define SD_RESP_DATA_REJECT_CRC	  0x0B      //由于CRC错误而被拒绝
#define SD_RESP_DATA_REJECT_WRITE 0x0D		//由于写错误而被拒绝

/* 错误码 error code */
#define   SD_NO_ERR			0x00			// 函数执行成功
#define   SD_ERR_NO_CARD		0x01			// SD卡没有完全插入到卡座中
#define   SD_ERR_USER_PARAM      	0x02			// 用户使用API函数时，入口参数有错误
#define   SD_ERR_CARD_PARAM		0x03			// 卡中参数有错误（与本模块不兼容）
#define	  SD_ERR_VOL_NOTSUSP            0x04			// 卡不支持3.3V供电
#define   SD_ERR_OVER_CARDRANGE		0x05			// 操作超出卡容量范围

/* SD命令可能返回的错误码 */
#define   SD_ERR_CMD_RESPTYPE	 	0x10			// 命令类型错误
#define   SD_ERR_CMD_TIMEOUT     	0x11			// SD命令响应超时
#define   SD_ERR_CMD_RESP		0x12			// SD命令响应错误
			
/* 数据流错误码 */
#define   SD_ERR_DATA_CRC16      	0x20			// 数据流CRC16校验不通过
#define   SD_ERR_DATA_START_TOK		0x21			// 读单块或多块时，数据开始令牌不正确
#define	  SD_ERR_DATA_RESP		0x22			// 写单块或多块时，SD卡数据响应令牌不正确

/* 等待错误码 */
#define   SD_ERR_TIMEOUT_WAIT    	0x30			// 写或擦操作时，发生超时错误
#define   SD_ERR_TIMEOUT_READ    	0x31			// 读操作超时错误
#define	  SD_ERR_TIMEOUT_WRITE	 	0x32			// 写操作超时错误
#define   SD_ERR_TIMEOUT_ERASE   	0x33			// 擦除操作超时错误
#define	  SD_ERR_TIMEOUT_WAITIDLE 	0x34			// 初始化SD卡时，等待SD卡进入空闲状态超时错误

/* 写操作可能返回的错误码 */
#define	  SD_ERR_WRITE_BLK		0x40			// 写块数据错误
#define	  SD_ERR_WRITE_BLKNUMS          0x41			// 写多块时，想要写入的块与正确写入的块数不一致
#define   SD_ERR_WRITE_PROTECT		0x42			// 卡外壳的写保护开关打在写保护位置


extern uint8 halSD_Init(void);
extern uint8 hal_SendCmd( uint8 *param);
extern uint8 halSD_ActiveInit(void);
extern uint8 halSD_GetCardInfo( void);

extern uint8 hal_send_cmd_complex( uint8 cmd, uint8* param, uint8 resptype, uint8 *resp);

extern void halSD_PackParam( uint8 *parameter, uint32 value);
extern uint8 halSD_BlockCommand( uint8 cmd, uint8 resptype, uint32 parameter);
extern uint8 halSD_ResetSD( void);
extern uint8 halSD_ReadCSD( uint8 csdlen, uint8 *recbuf);
extern uint8 halSD_StopTransmission(void);
extern uint8 halSD_ReadCard_Status( uint8 len, uint8 *buffer);
extern uint8 halSD_SetBlockLen( uint32 length);
extern uint8 halSD_ReadSingleBlock( uint32 blockaddr);
extern uint8 halSD_ReadMultipleBlock( uint32 blockaddr);
extern uint8 halSD_WriteSingleBlock( uint32 blockaddr);
extern uint8 halSD_WriteMultipleBlock( uint32 blockaddr);
extern uint8 halSD_EraseStartBlock(uint32 startblock);
extern uint8 halSD_EraseEndBlock(uint32 startblock);
extern uint8 halSD_EraseSelectedBlock(void);
extern uint8 halSD_ReadOCR(uint8 ocrlen, uint8 *recbuf);
extern uint8 halSD_ReadSD_Status(uint8 sdslen, uint8 *recbuf);
extern uint8 halSD_ReadSCR(uint8 scrlen, uint8 *recbuf);
extern uint8 halSD_ReadRegister( uint32 len, uint8 *recbuf);
extern uint8 halSD_ReadBlockData(uint32 len, uint8 *recbuf);
extern uint8 halSD_WriteBlockData(uint8 bmulti, uint32 len, uint8 *sendbuf);
extern void halSD_StopMultiToken(void);
extern uint8 halSD_WaitBusy(uint8 waittype);
extern void halSD_SPIDelay(uint8 value);
#else
extern uint8 halWriteCmdSD(uint8 *CMD);

extern uint8 halSDInit( void);
extern uint8 halReadBlockSD(uint8 *CMD, uint8 *Buffer, uint16 Size);
extern uint8 halReadCID(uint8 *Buffer);
extern uint8 halReadCSD(uint8 *Buffer);
extern hal_SD_volume_info_t *halGetVolumeInfoSD( void);
extern uint8 halReadSectorSD(uint32 sector,uint8 *buffer);
extern void halGetDataSD(uint16 size,uint8 *buffer);
extern uint8 halWriteSectorSD(uint32 addr,uint8 *Buffer);
extern uint8 halStartReadSectorSD( uint32 sector);
extern void halGetData_LBA( uint32 lba, uint16 size, uint8* buffer);
extern void halGetSectorOffset( uint32 lba, uint16 offset);
extern void hal_LBA_close( void);

#endif  // SD_CMD_COMPLEX
#endif  // _SD_H_

#endif