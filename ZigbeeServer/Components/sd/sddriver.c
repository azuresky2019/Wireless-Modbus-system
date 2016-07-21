/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.c
** Last modified Date:	2005-3-10
** Last Version:		V2.0
** Descriptions:		SD/MMC卡读写模块: 物理层 用户API函数
**						Soft Packet of SD Card: user API funciton
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-10
** Version:				V2.0
** Descriptions:		增加了对MMC卡的支持,增加了对UCOS-II的支持,使该模块不仅能运行于前后台系统,还可运行
**						于UCOS-II上
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/

#include "mmc.h"
#include "SDCmd.h"
#include "sddriver.h"
#include "Sdconfig.h"

#include <string.h>

/* SD卡信息结构体变量 the information structure variable of SD Card */
sd_struct sds;			

/* 超时时间单位表(单位:0.000000001ns) timeout unit table */
const uint32 time_unit[8] = {1000000000,100000000,10000000,
							 1000000,100000,10000,1000,100};

/* 超时时间表 timeout value table */							 
const uint8 time_value[16] = {0,10,12,13,15,20,25,30,
                              35,40,45,50,55,60,70,80};
 
/* 超时时间因数表 timeout factor table */                              
const uint8 r2w_fator[8] = {1,2,4,8,16,32,64,128};                           
    
	/*       
	***************************************************************************************************
		
	     用户API函数:  初始化,读,写,擦 SD卡  User API Function: Initialize,read,write,erase SD Card 
				
	***************************************************************************************************
	*/
				
/*******************************************************************************************************************
** 函数名称: uint8 SD_Initialize()				Name:	  uint8 SD_Initialize()
** 功能描述: 初始化SD/MMC卡						Function: initialize SD/MMC card
** 输　  入: 无									Input:	  NULL
** 输    出: 0:   成功    >0:  错误码			Output:	  0:  right			>0:  error code
********************************************************************************************************************/
uint8 SD_Initialize(void)
{
	uint8 recbuf[4], ret;
    uint8 param[4] = {CMD8_PATTERN, CMD8_VHS, 0, 0};
	uint32 i = 0;

//    SD_HardWareInit();					    		/* 初始化读写SD卡的硬件条件 Initialize the hardware that access SD Card */

        mmc_init();
        
	sds.card_cap = SCSD;
	sds.card_type = CARDTYPE_SD;

	#if 0
    if (SD_ChkCard() != 1)							/* 检查卡是否插入 check weather card is inserted */
    {
    	ret = SD_ERR_NO_CARD;   
    	goto SD_ERR;
    } 
    #endif
         
    MMC_Enable();								/* 1. 置CS为低 assert CS */  
	SD_SPIDelay(25);								/* 2. 至少延时 74 clock delay more than 74 clock */
    MMC_Disable();								/* 3. 置CS为高 dessert CS */
    SD_SPIDelay(2);									/* 4. 延时2(8 clock) delay 2(8 clock) */

    ret = SD_ResetSD();								/* 5. 发出CMDO命令复位SD卡 send CMD0 command to reset sd card */
	if (ret != SD_NO_ERR)
        goto SD_ERR;									

 	ret = SD_ActiveInit();							/* 6. 激活卡进入初始化过程. active card initialize process */
 	if (ret != SD_NO_ERR)
 		goto SD_ERR;

   	ret = SD_ReadOCR(recbuf);  						/* 7. 读OCR寄存器,查询卡支持的电压值 read OCR register,get the supported voltage */
    if (ret != SD_NO_ERR)
        goto SD_ERR;
	
	if(recbuf[3] & 0x40)
		sds.card_cap = HCSD;
	else
		sds.card_cap = SCSD;

 //   SPI_ClkToMax();									/* 8. 设置SPI时钟到最大值 set SPI clock to maximum */

	if(sds.card_cap != HCSD)
	{
	    ret = SD_SetBlockLen(SD_BLOCKSIZE);				/* 9. 设置块的长度: 512Bytes Set the block length: 512Bytes */
	    if (ret != SD_NO_ERR)  
	        goto SD_ERR;
	}

//	SD_GetSDStatus();
    ret = SD_GetCardInfo();							/* 10. 读CSD寄存器,获取SD卡信息 read CSD register, get the information of SD card */    
	if (ret != SD_NO_ERR)
		goto SD_ERR;

	SD_EndSD();
	return SD_NO_ERR;								/* 初始化成功 initialize sucessfully */

SD_ERR:	
	SD_EndSD();
	return ret;
}

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadBlock()					Name:	  uint8 SD_ReadBlock()
** 功能描述: 从SD/MMC卡中读一个块					Function: read a single block from SD/MMC card
** 输　  入: uint32 blockaddr: 块地址				Input:    uint32 blockaddr: address of block
			 uint8 *recbuf   : 接收缓冲区,长度512Bytes	 	  uint8 *recbuf   : the buffer of receive,length is 512Bytes
** 输    出: 0:   成功    >0:  错误码				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadBlock(uint32 blockaddr, uint8 *recbuf)
{
	uint8 ret;
	
	if (blockaddr > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;				/* 操作超出卡容量范围 operate over the card range */

 	ret = SD_ReadSingleBlock(blockaddr);			/* 读单块命令 read single blocks command */						
 	if(ret != SD_NO_ERR)
 		return ret;
	
  	ret = SD_ReadBlockData(SD_BLOCKSIZE, recbuf);	/* 读出数据 read data from sd card */
	if(ret != SD_NO_ERR)
 		return ret;

	return ret;
}

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadMultiBlock()				Name:	  uint8 SD_ReadMultiBlock()
** 功能描述: 从SD/MMC卡中读多个块					Function: read multi blocks from SD/MMC card
** 输　  入: uint32 blockaddr: 块地址				Input:	  uint32 blockaddr: address of block
			 uint32 blocknum : 块数量						  uint32 blocknum : the numbers of block
			 uint8 *recbuf   : 接收缓冲区,每块512字节		  uint8 *recbuf   : the buffer of receive,each block length is 512Bytes
** 输    出: 0:   成功    >0:  错误码				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
uint8 SD_ReadMultiBlock(uint32 startblock, uint32 blocknum, uint8 *recbuf)
{
    uint32 i;
    uint8 ret;
    
	if((startblock + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;				/* 操作超出卡容量范围 operate over the card range */
	  
	ret = SD_ReadMultipleBlock(startblock);			/* 读多块命令 read multiple blocks command */
	if(ret != SD_NO_ERR)	
		return ret;
    
    for(i = 0; i < blocknum; i++)
    {												/* 读出数据 read data from SD/MMC card */
    	ret = SD_ReadBlockData(SD_BLOCKSIZE, recbuf);
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
    		return ret;
    }
 	 
 	ret = SD_StopTransmission();				    /* 结束数据传输 stop transmission operation */ 

	return ret; 
}
#endif

/********************************************************************************************************************
** 函数名称: uint8 SD_WriteBlock()					Name:	  uint8 SD_WriteBlock()
** 功能描述: 向SD/MMC卡中写入一个块					Function: write a block to SD/MMC card
** 输　  入: uint32 blockaddr: 块地址				Input: 	  uint32 blockaddr: address of block
			 uint8 *sendbuf  : 发送缓冲区,长度512Bytes	  	  uint8 *sendbuf  : the buffer of send,length is 512Bytes
** 输    出: 0:   成功    >0:  错误码				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteBlock(uint32 blockaddr, uint8 *sendbuf)
{
	uint8 ret,tmp[2];
	
	if (blockaddr > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* 操作超出卡容量范围 operate over the card range */

	ret = SD_WriteSingleBlock(blockaddr);						/* 写单块命令 write single block */
	if (ret != SD_NO_ERR)
		return ret;
	
	ret = SD_WriteBlockData(0, SD_BLOCKSIZE, sendbuf);			/* 写入数据 write data */
 	if (ret == SD_NO_ERR)										/* 读Card Status寄存器, 检查写入是否成功 */
 	{															/* read Card Status register to check write wheather sucessfully */
 		ret = SD_ReadCard_Status(tmp);
 		if (ret != SD_NO_ERR)
 		{
 			return ret;											/* 读寄存器失败 read register fail */
		}

 		if((tmp[0] != 0) || (tmp[1] != 0))
 		{
			ret = SD_ERR_WRITE_BLK; 			     			/* 响应指示写失败 response indicate write fail */
 		}
 	}
  
 	return ret;													/* 返回写入结果 return the result of writing */									
}

/**********************************************************************************************************************
** 函数名称: uint8 SD_WriteMultiBlock()				Name:	  uint8 SD_WriteMultiBlock()
** 功能描述: 向SD/MMC卡中写入多个块					Function: write multi blocks to SD/MMC card
** 输　  入: uint32 blockaddr: 块地址				Input:	  uint32 blockaddr: address of block
			 uint32 blocknum : 块数量						  uint32 blocknum : the numbers of block
			 uint8 *sendbuf  : 发送缓冲区每块512字节    	  uint8 *sendbuf  : the send buffer,each block length is 512Bytes
** 输    出: 0:   成功    >0:  错误码				Output:	  0:  right			>0:  error code
***********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
uint8 SD_WriteMultiBlock(uint32 startblock, uint32 blocknum, uint8 *sendbuf)
{
	uint32 i;
	uint8 ret,tmp[2];
	
	if ((startblock + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* 操作超出卡容量范围 operate over the card range */

	ret = SD_WriteMultipleBlock(startblock);						/* 写多块命令 write multiple blocks command */
	if (ret != SD_NO_ERR)
		return ret;
	
    for (i = 0; i < blocknum; i++)
    {															
 		ret = SD_WriteBlockData(1, SD_BLOCKSIZE, sendbuf);		/* 写入数据 write data */
 		if (ret == SD_NO_ERR)	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		else
 		{														/* 写失败 write fail */
			SD_StopTransmission();								/* 停止数据传输 stop data transmission */	
			SD_WaitBusy(SD_WAIT_WRITE);							/* 等待 waiting */
			return ret;
		}
	}
   
    SD_StopMultiToken();										/* 发送数据停止令牌 send data stop token */
      
    ret = SD_WaitBusy(SD_WAIT_WRITE);							/* 等待写入的完成 wait for finishing writing */
    if (ret != SD_NO_ERR)
    {
    	return SD_ERR_TIMEOUT_WRITE;
    }
    
    if (sds.card_type == CARDTYPE_SD)
    {
   		ret = SD_GetNumWRBlcoks(&i);							/* 读正确写入的块数 read the blocks that be written correctly */
   		if (ret != SD_NO_ERR)
   		{
   		  	return ret;
   		}
   		if(i != blocknum)
			ret =  SD_ERR_WRITE_BLKNUMS;						/* 正确写入块数错误 the blocks that be written correctly is error */
   	}
   	else
   	{
   	 	ret = SD_ReadCard_Status(tmp);
 		if (ret != SD_NO_ERR)
 		{
 			return ret;											/* 读寄存器失败 read register fail */
		}
 		if((tmp[0] != 0) || (tmp[1] != 0))
			ret = SD_ERR_WRITE_BLK; 			     			/* 响应指示写失败 response indicate write fail */
   	}
   	 	
	return ret;													/* 返回写入成功 return write sucessfully */			
}
#endif

/*********************************************************************************************************************
** 函数名称: uint8 SD_EraseBlock()					Name:	  uint8 SD_EraseBlock()
** 功能描述: 擦除SD/MMC卡中的块						Function: Erase the block of SD/MMC card
** 输　  入: uint32 startaddr: 起始地址				Input:    uint32 startaddr: start address
			 uint32 endaddr  : 终止地址						  uint32 endaddr  : end address
** 输    出: 0:   成功    >0:  错误码				Output:	  0:  right			>0:  error code
** 注    意: startaddr 和 endaddr 建议为 sds.erase_unit 的整数倍, 因为有的卡只能以 sds.erase_unit 为单位进行擦除
*********************************************************************************************************************/
#if SD_EraseBlock_EN
uint8 SD_EraseBlock(uint32 startaddr, uint32 blocknum)
{
	long tmp;
	uint8 ret;

	if ((startaddr + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* 操作超出卡容量范围 operate over the card range */

	tmp = blocknum - sds.erase_unit;
	while(tmp >= 0)
	{
		ret = SD_EraseStartBlock(startaddr);
		if (ret != SD_NO_ERR)
			return ret;
		
		ret = SD_EraseEndBlock(startaddr + sds.erase_unit - 1);
		if (ret != SD_NO_ERR)
			return ret;
				
		ret = SD_EraseSelectedBlock();
		if (ret != SD_NO_ERR)
			return ret;

		startaddr += sds.erase_unit;								/* 起始地址递增 */
		blocknum  -= sds.erase_unit;
		tmp = blocknum - sds.erase_unit;
	};

	if(blocknum > 0)											/* 擦除不够once_erase块 */
	{															/* erase blocks that numbers is not enough once_erase */
		ret = SD_EraseStartBlock(startaddr);
		if (ret != SD_NO_ERR)
			return ret;
		
		ret = SD_EraseEndBlock(startaddr + blocknum - 1);
		if (ret != SD_NO_ERR)
			return ret;
			
		ret = SD_EraseSelectedBlock();
		if (ret != SD_NO_ERR)
			return ret;
	}
	return SD_NO_ERR;											/* 返回擦除成功 return erase sucessfully */
}
#endif


	/*****************************************************************
	
	    		下面为子程序
	 
	*****************************************************************/
/**************************
*
*
*
**************************/
void SD_GetSDStatus(void)
{
//	uint32 tmp;
//	uint8 sdstatus[64], ret;
//
//	if(sds.card_cap != HCSD) return;
//
//	ret = SD_ReadSD_Status(64, sdstatus);
//	if(ret != SD_NO_ERR)	
//		return;
}

/*******************************************************************************************************************
** 函数名称: uint8 SD_GetCardInfo()				Name:	  uint8 SD_GetCardInfo()
** 功能描述: 获得SD/MMC卡的信息					Function: get the information of SD/MMC card
** 输　  入: uint8 cardtype: 卡类型				Input:    uint8 cardtype: card type	
** 输    出: 0:   成功    >0:  错误码			Output:	  0:  right			>0:  error code
*******************************************************************************************************************/
uint8 SD_GetCardInfo(void)
{
	uint32 tmp;
	uint8 csdbuf[16],ret;

	ret = SD_ReadCSD(16, csdbuf);	 								    		/* 读CSD寄存器    read CSD register */
	if (ret != SD_NO_ERR)	
		return ret;	
	
	SD_CalTimeout(csdbuf);														/* 计算超时时间值 calculate timeout value */
	
	if(sds.card_cap == HCSD)
	{	
		/* 计算块的最大长度  */														/* calculate the size of a sector */
		sds.block_len = 512;
		
		/* 计算卡中块的个数 */														/* calculate the sector numbers of the SD Card */
		sds.block_num = (((uint32)(csdbuf[7] & 0X3F) << 16) | ((uint32)csdbuf[8] << 8) | csdbuf[9] + 1) << 10;			/* (C_SIZE + 1) * 512K */
			 	  																																			
		/* 计算擦除的单位(单位: 块) */	
		sds.erase_unit = 8192;	    												/* 擦除单位(块) */ 
	}
	else
	{
		/* 计算块的最大长度  */														/* calculate the size of a sector */
		sds.block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);  			/* (2 ^ READ_BL_LEN) */
		
		/* 计算卡中块的个数 */														/* calculate the sector numbers of the SD Card */
		sds.block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
		      			 (csdbuf[C_SIZE_POS2] << 2) +
		 	 			((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;				/* (C_SIZE + 1)*/
			 	  															
		tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +   
		      ((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;				/* (C_SIZE_MULT + 2) */
	    	
	    /* 获得卡中块的数量 */														/* get the block numbers in card */
		sds.block_num = sds.block_num * (1 << tmp);									/* (C_SIZE + 1) * 2 ^ (C_SIZE_MULT + 2) */
																				
		/* 计算擦除的单位(单位: 块) */	
		if (sds.card_type == CARDTYPE_MMC)
		{					    
			tmp  = ((csdbuf[ERASE_GRP_SIZE_POS] & ERASE_GRP_SIZE_MSK) >> 2) + 1;  	/* (ERASE_GRP_SIZE + 1)  */ 
			
			/* (ERASE_GRP_SIZE + 1) * (ERASE_GRP_MULTI + 1) */
			tmp *= ((csdbuf[ERASE_GRP_MULTI_POS1] & ERASE_GRP_MULTI_MSK1) << 3) +
			       ((csdbuf[ERASE_GRP_MULTI_POS2] & ERASE_GRP_MULTI_MSK2) >> 5) + 1;	
		}
		else																		/*calculate the size of sector */
			tmp = ((csdbuf[SECTOR_SIZE_POS1] & SECTOR_SIZE_MSK1) << 1) +  			
		          ((csdbuf[SECTOR_SIZE_POS2] & SECTOR_SIZE_MSK2) >> 7) + 1;			/* SD: SECTOR_SIZE */
		
		sds.erase_unit = tmp;	    												/* 擦除单位(块) */ 
	}
	return SD_NO_ERR;															/* 返回执行成功 return perform sucessfully */
}

/*******************************************************************************************************************
** 函数名称: uint8 SD_CalTimeout()				Name:	  uint8 SD_CalTimeout()
** 功能描述: 计算读/写/擦超时时间				Function: calculate timeout of reading,writing,erasing
** 输　  入: uint8 *csdbuf : CSD寄存器内容		Input: 	  uint8 *csdbuf : CSD register content
** 输    出: 0:   成功    >0:  错误码			Output:	  0:  right			>0:  error code
*******************************************************************************************************************/
uint8 SD_CalTimeout(uint8 *csdbuf)
{
//	uint32 tmp;
//	uint8 time_u,time_v,fator;
	csdbuf = csdbuf;

	sds.timeout_read = READ_TIMEOUT_100MS;								/* 默认读超时为100ms */
	sds.timeout_write = WRITE_TIMEOUT_250MS;							/* 默认写超时为250ms */
	sds.timeout_erase = WRITE_TIMEOUT_250MS;

//	if(sds.card_cap != HCSD)
//	{
//		time_u = (csdbuf[TAAC_POS] & TAAC_MSK);								/* 读超时时间单位 read timeout unit */
//		time_v = (csdbuf[TAAC_POS] & NSAC_MSK) >> 3;						/* 读超时时间值   read timeout value */
//		fator = (csdbuf[R2WFACTOR_POS] & R2WFACTOR_MSK) >> 2;				/* 读超时时间因数 read timeout factor */
//		
//		if(time_v == 0)	return SD_ERR_CARD_PARAM;							/* 卡参数有错误 card parameter is error */
//		
//		tmp = SPI_CLOCK * time_value[time_v] / 10 / time_unit[time_u];		/* TACC * f (单位 unit: clock) */
//		tmp = tmp + csdbuf[NSAC_POS] * 100;									/* TACC * f + NSAC * 100 (单位 unit: clock) */
//		
//		/* 计算得到的超时值 the timeout value of being calculated */
//		sds.timeout_read = tmp;
//		sds.timeout_write = tmp * r2w_fator[fator];							/* (TACC * f + NSAC * 100) * R2WFACTOR (单位 unit:clock)*/
//		
//		if (sds.card_type == CARDTYPE_SD)
//		{
//			sds.timeout_read  = sds.timeout_read * 100 / 8;					/* 实际值为计算值的100倍 */
//			sds.timeout_write = sds.timeout_write * 100 / 8;
//			if (sds.timeout_read > READ_TIMEOUT_100MS)						/* 取计算值与默认值中的最小值 */
//				sds.timeout_read = READ_TIMEOUT_100MS;
//			
//			if (sds.timeout_write > WRITE_TIMEOUT_250MS)
//				sds.timeout_write = WRITE_TIMEOUT_250MS;
//		}
//		else
//		{
//			sds.timeout_read  = sds.timeout_read * 10 / 8;					/* 实际值为计算值的10倍 */
//			sds.timeout_write = sds.timeout_write * 10 / 8;
//		}
//		
//		sds.timeout_erase = sds.timeout_write;
//	}

	sds.timeout_erase = sds.timeout_write;
	return SD_NO_ERR;	
}

/*******************************************************************************************************************
** 函数名称: uint8 SD_ActiveInit()				Name:	  uint8 SD_ActiveInit()
** 功能描述: 激活卡,并获得卡型					Function: active card, and get the card type 
** 输　  入: 无								 	Input:    NULL
** 输    出: 0:   成功    >0:  错误码			Output:	  0:  right			>0:  error code
** 函数说明: 该命令不断重复发送到SD卡，直到响应R1的Bit0(Idle)位为0，表示SD卡内部初始化处理完成。
		     当响应的Idle位为0时，SD卡就完全进入SPI模式了。当然重复发送命令CMD1是有次数限制的，
		     最大次数为宏定义SD_IDLE_WAIT_MAX.
*******************************************************************************************************************/
uint8 SD_ActiveInit(void)
{
	uint8 param[4] = {CMD8_PATTERN, CMD8_VHS, 0, 0};
	uint8 resp[5], ret;
	uint32 i = 0;
		
	ret = SD_SendCmd(CMD8, param, CMD8_R, resp);
	if(ret != SD_NO_ERR)
 		return ret;

	if(resp[4] == 0x01) // version 2
	{
		if((resp[0] != CMD8_PATTERN) || (resp[1] != CMD8_VHS))
			return SD_ERR_UNKNOWN_CARD;

		i = 0;
		do
		{
			memset(param, 0, 4);
			ret = SD_SendCmd(CMD55, param, CMD55_R, resp);
		    if(ret != SD_NO_ERR)
		       	return ret;

			param[3] = 0x40;
		    ret = SD_SendCmd(ACMD41, param, ACMD41_R, resp);		/* 激活内部初始化命令 active card to initialize process internal */
		    if(ret != SD_NO_ERR)	
		    	return SD_ERR_UNKNOWN_CARD;
		} while((resp[0] != 0x00) && (i++ < 50));

		if(i >= 50)
			return SD_ERR_UNKNOWN_CARD;

		sds.card_type = CARDTYPE_SD;
		return SD_NO_ERR;
	}
	else if(resp[4] == 0x05) // version 1
	{
		memset(param, 0, 4);
		for(i = 0; i < 200; i++)
		{
			ret = SD_SendCmd(CMD55, param, CMD55_R, resp);
		    if(ret != SD_NO_ERR)
		       	continue;
	
		    ret = SD_SendCmd(ACMD41, param, ACMD41_R, resp);	
		    if((ret != SD_NO_ERR) && (resp[0] == 0))
				break;	
		};

		if(i >= 200) // maybe MMC card
		{
			i = 0;
			do 
			{
		        ret = SD_SendCmd(CMD1, param, CMD1_R, resp);
		        if(ret != SD_NO_ERR)
		       		return ret;
		        i ++;
		    }while (((resp[0] & MSK_IDLE) == MSK_IDLE) && (i <= SD_IDLE_WAIT_MAX));

		    if (i >= SD_IDLE_WAIT_MAX)
		        return SD_ERR_TIMEOUT_WAITIDLE;
			else
				sds.card_type = CARDTYPE_MMC;
		}

		return SD_NO_ERR;
	}
	else
	{
		return SD_ERR_UNKNOWN_CARD;
	}  
}

void SD_StartSD(void)
{
}

void SD_EndSD(void)
{
}