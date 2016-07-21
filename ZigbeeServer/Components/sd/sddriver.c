/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.c
** Last modified Date:	2005-3-10
** Last Version:		V2.0
** Descriptions:		SD/MMC����дģ��: ����� �û�API����
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
** Descriptions:		�����˶�MMC����֧��,�����˶�UCOS-II��֧��,ʹ��ģ�鲻����������ǰ��̨ϵͳ,��������
**						��UCOS-II��
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

/* SD����Ϣ�ṹ����� the information structure variable of SD Card */
sd_struct sds;			

/* ��ʱʱ�䵥λ��(��λ:0.000000001ns) timeout unit table */
const uint32 time_unit[8] = {1000000000,100000000,10000000,
							 1000000,100000,10000,1000,100};

/* ��ʱʱ��� timeout value table */							 
const uint8 time_value[16] = {0,10,12,13,15,20,25,30,
                              35,40,45,50,55,60,70,80};
 
/* ��ʱʱ�������� timeout factor table */                              
const uint8 r2w_fator[8] = {1,2,4,8,16,32,64,128};                           
    
	/*       
	***************************************************************************************************
		
	     �û�API����:  ��ʼ��,��,д,�� SD��  User API Function: Initialize,read,write,erase SD Card 
				
	***************************************************************************************************
	*/
				
/*******************************************************************************************************************
** ��������: uint8 SD_Initialize()				Name:	  uint8 SD_Initialize()
** ��������: ��ʼ��SD/MMC��						Function: initialize SD/MMC card
** �䡡  ��: ��									Input:	  NULL
** ��    ��: 0:   �ɹ�    >0:  ������			Output:	  0:  right			>0:  error code
********************************************************************************************************************/
uint8 SD_Initialize(void)
{
	uint8 recbuf[4], ret;
    uint8 param[4] = {CMD8_PATTERN, CMD8_VHS, 0, 0};
	uint32 i = 0;

//    SD_HardWareInit();					    		/* ��ʼ����дSD����Ӳ������ Initialize the hardware that access SD Card */

        mmc_init();
        
	sds.card_cap = SCSD;
	sds.card_type = CARDTYPE_SD;

	#if 0
    if (SD_ChkCard() != 1)							/* ��鿨�Ƿ���� check weather card is inserted */
    {
    	ret = SD_ERR_NO_CARD;   
    	goto SD_ERR;
    } 
    #endif
         
    MMC_Enable();								/* 1. ��CSΪ�� assert CS */  
	SD_SPIDelay(25);								/* 2. ������ʱ 74 clock delay more than 74 clock */
    MMC_Disable();								/* 3. ��CSΪ�� dessert CS */
    SD_SPIDelay(2);									/* 4. ��ʱ2(8 clock) delay 2(8 clock) */

    ret = SD_ResetSD();								/* 5. ����CMDO���λSD�� send CMD0 command to reset sd card */
	if (ret != SD_NO_ERR)
        goto SD_ERR;									

 	ret = SD_ActiveInit();							/* 6. ��������ʼ������. active card initialize process */
 	if (ret != SD_NO_ERR)
 		goto SD_ERR;

   	ret = SD_ReadOCR(recbuf);  						/* 7. ��OCR�Ĵ���,��ѯ��֧�ֵĵ�ѹֵ read OCR register,get the supported voltage */
    if (ret != SD_NO_ERR)
        goto SD_ERR;
	
	if(recbuf[3] & 0x40)
		sds.card_cap = HCSD;
	else
		sds.card_cap = SCSD;

 //   SPI_ClkToMax();									/* 8. ����SPIʱ�ӵ����ֵ set SPI clock to maximum */

	if(sds.card_cap != HCSD)
	{
	    ret = SD_SetBlockLen(SD_BLOCKSIZE);				/* 9. ���ÿ�ĳ���: 512Bytes Set the block length: 512Bytes */
	    if (ret != SD_NO_ERR)  
	        goto SD_ERR;
	}

//	SD_GetSDStatus();
    ret = SD_GetCardInfo();							/* 10. ��CSD�Ĵ���,��ȡSD����Ϣ read CSD register, get the information of SD card */    
	if (ret != SD_NO_ERR)
		goto SD_ERR;

	SD_EndSD();
	return SD_NO_ERR;								/* ��ʼ���ɹ� initialize sucessfully */

SD_ERR:	
	SD_EndSD();
	return ret;
}

/********************************************************************************************************************
** ��������: uint8 SD_ReadBlock()					Name:	  uint8 SD_ReadBlock()
** ��������: ��SD/MMC���ж�һ����					Function: read a single block from SD/MMC card
** �䡡  ��: uint32 blockaddr: ���ַ				Input:    uint32 blockaddr: address of block
			 uint8 *recbuf   : ���ջ�����,����512Bytes	 	  uint8 *recbuf   : the buffer of receive,length is 512Bytes
** ��    ��: 0:   �ɹ�    >0:  ������				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadBlock(uint32 blockaddr, uint8 *recbuf)
{
	uint8 ret;
	
	if (blockaddr > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;				/* ����������������Χ operate over the card range */

 	ret = SD_ReadSingleBlock(blockaddr);			/* ���������� read single blocks command */						
 	if(ret != SD_NO_ERR)
 		return ret;
	
  	ret = SD_ReadBlockData(SD_BLOCKSIZE, recbuf);	/* �������� read data from sd card */
	if(ret != SD_NO_ERR)
 		return ret;

	return ret;
}

/********************************************************************************************************************
** ��������: uint8 SD_ReadMultiBlock()				Name:	  uint8 SD_ReadMultiBlock()
** ��������: ��SD/MMC���ж������					Function: read multi blocks from SD/MMC card
** �䡡  ��: uint32 blockaddr: ���ַ				Input:	  uint32 blockaddr: address of block
			 uint32 blocknum : ������						  uint32 blocknum : the numbers of block
			 uint8 *recbuf   : ���ջ�����,ÿ��512�ֽ�		  uint8 *recbuf   : the buffer of receive,each block length is 512Bytes
** ��    ��: 0:   �ɹ�    >0:  ������				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
uint8 SD_ReadMultiBlock(uint32 startblock, uint32 blocknum, uint8 *recbuf)
{
    uint32 i;
    uint8 ret;
    
	if((startblock + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;				/* ����������������Χ operate over the card range */
	  
	ret = SD_ReadMultipleBlock(startblock);			/* ��������� read multiple blocks command */
	if(ret != SD_NO_ERR)	
		return ret;
    
    for(i = 0; i < blocknum; i++)
    {												/* �������� read data from SD/MMC card */
    	ret = SD_ReadBlockData(SD_BLOCKSIZE, recbuf);
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
    		return ret;
    }
 	 
 	ret = SD_StopTransmission();				    /* �������ݴ��� stop transmission operation */ 

	return ret; 
}
#endif

/********************************************************************************************************************
** ��������: uint8 SD_WriteBlock()					Name:	  uint8 SD_WriteBlock()
** ��������: ��SD/MMC����д��һ����					Function: write a block to SD/MMC card
** �䡡  ��: uint32 blockaddr: ���ַ				Input: 	  uint32 blockaddr: address of block
			 uint8 *sendbuf  : ���ͻ�����,����512Bytes	  	  uint8 *sendbuf  : the buffer of send,length is 512Bytes
** ��    ��: 0:   �ɹ�    >0:  ������				Output:	  0:  right			>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteBlock(uint32 blockaddr, uint8 *sendbuf)
{
	uint8 ret,tmp[2];
	
	if (blockaddr > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* ����������������Χ operate over the card range */

	ret = SD_WriteSingleBlock(blockaddr);						/* д�������� write single block */
	if (ret != SD_NO_ERR)
		return ret;
	
	ret = SD_WriteBlockData(0, SD_BLOCKSIZE, sendbuf);			/* д������ write data */
 	if (ret == SD_NO_ERR)										/* ��Card Status�Ĵ���, ���д���Ƿ�ɹ� */
 	{															/* read Card Status register to check write wheather sucessfully */
 		ret = SD_ReadCard_Status(tmp);
 		if (ret != SD_NO_ERR)
 		{
 			return ret;											/* ���Ĵ���ʧ�� read register fail */
		}

 		if((tmp[0] != 0) || (tmp[1] != 0))
 		{
			ret = SD_ERR_WRITE_BLK; 			     			/* ��Ӧָʾдʧ�� response indicate write fail */
 		}
 	}
  
 	return ret;													/* ����д���� return the result of writing */									
}

/**********************************************************************************************************************
** ��������: uint8 SD_WriteMultiBlock()				Name:	  uint8 SD_WriteMultiBlock()
** ��������: ��SD/MMC����д������					Function: write multi blocks to SD/MMC card
** �䡡  ��: uint32 blockaddr: ���ַ				Input:	  uint32 blockaddr: address of block
			 uint32 blocknum : ������						  uint32 blocknum : the numbers of block
			 uint8 *sendbuf  : ���ͻ�����ÿ��512�ֽ�    	  uint8 *sendbuf  : the send buffer,each block length is 512Bytes
** ��    ��: 0:   �ɹ�    >0:  ������				Output:	  0:  right			>0:  error code
***********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
uint8 SD_WriteMultiBlock(uint32 startblock, uint32 blocknum, uint8 *sendbuf)
{
	uint32 i;
	uint8 ret,tmp[2];
	
	if ((startblock + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* ����������������Χ operate over the card range */

	ret = SD_WriteMultipleBlock(startblock);						/* д������� write multiple blocks command */
	if (ret != SD_NO_ERR)
		return ret;
	
    for (i = 0; i < blocknum; i++)
    {															
 		ret = SD_WriteBlockData(1, SD_BLOCKSIZE, sendbuf);		/* д������ write data */
 		if (ret == SD_NO_ERR)	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		else
 		{														/* дʧ�� write fail */
			SD_StopTransmission();								/* ֹͣ���ݴ��� stop data transmission */	
			SD_WaitBusy(SD_WAIT_WRITE);							/* �ȴ� waiting */
			return ret;
		}
	}
   
    SD_StopMultiToken();										/* ��������ֹͣ���� send data stop token */
      
    ret = SD_WaitBusy(SD_WAIT_WRITE);							/* �ȴ�д������ wait for finishing writing */
    if (ret != SD_NO_ERR)
    {
    	return SD_ERR_TIMEOUT_WRITE;
    }
    
    if (sds.card_type == CARDTYPE_SD)
    {
   		ret = SD_GetNumWRBlcoks(&i);							/* ����ȷд��Ŀ��� read the blocks that be written correctly */
   		if (ret != SD_NO_ERR)
   		{
   		  	return ret;
   		}
   		if(i != blocknum)
			ret =  SD_ERR_WRITE_BLKNUMS;						/* ��ȷд��������� the blocks that be written correctly is error */
   	}
   	else
   	{
   	 	ret = SD_ReadCard_Status(tmp);
 		if (ret != SD_NO_ERR)
 		{
 			return ret;											/* ���Ĵ���ʧ�� read register fail */
		}
 		if((tmp[0] != 0) || (tmp[1] != 0))
			ret = SD_ERR_WRITE_BLK; 			     			/* ��Ӧָʾдʧ�� response indicate write fail */
   	}
   	 	
	return ret;													/* ����д��ɹ� return write sucessfully */			
}
#endif

/*********************************************************************************************************************
** ��������: uint8 SD_EraseBlock()					Name:	  uint8 SD_EraseBlock()
** ��������: ����SD/MMC���еĿ�						Function: Erase the block of SD/MMC card
** �䡡  ��: uint32 startaddr: ��ʼ��ַ				Input:    uint32 startaddr: start address
			 uint32 endaddr  : ��ֹ��ַ						  uint32 endaddr  : end address
** ��    ��: 0:   �ɹ�    >0:  ������				Output:	  0:  right			>0:  error code
** ע    ��: startaddr �� endaddr ����Ϊ sds.erase_unit ��������, ��Ϊ�еĿ�ֻ���� sds.erase_unit Ϊ��λ���в���
*********************************************************************************************************************/
#if SD_EraseBlock_EN
uint8 SD_EraseBlock(uint32 startaddr, uint32 blocknum)
{
	long tmp;
	uint8 ret;

	if ((startaddr + blocknum) > sds.block_num)	
		return SD_ERR_OVER_CARDRANGE;							/* ����������������Χ operate over the card range */

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

		startaddr += sds.erase_unit;								/* ��ʼ��ַ���� */
		blocknum  -= sds.erase_unit;
		tmp = blocknum - sds.erase_unit;
	};

	if(blocknum > 0)											/* ��������once_erase�� */
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
	return SD_NO_ERR;											/* ���ز����ɹ� return erase sucessfully */
}
#endif


	/*****************************************************************
	
	    		����Ϊ�ӳ���
	 
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
** ��������: uint8 SD_GetCardInfo()				Name:	  uint8 SD_GetCardInfo()
** ��������: ���SD/MMC������Ϣ					Function: get the information of SD/MMC card
** �䡡  ��: uint8 cardtype: ������				Input:    uint8 cardtype: card type	
** ��    ��: 0:   �ɹ�    >0:  ������			Output:	  0:  right			>0:  error code
*******************************************************************************************************************/
uint8 SD_GetCardInfo(void)
{
	uint32 tmp;
	uint8 csdbuf[16],ret;

	ret = SD_ReadCSD(16, csdbuf);	 								    		/* ��CSD�Ĵ���    read CSD register */
	if (ret != SD_NO_ERR)	
		return ret;	
	
	SD_CalTimeout(csdbuf);														/* ���㳬ʱʱ��ֵ calculate timeout value */
	
	if(sds.card_cap == HCSD)
	{	
		/* ��������󳤶�  */														/* calculate the size of a sector */
		sds.block_len = 512;
		
		/* ���㿨�п�ĸ��� */														/* calculate the sector numbers of the SD Card */
		sds.block_num = (((uint32)(csdbuf[7] & 0X3F) << 16) | ((uint32)csdbuf[8] << 8) | csdbuf[9] + 1) << 10;			/* (C_SIZE + 1) * 512K */
			 	  																																			
		/* ��������ĵ�λ(��λ: ��) */	
		sds.erase_unit = 8192;	    												/* ������λ(��) */ 
	}
	else
	{
		/* ��������󳤶�  */														/* calculate the size of a sector */
		sds.block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);  			/* (2 ^ READ_BL_LEN) */
		
		/* ���㿨�п�ĸ��� */														/* calculate the sector numbers of the SD Card */
		sds.block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
		      			 (csdbuf[C_SIZE_POS2] << 2) +
		 	 			((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;				/* (C_SIZE + 1)*/
			 	  															
		tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +   
		      ((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;				/* (C_SIZE_MULT + 2) */
	    	
	    /* ��ÿ��п������ */														/* get the block numbers in card */
		sds.block_num = sds.block_num * (1 << tmp);									/* (C_SIZE + 1) * 2 ^ (C_SIZE_MULT + 2) */
																				
		/* ��������ĵ�λ(��λ: ��) */	
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
		
		sds.erase_unit = tmp;	    												/* ������λ(��) */ 
	}
	return SD_NO_ERR;															/* ����ִ�гɹ� return perform sucessfully */
}

/*******************************************************************************************************************
** ��������: uint8 SD_CalTimeout()				Name:	  uint8 SD_CalTimeout()
** ��������: �����/д/����ʱʱ��				Function: calculate timeout of reading,writing,erasing
** �䡡  ��: uint8 *csdbuf : CSD�Ĵ�������		Input: 	  uint8 *csdbuf : CSD register content
** ��    ��: 0:   �ɹ�    >0:  ������			Output:	  0:  right			>0:  error code
*******************************************************************************************************************/
uint8 SD_CalTimeout(uint8 *csdbuf)
{
//	uint32 tmp;
//	uint8 time_u,time_v,fator;
	csdbuf = csdbuf;

	sds.timeout_read = READ_TIMEOUT_100MS;								/* Ĭ�϶���ʱΪ100ms */
	sds.timeout_write = WRITE_TIMEOUT_250MS;							/* Ĭ��д��ʱΪ250ms */
	sds.timeout_erase = WRITE_TIMEOUT_250MS;

//	if(sds.card_cap != HCSD)
//	{
//		time_u = (csdbuf[TAAC_POS] & TAAC_MSK);								/* ����ʱʱ�䵥λ read timeout unit */
//		time_v = (csdbuf[TAAC_POS] & NSAC_MSK) >> 3;						/* ����ʱʱ��ֵ   read timeout value */
//		fator = (csdbuf[R2WFACTOR_POS] & R2WFACTOR_MSK) >> 2;				/* ����ʱʱ������ read timeout factor */
//		
//		if(time_v == 0)	return SD_ERR_CARD_PARAM;							/* �������д��� card parameter is error */
//		
//		tmp = SPI_CLOCK * time_value[time_v] / 10 / time_unit[time_u];		/* TACC * f (��λ unit: clock) */
//		tmp = tmp + csdbuf[NSAC_POS] * 100;									/* TACC * f + NSAC * 100 (��λ unit: clock) */
//		
//		/* ����õ��ĳ�ʱֵ the timeout value of being calculated */
//		sds.timeout_read = tmp;
//		sds.timeout_write = tmp * r2w_fator[fator];							/* (TACC * f + NSAC * 100) * R2WFACTOR (��λ unit:clock)*/
//		
//		if (sds.card_type == CARDTYPE_SD)
//		{
//			sds.timeout_read  = sds.timeout_read * 100 / 8;					/* ʵ��ֵΪ����ֵ��100�� */
//			sds.timeout_write = sds.timeout_write * 100 / 8;
//			if (sds.timeout_read > READ_TIMEOUT_100MS)						/* ȡ����ֵ��Ĭ��ֵ�е���Сֵ */
//				sds.timeout_read = READ_TIMEOUT_100MS;
//			
//			if (sds.timeout_write > WRITE_TIMEOUT_250MS)
//				sds.timeout_write = WRITE_TIMEOUT_250MS;
//		}
//		else
//		{
//			sds.timeout_read  = sds.timeout_read * 10 / 8;					/* ʵ��ֵΪ����ֵ��10�� */
//			sds.timeout_write = sds.timeout_write * 10 / 8;
//		}
//		
//		sds.timeout_erase = sds.timeout_write;
//	}

	sds.timeout_erase = sds.timeout_write;
	return SD_NO_ERR;	
}

/*******************************************************************************************************************
** ��������: uint8 SD_ActiveInit()				Name:	  uint8 SD_ActiveInit()
** ��������: ���,����ÿ���					Function: active card, and get the card type 
** �䡡  ��: ��								 	Input:    NULL
** ��    ��: 0:   �ɹ�    >0:  ������			Output:	  0:  right			>0:  error code
** ����˵��: ��������ظ����͵�SD����ֱ����ӦR1��Bit0(Idle)λΪ0����ʾSD���ڲ���ʼ��������ɡ�
		     ����Ӧ��IdleλΪ0ʱ��SD������ȫ����SPIģʽ�ˡ���Ȼ�ظ���������CMD1���д������Ƶģ�
		     ������Ϊ�궨��SD_IDLE_WAIT_MAX.
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
		    ret = SD_SendCmd(ACMD41, param, ACMD41_R, resp);		/* �����ڲ���ʼ������ active card to initialize process internal */
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