

#include "sdcmd.h"
#include "Sddriver.h"
#include "Sdconfig.h"
#include "mmc.h"

//#include "newUart.h"
/********************************************************************************************************************
** 函数名称: uint8 SD_SendCmd()						Name:	  uint8 SD_SendCmd()
** 功能描述: 向卡发送命令,并取得响应				Function: send command to the card,and get a response
** 输　  入: uint8 cmd	    : 命令字				Input:	  uint8 cmd	    : command byte	
			 uint8 *param	: 命令参数,长度为4字节			  uint8 *param	: command parameter,length is 4 bytes  
			 uint8 resptype : 响应类型						  uint8 resptype: response type
			 uint8 *resp	: 响应,长度为1-5字节			  uint8 *resp	: response,length is 1-5 bytes
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
uint8 SD_SendCmd(uint8 cmd, uint8 *param, uint8 resptype, uint8 *resp)
{
	int i;
	uint8 tmp,rlen;
    
	MMC_Enable();
   
    mmc_write_byte((cmd & 0x3F) | 0x40);				 /* 发送命令头和命令字 send command header and word */
    
//    UartTX_Send_String( "c", 1);
    for (i = 3; i >= 0; i--){
 //     UartTX_Send_String( "e", 1);
        mmc_write_byte(param[i]);						 /* 发送参数 send parameters */
    }
    
//    UartTX_Send_String( "d", 1);
	if(cmd == CMD8)
		mmc_write_byte(CMD8_CRC);
	else if(cmd == CMD0)
		mmc_write_byte(0x95);							 /* CRC校验码,只用于第1个命令 CRC,only used for the first command */
	else
		mmc_write_byte(0x00);
    
    rlen = 0;
//    UartTX_Send_String( "a", 1);
    switch (resptype)								 /* 根据不同的命令,得到不同的响应长度 */
    {												 /* according various command,get the various response length */
  		case R1:
   	 	case R1B: rlen = 1;  break;
       		 
    	case R2:  rlen = 2;	 break;
       		 
   		case R3:
		case R7:  rlen = 5;	 break;
       		 
    	default:  mmc_write_byte(0xFF);	
      		      MMC_Disable();						 
        	      return SD_ERR_CMD_RESPTYPE;		 /* 返回命令响应类型错误 return error of command response type */
    		      break;
    }
 //   UartTX_Send_String( "b", 1);
    i = 0;				
    do 												 /* 等待响应,响应的开始位为0 */
    {												 /* Wait for a response,a response is a start bit(zero) */ 
        tmp = mmc_read_byte();
        i++;
    }while (((tmp & 0x80) != 0) && (i < SD_CMD_TIMEOUT));
    
    if (i >= SD_CMD_TIMEOUT)
    {				
        MMC_Disable();
        return SD_ERR_CMD_TIMEOUT;					 /* 返回命令超时 return response timeout of command */
    }
    
    for (i = rlen - 1; i >= 0; i--)
    {
        resp[i] = tmp;
        tmp = mmc_read_byte();					 	 /* 循环的最后发送8clock  at the last recycle,clock out 8 clock */
    }

	MMC_Disable();

    return SD_NO_ERR;								 /* 返回执行成功 return perform sucessfully */
}

/********************************************************************************************************************
** 函数名称: void SD_PackParam()					Name:	  void SD_PackParam()
** 功能描述: 将32位的参数转为字节形式				Function: change 32bit parameter to bytes form 
** 输　  入: uint8 *parameter: 字节参数缓冲区		Input:	  uint8 *parameter: the buffer of bytes parameter
			 uint32 value    : 32位参数						  uint32 value    : 32bit parameter
** 输 　 出: 无										Output:	  NULL
*********************************************************************************************************************/
void SD_PackParam(uint8 *parameter, uint32 value)
{
    parameter[3] = (uint8)(value >> 24);
    parameter[2] = (uint8)(value >> 16);
    parameter[1] = (uint8)(value >> 8);
    parameter[0] = (uint8)(value);
}

/********************************************************************************************************************
** 函数名称: uint8 SD_BlockCommand()					Name:	  uint8 SD_BlockCommand()
** 功能描述: 块命令									Function: command about block operation
** 输　  入: uint8 cmd	     : 命令字				Input:	  uint8 cmd	      : command byte 
			 uint8 resptype  : 响应类型						  uint8 resptype  : response type
			 uint32 parameter: 块操作参数			 		  uint32 parameter: parameter of block operation
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_BlockCommand(uint8 cmd, uint8 resptype, uint32 parameter)
{
	uint8 param[4],resp,ret;
	
	if(sds.card_cap == SCSD)
		parameter <<= SD_BLOCKSIZE_NBITS;					 /* 调整地址:左移9位 adjust address: move 9 bits left */

	SD_PackParam(param, parameter);						 /* 将参数转化为字节形式 change the parameter to bytes form */	

	ret = SD_SendCmd(cmd, param, resptype, &resp);
	if (ret != SD_NO_ERR)
	   	 return ret;							 		 /* 结束数据传输失败 stop transmission operation fail */
	
	if (resp != 0)
		 return SD_ERR_CMD_RESP;		 				 /* 响应错误 response is error */
		 
	return SD_NO_ERR;
}

	/*
	************************************************
	
	 	下面为SD卡SPI命令

	************************************************
	*/
	
/********************************************************************************************************************
** 函数名称: uint8 SD_ResetSD()						Name:	  uint8 SD_ResetSD()
** 功能描述: 复位SD/MMC卡							Function: reset SD/MMC card
** 输　  入: 无										Input:	  NULL
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ResetSD(void)
{
	uint8 param[4] = {0,0,0,0},resp;
    return (SD_SendCmd(CMD0, param, CMD0_R, &resp));	/* 复位命令 command that reset card */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_CheckSDVer()					Name:	  uint8 SD_CheckSDVer()
** 功能描述: 检查是否Ver2.0							Function: check if it is ver2.0
** 输　  入: 无										Input:	  NULL
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
//uint8 SD_CheckSDVer(void)
//{
//	uint8 param[4] = {0xaa, 0x01, 0, 0}, resp;
//	
//    return (SD_SendCmd(CMD8, param, CMD8_R, &resp));
//}

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadCSD()						Name:	  uint8 SD_ReadCSD()
** 功能描述: 读SD/MMC卡的CSD寄存器					Function: read CSD register of SD/MMC card 
** 输　  入: uint8 csdlen  : 寄存器长度(固定为16)			  uint8 csdlen  : len of register (fixed,is 16)
			 uint8 *recbuf : 接收缓冲区					      uint8 *recbuf : recbuffer	
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadCSD(uint8 csdlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
  
    ret = SD_SendCmd(CMD9, param, CMD9_R, &resp);		/* 读CSD寄存器命令 command that read CSD register */
    if (ret != SD_NO_ERR) 									
        return ret;									
  
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* 响应错误 response is error */
    
	return (SD_ReadRegister(csdlen, recbuf));
}

/*******************************************************************************************************************
** 函数名称: uint8 SD_ReadCID()						Name:	  uint8 SD_ReadCID()
** 功能描述: 读SD卡的CID寄存器						Function: read CID register of sd card
** 输　  入: uint8 cidlen  : 寄存器长度(固定为16)			  uint8 cidlen  : len of register (fixed,is 16)
			 uint8 *recbuf : 接收缓冲区					      uint8 *recbuf : recbuffer	
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
#if SD_ReadCID_EN
uint8 SD_ReadCID(uint8 cidlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
 
    ret = SD_SendCmd(CMD10, param, CMD10_R, &resp);		/* 读CID寄存器命令 command that read CID register */
    if ( ret != SD_NO_ERR)
   		return ret;			  									
   
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* 响应错误 response is error */
      
  	return (SD_ReadRegister(cidlen, recbuf));
}
#endif

/********************************************************************************************************************
** 函数名称: uint8 SD_StopTransmission()				Name:	  uint8 SD_StopTransmission()
** 功能描述: 停止数据传输							Function: stop data transmission 
** 输　  入: 无								 		Input:    NULL
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_StopTransmission(void)
{
	uint8 param[4] = {0,0,0,0},resp;
	
 	return (SD_SendCmd(CMD12, param, CMD12_R, &resp));	/* 结束数据传输命令失败 stop transmission command fail */
}

/*********************************************************************************************************************
** 函数名称: uint8 SD_ReadCard_Status()				Name:	  uint8 SD_ReadCard_Status()
** 功能描述: 读SD/MMC卡的 Card Status 寄存器		Function: read Card Status register of SD/MMC card 
** 输　  入:
			 uint8 *recbuf : 接收缓冲区					      uint8 *recbuf : recbuffer
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
**********************************************************************************************************************/
uint8 SD_ReadCard_Status( uint8 *buffer)
{
    uint8 param[4] = {0,0,0,0};
    return (SD_SendCmd(CMD13, param, CMD13_R, buffer)); /* 读 Card Status 寄存器 */
    									 	 			/* read register of Card Status */
}


/********************************************************************************************************************
** 函数名称: uint8 SD_SetBlockLen()					Name:	  uint8 SD_SetBlockLen()
** 功能描述: 设置一个块的长度						Function: set a block len of card 
** 输　  入: uint32 length	: 块的长度值			Input:	  uint32 length	: the length of a block
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_SetBlockLen(uint32 length)
{
	uint8 param[4],resp,ret;
  
    SD_PackParam(param, length);					/* 将参数转化为字节形式 change the parameter to bytes form */											
    ret = SD_SendCmd(CMD16, param, CMD16_R, &resp);
    if (ret != SD_NO_ERR)
 		return ret;									/* 设置块的长度为length失败 set the length of block to length fail */

	if (resp != 0)
    	return SD_ERR_CMD_RESP;			   			/* 响应错误 response is error */
    
    return SD_NO_ERR; 								/* 返回执行成功 return perform sucessfully */			
}

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadSingleBlock()				Name:	  uint8 SD_ReadSingleBlock()
** 功能描述: 读单块命令								Function: read single block command
** 输　  入: uint32 blockaddr: 块地址				Input:	  uint32 blockaddr: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right	>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadSingleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD17, CMD17_R, blockaddr)); /* 读单块命令 command that read single block */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadMultipleBlock()			Name:	  uint8 SD_ReadMultipleBlock()
** 功能描述: 读多块命令								Function: read multiple block command 
** 输　  入: uint32 blockaddr: 块地址				Input:	  uint32 blockaddr: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadMultipleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD18, CMD18_R, blockaddr)); /* 读多块命令 command that read multiple block */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_WriteSingleBlock()				Name:	  uint8 SD_WriteSingleBlock()
** 功能描述: 写单块命令								Function: write single block command
** 输　  入: uint32 blockaddr: block address			Input:	  uint32 blockaddr: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteSingleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD24, CMD24_R, blockaddr)); /* 写单块命令 command that write single block */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_WriteMultipleBlock()			Name:	  uint8 SD_WriteMultipleBlock()
** 功能描述: 写多块命令								Function: write multiple block command
** 输　  入: uint32 blockaddr: 块地址				Input:	  uint32 blockaddr: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right	>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteMultipleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD25, CMD25_R, blockaddr)); /* 写多块命令 command that write multiple block */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_ProgramCSD()						Name:	  uint8 SD_ProgramCSD()
** 功能描述: 写CSD寄存器								Function: write CSD register
** 输　  入: uint8 *buff   		 : CSD寄存器内容		Input:	  uint8 *buff   	  : the content of CSD register	
			 uint8 len	  		 : CSD寄存器长度			  	  uint8 len			  : the length of CSD register
** 输 　 出: 0:   正确    >0:   错误码		  			Output:	  0:  right		>0:  error code
********************************************************************************************************************/
#if SD_ProgramCSD_EN
uint8 SD_ProgramCSD(uint8 len, uint8 *buff)
{
	uint8 param[4] = {0,0,0,0},resp,ret;
	
	if (len != 16) return SD_ERR_USER_PARAM;

	ret = SD_SendCmd(CMD27, param, CMD27_R, &resp); 	/* 发送写CSD寄存器命令 send command that write CSD */
	if (ret != SD_NO_ERR)
		return ret;
		        
    if (resp != 0)    
        return SD_ERR_CMD_RESP;
		
	buff[15] = (SD_GetCRC7(buff, 15) << 1) + 0x01;  	/* 计算CSD中的crc 位域 calculate crc field in CSD */
		
	return(SD_WriteBlockData(0, 16, buff));
}

/********************************************************************************************************************
** 函数名称: uint8 SD_GetCRC7()						Name:	  uint8 SD_GetCRC7()
** 功能描述: 计算CRC7								Function: calculate crc7
** 输　  入: uint8 *pSource: 数据					Input:    uint8 *pSource: data
			 uint16 len    : 数据长度						  uint16 len   : data length
** 输 　 出: CRC7码									Output:	  CRC7 code
*********************************************************************************************************************/
uint8 SD_GetCRC7(uint8 *pSource, uint16 len)
{
	uint8 i = 0, j;
	uint8 reg = 0;
	
	do
	{
	    for (j = 0; j < 8; j++)
	    {
			reg <<= 1;
			reg ^= ((((pSource[i] << j) ^ reg) & 0x80) ? 0x9 : 0);
	    }
	    
	    i++;
	    
	}while(i < len);
	
	return reg;
}	
#endif	

#if SD_EraseBlock_EN
/********************************************************************************************************************
** 函数名称: uint8 SD_EraseStartBlock()				Name:	  uint8 SD_EraseStartBlock()
** 功能描述: 设置块擦除起始地址						Function: select the start block address of erasing operation 
** 输　  入: uint32 startblock: 块地址				Input: 	  uint32 startblock	: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right	   >0:  error code
*********************************************************************************************************************/
uint8 SD_EraseStartBlock(uint32 startblock)
{
	if (sds.card_type == CARDTYPE_SD)
		return (SD_BlockCommand(CMD32, CMD32_R, startblock));	/* 发送擦除起始块地址 send the start block address of erasing operation */

	return (SD_BlockCommand(CMD35, CMD35_R, startblock));	/* 发送擦除起始块地址 send the start block address of erasing operation */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_EraseEndBlock()				Name:	  uint8 SD_EraseEndBlock()
** 功能描述: 设置块擦除终止地址						Function: select the end block address of erasing operation  
** 输　  入: uint32 endblock: 块地址					Input:	  uint32 Length	: block address
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right	   >0:  error code
*********************************************************************************************************************/
uint8 SD_EraseEndBlock(uint32 endblock)
{
	if (sds.card_type == CARDTYPE_SD)
		return (SD_BlockCommand(CMD33, CMD33_R, endblock));     /* 发送擦除终止块地址 send the end block address of erasing operation */

	return (SD_BlockCommand(CMD36, CMD36_R, endblock));     /* 发送擦除终止块地址 send the end block address of erasing operation */
}

/********************************************************************************************************************
** 函数名称: uint8 SD_EraseSelectedBlock()			Name:	  uint8 SD_EraseSelectedBlock()
** 功能描述: 擦除已选中的块							Function: erase block selected
** 输　  入: 无										Input:	  NULL
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_EraseSelectedBlock(void)
{
	uint8 param[4],resp,tmp;
	
	SD_PackParam(param, 0);
	
	tmp = SD_SendCmd(CMD38, param, CMD38_R, &resp);	 	    /* 擦除所选择的块  erase blocks selected */
	if (tmp != SD_NO_ERR)
		return tmp;							 	
	
	if (SD_WaitBusy(SD_WAIT_ERASE) != SD_NO_ERR)			/* 等待擦除完成 wait for finishing erasing */
		return SD_ERR_TIMEOUT_ERASE;
	return SD_NO_ERR;									
}	
#endif

/*********************************************************************************************************************
** 函数名称: uint8 SD_ReadOCR()						Name:	  uint8 SD_ReadOCR()
** 功能描述: 读操作条件寄存器OCR					Function: read OCR register of card
** 输　  入: 
			 uint8 *recbuf : 接收缓冲区					      uint8 *recbuf : recbuffer	
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
**********************************************************************************************************************/
uint8 SD_ReadOCR(uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp[5],tmp;

    tmp = SD_SendCmd(CMD58, param, CMD58_R, resp);		/* 读 OCR 寄存器命令 */
    if (tmp != SD_NO_ERR)								/* read OCR register command */
    	return tmp;		 										

	if (resp[4] != 0)
        return SD_ERR_CMD_RESP;			 				/* 响应错误 response is error */

    for (tmp = 0; tmp < 4; tmp++)
    	recbuf[tmp] = resp[tmp];					/* 复制OCR寄存器内容到接收缓冲区 */
    
    return SD_NO_ERR;
}



/*********************************************************************************************************************
** 函数名称: uint8 SD_ReadSD_Status()				     Name:	   uint8 SD_ReadSD_Status()
** 功能描述: 读SD卡的 SD_Status 寄存器				     Function: read SD_Status register of sd card 
** 输　  入: uint8 sdslen  		: 寄存器长度(固定为64)	 Input:    uint8 sdslen: len of register (fixed,is 64)
			 uint8 *recbuf 		: 接收缓冲区				       uint8 *recbuf: recbuffer	
** 输 　 出: 0:   正确    >0:   错误码		  			 Output:	  0:  right		>0:  error code
** 注    意: 只有SD卡才有SD Status 寄存器				 Note:     only SD card have SD Status Register
**********************************************************************************************************************/
#if SD_ReadSD_Status_EN
uint8 SD_ReadSD_Status(uint8 sdslen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp[2],ret;
    
    ret = SD_SendCmd(CMD55, param, CMD55_R, resp);			/* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR)
    	return ret;											/* command that the followed commnad is a specific application */
    												 
    if (resp[0] != 0)
        return SD_ERR_CMD_RESP;								/* 响应错误 response is error */
    
    ret = SD_SendCmd(ACMD13, param, ACMD13_R, resp);		/* 读 SD_Status 命令 */
    if (ret != SD_NO_ERR)
    	return ret;											/* command that read SD_Status register */
   												
    if ((resp[0] != 0) || (resp[1] != 0))
        return SD_ERR_CMD_RESP;								/* 响应错误 response is error */
        
	return (SD_ReadBlockData(sdslen, recbuf));				/* 读出寄存器内容 read the content of the register */
}
#endif

/*******************************************************************************************************************
** 函数名称: uint8 SD_ReadSCR()							Name:	  uint8 SD_ReadSCR()
** 功能描述: 读SD卡的 SCR 寄存器						Function: read SCR register of SD card 
** 输　  入: uint8 scrlen  		: 寄存器长度(固定为8) 	Input:    uint8 scrlen		 : len of register (fixed,is 8)
			 uint8 *recbuf 		: 接收缓冲区					  uint8 *recbuf		 : recieve buffer	
** 输 　 出: 0:   正确    >0:   错误码		  			Output:	  0:  right		>0:  error code
** 备	 注: MMC卡没有该寄存器							Note:	  MMC Card have not this register
********************************************************************************************************************/
#if SD_ReadSCR_EN
uint8 SD_ReadSCR(uint8 scrlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
    
    ret = SD_SendCmd(CMD55, param, CMD55_R, &resp);		/* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR)								/* command that the followed commnad is a specific application */
    	return ret;													
    												 
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* 响应错误 response is error */
    
    ret = SD_SendCmd(ACMD51, param, ACMD51_R, &resp);   /* 发送读 SD Status 命令*/
    if (ret != SD_NO_ERR)								/* command that read SD Status register */
   		return ret;													
				    															
    if (resp != 0)
        return SD_ERR_CMD_RESP;						 	/* 响应错误 response is error */
        
	return (SD_ReadBlockData(scrlen, recbuf));	 		/* 读出寄存器内容 read the content of the register */
}
#endif

/********************************************************************************************************************
** 函数名称: uint8 SD_GetNumWRBlcoks()				Name:	  uint8 SD_GetNumWRBlcoks()
** 功能描述: 得到正确写入的块数						Function: get the block numbers that written correctly
** 输　  入: uint32 *blocknum: 返回的块数			Input:	  uint32 blocknum	: the block numbers returned
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
** 注	 意: MMC卡没有该命令						Note:     MMC Card have no this command
*********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
uint8 SD_GetNumWRBlcoks(uint32 *blocknum)
{
    uint8 tmp[4] = {0,0,0,0},resp,ret;
  
    ret = SD_SendCmd(CMD55, tmp, CMD55_R, &resp);	  	 /* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR) 								 /* command that the followed commnad is a specific application */
    	return ret;
    	 
    if (resp != 0)
    	return SD_ERR_CMD_RESP;    	
 											
   	ret = SD_SendCmd(ACMD22, tmp, ACMD22_R, &resp);  	 /* 读取正确写入的块数命令 */
   	if (ret != SD_NO_ERR)								 /* command that read the numbers of block written correctly */
   		return ret;											    
   		 														
	if (resp != 0)
    	return SD_ERR_CMD_RESP;							 /* 响应错误 response is error */
    		
    ret = SD_ReadBlockData(4, tmp);						 /* 读块数 read the numbvers of block */
    if (ret != SD_NO_ERR)
    	return ret;
    	
    *blocknum = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];	
    													 /* 转换为32位 change to 32 bits */

	return SD_NO_ERR;    								 /* 返回执行成功 return perform sucessfully */		
}
#endif

		/*********************************************************
		
		    			下面为一些数据传输函数
		
		**********************************************************/

/********************************************************************************************************************
** 函数名称: uint8 SD_ReadRegister()				Name:	  uint8 SD_ReadRegister()
** 功能描述: 从SD卡读取数据							Function: read data from SD card
** 输　  入: uint32 len	  : 长度					Input:	  uint32 len   : length
			 uint8 *recbuf: 接收缓冲区					 	  uint8 *recbuf: receive buffer
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadRegister(uint32 len, uint8 *recbuf)
{	
	uint32 i = 0;
	uint8 resp;

    MMC_Enable();
    do{    											/* 等待数据起始令牌 wait for data start token */
        resp = mmc_read_byte();
    	i++;
    }while((resp == 0xFF) && (i < SD_READREG_TIMEOUT));    
    
    if (i >= SD_READREG_TIMEOUT)
    {
    	MMC_Disable();
    	return SD_ERR_TIMEOUT_READ;					/* 超时, 返回错误 timeout, return error */
  	}
  	
   	if (resp != SD_TOK_READ_STARTBLOCK)				
   	{												/* 不是收到数据起始令牌 not receive data start token */
		recbuf[0] = resp;							
		i = 1;										/* 还有len - 1个字节要接收 still len - 1 bytes will be received */
   	}
   	else
   		i = 0;										/* 收到数据起始令牌,还有len个字节要接收 received data start token,still len bytes will be received */
   	  	
    for (; i < len; i++)
   		recbuf[i] = mmc_read_byte();					/* 接收数据 receive data */
   		
    i = mmc_read_byte();								
    i = (i << 8) + mmc_read_byte();    				/* 读取16位CRC get 16-bit CRC */	
  
  
    mmc_write_byte(0xFF);								/* 返回之前发送8个clock  clock out 8 clk before return */
    MMC_Disable();	
    	
	return SD_NO_ERR;
}	

/*******************************************************************************************************************
** 函数名称: uint8 SD_ReadBlockData()			Name:	  uint8 SD_ReadBlockData()
** 功能描述: 从卡中读取数据块					Function: read block data from card
** 输　  入: uint32 len    : 长度				Input:	  uint32 len    : length
			 uint8 *recbuf : 接收缓冲区					  uint8 *recbuf : the buffer of receive
** 输 　 出: 0:   正确    >0:   错误码		  	Output:	  0:  right		>0:  error code
*******************************************************************************************************************/
uint8 SD_ReadBlockData(uint32 len, uint8 *recbuf)
{
    uint8 tmp;
    uint32 i = 0,timeout;
    
	timeout = sds.timeout_read;							/* 等待接收数据开始令牌最长时间 wait time that receive data start token */
    
    MMC_Enable();    
    do
    { 											    	/* 等待接收数据开始令牌0xFE  wait for receiving data start token 0xFE */
        tmp = mmc_read_byte();
        i++;
    }while((tmp == 0xFF) && (i < timeout));
	

	if (i >= timeout)
	{
		MMC_Disable();
       	return SD_ERR_TIMEOUT_READ;						/* 返回读超时错误码  return error timeout error code of reading */
	}
	
	if (tmp != SD_TOK_READ_STARTBLOCK)					/* 块读开始令牌错误 read start block token is error */
	{
		mmc_write_byte(0xFF);
		MMC_Disable();
		return SD_ERR_DATA_START_TOK;
	}
	
	for (i = 0; i < len; i++)
   		recbuf[i] = mmc_read_byte();						/* 接收数据 receive data */
   		
    i = mmc_read_byte();								
    i = (i << 8) + mmc_read_byte();    					/* 读取16位CRC get 16-bit CRC */	  

	mmc_write_byte(0xFF); 
	MMC_Disable();

  	return SD_NO_ERR;									/* 返回函数执行成功 return function perform sucessfully */
}

/*******************************************************************************************************************
** 函数名称: uint8 SD_WriteBlockData()				Name:	  uint8 SD_WriteBlockData()
** 功能描述: 向卡写数据块							Function: write block data to card
** 输　  入: uint8 bmulti  : 是否为多块操作1:是0:否 Input:	  uint8 bmulti   : multi blocks operate 1:Y 0:N 
			 uint32 len    : 长度						  	  uint32 len     : length
			 uint8 *sendbuf: 发送缓冲区					 	  uint8 *sendbuf : the buffer of send
** 输 　 出: 0:   正确    >0:   错误码		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
uint8 SD_WriteBlockData(uint8 bmulti, uint32 len, uint8 *sendbuf)
{
	uint16 i;
	uint8 tmp;

	MMC_Enable();
		
    mmc_write_byte(0xFF);								/* 开始发送数据之前发送8个clock clock out 8 clk before start */
    
    if (bmulti == 1)
        mmc_write_byte(SD_TOK_WRITE_STARTBLOCK_M);	/* 写多块开始令牌 start token of write multi blocks */
	else
		mmc_write_byte(SD_TOK_WRITE_STARTBLOCK);		/* 写单块开始令牌 start token of write single block */

	for (i = 0; i < len; i++)
        mmc_write_byte(sendbuf[i]);					/* 发送数据 send data */


	mmc_write_byte((i >> 8) & 0xFF);
	mmc_write_byte(i & 0xFF); 						/* 发送CRC16校验码 send CRC16 check code */
			    
	tmp = mmc_read_byte();
  	if ((tmp & SD_RESP_DATA_MSK) != SD_RESP_DATA_ACCETPTED)	
  	{		
   		mmc_write_byte(0xFF);							/* 返回之前发送8个clock  clock out 8 clk before return */
   		MMC_Disable();
   		return SD_ERR_DATA_RESP;					/* 数据响应错误 data response error */
    }
        
    MMC_Disable();
     		
    if (SD_WaitBusy(SD_WAIT_WRITE) != SD_NO_ERR)			
    	return SD_ERR_TIMEOUT_WRITE;				/* 写入超时 write time out */
    return SD_NO_ERR; 							/* 写入正确 write right */
 }

/*******************************************************************************************************************
** 函数名称: void SD_StopMultiToken()				Name:	  void SD_StopMultiToken(void)
** 功能描述: 发送多块写停止令牌						Function: send the token that stop multiple block write
** 输　  入: 无									    Input:	  NULL
** 输 　 出: 无										Output:	  NULL
********************************************************************************************************************/
void SD_StopMultiToken(void)
{
	MMC_Enable();
	
	mmc_write_byte(0xFF);								/* 先发送8个clock send 8 clock first */
	mmc_write_byte(SD_TOK_STOP_MULTI);				/* 发送停止数据传输令牌 send stop transmission token */
	mmc_read_byte();
	
    MMC_Disable();
}


/********************************************************************************************************************
** 函数名称: void SD_WaitBusy()						Name:	  void SD_WaitBusy()
** 功能描述: 查询SD卡是否处于忙状态					Function: poll SD Card wheather it is busy
** 输　  入: uint32 waittype: 超时类型				Input:	  uint32 timeout: time out type
** 输 　 出: 0: 未超时  >0: 错误码					Output:	  0: not time out   > 0: error code
*********************************************************************************************************************/
uint8 SD_WaitBusy(uint8 waittype)
{
    uint32 timeout, i = 0;
    uint8 tmp;
    
  	if (waittype == SD_WAIT_WRITE)
  		timeout = sds.timeout_write;				/* 等待类型为写操作 wait type is write operation */
  	else
  		timeout = sds.timeout_erase;   				/* 等待类型为擦除操作 wait type is erase operation */
    	
   
	MMC_Enable();
   	do
   	{ 												/* 等待忙结束 wait for being busy end */
        tmp = mmc_read_byte();
        i++;
    }while ((tmp != 0xFF) && (i < timeout));		/* 忙时收到的值为0 always receive 0 when card is busy */    


	if(i < timeout) 
		tmp = SD_NO_ERR;							/* 返回0,表示没超时 return 0 indicate that operation is not time out */
	else 
		tmp = SD_ERR_TIMEOUT_WAIT;					/* 返回错误码,表示超时 return error code indicate that operation is time out */

	mmc_write_byte(0xFF);
	MMC_Disable();								
	return tmp;										/* 返回执行结果 */
}

/********************************************************************************************************************
** 函数名称: void SD_SPIDelay()						Name:	  void SD_SPIDelay()
** 功能描述: SPI总线延时							Function: SPI Bus delay 
** 输　  入: uint8 value: 延时值,不超过255		    Input:	  uint8 value : delay value,do not beyond 255
** 输 　 出: 无										Output:	  NULL
*********************************************************************************************************************/
void SD_SPIDelay(uint8 value)
{
    uint8 i;

    for (i = 0; i < value; i++)
        mmc_write_byte(0xFF);						 	/* 发送0xFF clock out 0xFF */
}








