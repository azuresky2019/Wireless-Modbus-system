

#include "sdcmd.h"
#include "Sddriver.h"
#include "Sdconfig.h"
#include "mmc.h"

//#include "newUart.h"
/********************************************************************************************************************
** ��������: uint8 SD_SendCmd()						Name:	  uint8 SD_SendCmd()
** ��������: �򿨷�������,��ȡ����Ӧ				Function: send command to the card,and get a response
** �䡡  ��: uint8 cmd	    : ������				Input:	  uint8 cmd	    : command byte	
			 uint8 *param	: �������,����Ϊ4�ֽ�			  uint8 *param	: command parameter,length is 4 bytes  
			 uint8 resptype : ��Ӧ����						  uint8 resptype: response type
			 uint8 *resp	: ��Ӧ,����Ϊ1-5�ֽ�			  uint8 *resp	: response,length is 1-5 bytes
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
uint8 SD_SendCmd(uint8 cmd, uint8 *param, uint8 resptype, uint8 *resp)
{
	int i;
	uint8 tmp,rlen;
    
	MMC_Enable();
   
    mmc_write_byte((cmd & 0x3F) | 0x40);				 /* ��������ͷ�������� send command header and word */
    
//    UartTX_Send_String( "c", 1);
    for (i = 3; i >= 0; i--){
 //     UartTX_Send_String( "e", 1);
        mmc_write_byte(param[i]);						 /* ���Ͳ��� send parameters */
    }
    
//    UartTX_Send_String( "d", 1);
	if(cmd == CMD8)
		mmc_write_byte(CMD8_CRC);
	else if(cmd == CMD0)
		mmc_write_byte(0x95);							 /* CRCУ����,ֻ���ڵ�1������ CRC,only used for the first command */
	else
		mmc_write_byte(0x00);
    
    rlen = 0;
//    UartTX_Send_String( "a", 1);
    switch (resptype)								 /* ���ݲ�ͬ������,�õ���ͬ����Ӧ���� */
    {												 /* according various command,get the various response length */
  		case R1:
   	 	case R1B: rlen = 1;  break;
       		 
    	case R2:  rlen = 2;	 break;
       		 
   		case R3:
		case R7:  rlen = 5;	 break;
       		 
    	default:  mmc_write_byte(0xFF);	
      		      MMC_Disable();						 
        	      return SD_ERR_CMD_RESPTYPE;		 /* ����������Ӧ���ʹ��� return error of command response type */
    		      break;
    }
 //   UartTX_Send_String( "b", 1);
    i = 0;				
    do 												 /* �ȴ���Ӧ,��Ӧ�Ŀ�ʼλΪ0 */
    {												 /* Wait for a response,a response is a start bit(zero) */ 
        tmp = mmc_read_byte();
        i++;
    }while (((tmp & 0x80) != 0) && (i < SD_CMD_TIMEOUT));
    
    if (i >= SD_CMD_TIMEOUT)
    {				
        MMC_Disable();
        return SD_ERR_CMD_TIMEOUT;					 /* �������ʱ return response timeout of command */
    }
    
    for (i = rlen - 1; i >= 0; i--)
    {
        resp[i] = tmp;
        tmp = mmc_read_byte();					 	 /* ѭ���������8clock  at the last recycle,clock out 8 clock */
    }

	MMC_Disable();

    return SD_NO_ERR;								 /* ����ִ�гɹ� return perform sucessfully */
}

/********************************************************************************************************************
** ��������: void SD_PackParam()					Name:	  void SD_PackParam()
** ��������: ��32λ�Ĳ���תΪ�ֽ���ʽ				Function: change 32bit parameter to bytes form 
** �䡡  ��: uint8 *parameter: �ֽڲ���������		Input:	  uint8 *parameter: the buffer of bytes parameter
			 uint32 value    : 32λ����						  uint32 value    : 32bit parameter
** �� �� ��: ��										Output:	  NULL
*********************************************************************************************************************/
void SD_PackParam(uint8 *parameter, uint32 value)
{
    parameter[3] = (uint8)(value >> 24);
    parameter[2] = (uint8)(value >> 16);
    parameter[1] = (uint8)(value >> 8);
    parameter[0] = (uint8)(value);
}

/********************************************************************************************************************
** ��������: uint8 SD_BlockCommand()					Name:	  uint8 SD_BlockCommand()
** ��������: ������									Function: command about block operation
** �䡡  ��: uint8 cmd	     : ������				Input:	  uint8 cmd	      : command byte 
			 uint8 resptype  : ��Ӧ����						  uint8 resptype  : response type
			 uint32 parameter: ���������			 		  uint32 parameter: parameter of block operation
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_BlockCommand(uint8 cmd, uint8 resptype, uint32 parameter)
{
	uint8 param[4],resp,ret;
	
	if(sds.card_cap == SCSD)
		parameter <<= SD_BLOCKSIZE_NBITS;					 /* ������ַ:����9λ adjust address: move 9 bits left */

	SD_PackParam(param, parameter);						 /* ������ת��Ϊ�ֽ���ʽ change the parameter to bytes form */	

	ret = SD_SendCmd(cmd, param, resptype, &resp);
	if (ret != SD_NO_ERR)
	   	 return ret;							 		 /* �������ݴ���ʧ�� stop transmission operation fail */
	
	if (resp != 0)
		 return SD_ERR_CMD_RESP;		 				 /* ��Ӧ���� response is error */
		 
	return SD_NO_ERR;
}

	/*
	************************************************
	
	 	����ΪSD��SPI����

	************************************************
	*/
	
/********************************************************************************************************************
** ��������: uint8 SD_ResetSD()						Name:	  uint8 SD_ResetSD()
** ��������: ��λSD/MMC��							Function: reset SD/MMC card
** �䡡  ��: ��										Input:	  NULL
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ResetSD(void)
{
	uint8 param[4] = {0,0,0,0},resp;
    return (SD_SendCmd(CMD0, param, CMD0_R, &resp));	/* ��λ���� command that reset card */
}

/********************************************************************************************************************
** ��������: uint8 SD_CheckSDVer()					Name:	  uint8 SD_CheckSDVer()
** ��������: ����Ƿ�Ver2.0							Function: check if it is ver2.0
** �䡡  ��: ��										Input:	  NULL
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
//uint8 SD_CheckSDVer(void)
//{
//	uint8 param[4] = {0xaa, 0x01, 0, 0}, resp;
//	
//    return (SD_SendCmd(CMD8, param, CMD8_R, &resp));
//}

/********************************************************************************************************************
** ��������: uint8 SD_ReadCSD()						Name:	  uint8 SD_ReadCSD()
** ��������: ��SD/MMC����CSD�Ĵ���					Function: read CSD register of SD/MMC card 
** �䡡  ��: uint8 csdlen  : �Ĵ�������(�̶�Ϊ16)			  uint8 csdlen  : len of register (fixed,is 16)
			 uint8 *recbuf : ���ջ�����					      uint8 *recbuf : recbuffer	
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadCSD(uint8 csdlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
  
    ret = SD_SendCmd(CMD9, param, CMD9_R, &resp);		/* ��CSD�Ĵ������� command that read CSD register */
    if (ret != SD_NO_ERR) 									
        return ret;									
  
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* ��Ӧ���� response is error */
    
	return (SD_ReadRegister(csdlen, recbuf));
}

/*******************************************************************************************************************
** ��������: uint8 SD_ReadCID()						Name:	  uint8 SD_ReadCID()
** ��������: ��SD����CID�Ĵ���						Function: read CID register of sd card
** �䡡  ��: uint8 cidlen  : �Ĵ�������(�̶�Ϊ16)			  uint8 cidlen  : len of register (fixed,is 16)
			 uint8 *recbuf : ���ջ�����					      uint8 *recbuf : recbuffer	
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
#if SD_ReadCID_EN
uint8 SD_ReadCID(uint8 cidlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
 
    ret = SD_SendCmd(CMD10, param, CMD10_R, &resp);		/* ��CID�Ĵ������� command that read CID register */
    if ( ret != SD_NO_ERR)
   		return ret;			  									
   
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* ��Ӧ���� response is error */
      
  	return (SD_ReadRegister(cidlen, recbuf));
}
#endif

/********************************************************************************************************************
** ��������: uint8 SD_StopTransmission()				Name:	  uint8 SD_StopTransmission()
** ��������: ֹͣ���ݴ���							Function: stop data transmission 
** �䡡  ��: ��								 		Input:    NULL
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_StopTransmission(void)
{
	uint8 param[4] = {0,0,0,0},resp;
	
 	return (SD_SendCmd(CMD12, param, CMD12_R, &resp));	/* �������ݴ�������ʧ�� stop transmission command fail */
}

/*********************************************************************************************************************
** ��������: uint8 SD_ReadCard_Status()				Name:	  uint8 SD_ReadCard_Status()
** ��������: ��SD/MMC���� Card Status �Ĵ���		Function: read Card Status register of SD/MMC card 
** �䡡  ��:
			 uint8 *recbuf : ���ջ�����					      uint8 *recbuf : recbuffer
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
**********************************************************************************************************************/
uint8 SD_ReadCard_Status( uint8 *buffer)
{
    uint8 param[4] = {0,0,0,0};
    return (SD_SendCmd(CMD13, param, CMD13_R, buffer)); /* �� Card Status �Ĵ��� */
    									 	 			/* read register of Card Status */
}


/********************************************************************************************************************
** ��������: uint8 SD_SetBlockLen()					Name:	  uint8 SD_SetBlockLen()
** ��������: ����һ����ĳ���						Function: set a block len of card 
** �䡡  ��: uint32 length	: ��ĳ���ֵ			Input:	  uint32 length	: the length of a block
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_SetBlockLen(uint32 length)
{
	uint8 param[4],resp,ret;
  
    SD_PackParam(param, length);					/* ������ת��Ϊ�ֽ���ʽ change the parameter to bytes form */											
    ret = SD_SendCmd(CMD16, param, CMD16_R, &resp);
    if (ret != SD_NO_ERR)
 		return ret;									/* ���ÿ�ĳ���Ϊlengthʧ�� set the length of block to length fail */

	if (resp != 0)
    	return SD_ERR_CMD_RESP;			   			/* ��Ӧ���� response is error */
    
    return SD_NO_ERR; 								/* ����ִ�гɹ� return perform sucessfully */			
}

/********************************************************************************************************************
** ��������: uint8 SD_ReadSingleBlock()				Name:	  uint8 SD_ReadSingleBlock()
** ��������: ����������								Function: read single block command
** �䡡  ��: uint32 blockaddr: ���ַ				Input:	  uint32 blockaddr: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right	>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadSingleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD17, CMD17_R, blockaddr)); /* ���������� command that read single block */
}

/********************************************************************************************************************
** ��������: uint8 SD_ReadMultipleBlock()			Name:	  uint8 SD_ReadMultipleBlock()
** ��������: ���������								Function: read multiple block command 
** �䡡  ��: uint32 blockaddr: ���ַ				Input:	  uint32 blockaddr: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadMultipleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD18, CMD18_R, blockaddr)); /* ��������� command that read multiple block */
}

/********************************************************************************************************************
** ��������: uint8 SD_WriteSingleBlock()				Name:	  uint8 SD_WriteSingleBlock()
** ��������: д��������								Function: write single block command
** �䡡  ��: uint32 blockaddr: block address			Input:	  uint32 blockaddr: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteSingleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD24, CMD24_R, blockaddr)); /* д�������� command that write single block */
}

/********************************************************************************************************************
** ��������: uint8 SD_WriteMultipleBlock()			Name:	  uint8 SD_WriteMultipleBlock()
** ��������: д�������								Function: write multiple block command
** �䡡  ��: uint32 blockaddr: ���ַ				Input:	  uint32 blockaddr: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right	>0:  error code
*********************************************************************************************************************/
uint8 SD_WriteMultipleBlock(uint32 blockaddr)
{
	return (SD_BlockCommand(CMD25, CMD25_R, blockaddr)); /* д������� command that write multiple block */
}

/********************************************************************************************************************
** ��������: uint8 SD_ProgramCSD()						Name:	  uint8 SD_ProgramCSD()
** ��������: дCSD�Ĵ���								Function: write CSD register
** �䡡  ��: uint8 *buff   		 : CSD�Ĵ�������		Input:	  uint8 *buff   	  : the content of CSD register	
			 uint8 len	  		 : CSD�Ĵ�������			  	  uint8 len			  : the length of CSD register
** �� �� ��: 0:   ��ȷ    >0:   ������		  			Output:	  0:  right		>0:  error code
********************************************************************************************************************/
#if SD_ProgramCSD_EN
uint8 SD_ProgramCSD(uint8 len, uint8 *buff)
{
	uint8 param[4] = {0,0,0,0},resp,ret;
	
	if (len != 16) return SD_ERR_USER_PARAM;

	ret = SD_SendCmd(CMD27, param, CMD27_R, &resp); 	/* ����дCSD�Ĵ������� send command that write CSD */
	if (ret != SD_NO_ERR)
		return ret;
		        
    if (resp != 0)    
        return SD_ERR_CMD_RESP;
		
	buff[15] = (SD_GetCRC7(buff, 15) << 1) + 0x01;  	/* ����CSD�е�crc λ�� calculate crc field in CSD */
		
	return(SD_WriteBlockData(0, 16, buff));
}

/********************************************************************************************************************
** ��������: uint8 SD_GetCRC7()						Name:	  uint8 SD_GetCRC7()
** ��������: ����CRC7								Function: calculate crc7
** �䡡  ��: uint8 *pSource: ����					Input:    uint8 *pSource: data
			 uint16 len    : ���ݳ���						  uint16 len   : data length
** �� �� ��: CRC7��									Output:	  CRC7 code
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
** ��������: uint8 SD_EraseStartBlock()				Name:	  uint8 SD_EraseStartBlock()
** ��������: ���ÿ������ʼ��ַ						Function: select the start block address of erasing operation 
** �䡡  ��: uint32 startblock: ���ַ				Input: 	  uint32 startblock	: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right	   >0:  error code
*********************************************************************************************************************/
uint8 SD_EraseStartBlock(uint32 startblock)
{
	if (sds.card_type == CARDTYPE_SD)
		return (SD_BlockCommand(CMD32, CMD32_R, startblock));	/* ���Ͳ�����ʼ���ַ send the start block address of erasing operation */

	return (SD_BlockCommand(CMD35, CMD35_R, startblock));	/* ���Ͳ�����ʼ���ַ send the start block address of erasing operation */
}

/********************************************************************************************************************
** ��������: uint8 SD_EraseEndBlock()				Name:	  uint8 SD_EraseEndBlock()
** ��������: ���ÿ������ֹ��ַ						Function: select the end block address of erasing operation  
** �䡡  ��: uint32 endblock: ���ַ					Input:	  uint32 Length	: block address
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right	   >0:  error code
*********************************************************************************************************************/
uint8 SD_EraseEndBlock(uint32 endblock)
{
	if (sds.card_type == CARDTYPE_SD)
		return (SD_BlockCommand(CMD33, CMD33_R, endblock));     /* ���Ͳ�����ֹ���ַ send the end block address of erasing operation */

	return (SD_BlockCommand(CMD36, CMD36_R, endblock));     /* ���Ͳ�����ֹ���ַ send the end block address of erasing operation */
}

/********************************************************************************************************************
** ��������: uint8 SD_EraseSelectedBlock()			Name:	  uint8 SD_EraseSelectedBlock()
** ��������: ������ѡ�еĿ�							Function: erase block selected
** �䡡  ��: ��										Input:	  NULL
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_EraseSelectedBlock(void)
{
	uint8 param[4],resp,tmp;
	
	SD_PackParam(param, 0);
	
	tmp = SD_SendCmd(CMD38, param, CMD38_R, &resp);	 	    /* ������ѡ��Ŀ�  erase blocks selected */
	if (tmp != SD_NO_ERR)
		return tmp;							 	
	
	if (SD_WaitBusy(SD_WAIT_ERASE) != SD_NO_ERR)			/* �ȴ�������� wait for finishing erasing */
		return SD_ERR_TIMEOUT_ERASE;
	return SD_NO_ERR;									
}	
#endif

/*********************************************************************************************************************
** ��������: uint8 SD_ReadOCR()						Name:	  uint8 SD_ReadOCR()
** ��������: �����������Ĵ���OCR					Function: read OCR register of card
** �䡡  ��: 
			 uint8 *recbuf : ���ջ�����					      uint8 *recbuf : recbuffer	
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
**********************************************************************************************************************/
uint8 SD_ReadOCR(uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp[5],tmp;

    tmp = SD_SendCmd(CMD58, param, CMD58_R, resp);		/* �� OCR �Ĵ������� */
    if (tmp != SD_NO_ERR)								/* read OCR register command */
    	return tmp;		 										

	if (resp[4] != 0)
        return SD_ERR_CMD_RESP;			 				/* ��Ӧ���� response is error */

    for (tmp = 0; tmp < 4; tmp++)
    	recbuf[tmp] = resp[tmp];					/* ����OCR�Ĵ������ݵ����ջ����� */
    
    return SD_NO_ERR;
}



/*********************************************************************************************************************
** ��������: uint8 SD_ReadSD_Status()				     Name:	   uint8 SD_ReadSD_Status()
** ��������: ��SD���� SD_Status �Ĵ���				     Function: read SD_Status register of sd card 
** �䡡  ��: uint8 sdslen  		: �Ĵ�������(�̶�Ϊ64)	 Input:    uint8 sdslen: len of register (fixed,is 64)
			 uint8 *recbuf 		: ���ջ�����				       uint8 *recbuf: recbuffer	
** �� �� ��: 0:   ��ȷ    >0:   ������		  			 Output:	  0:  right		>0:  error code
** ע    ��: ֻ��SD������SD Status �Ĵ���				 Note:     only SD card have SD Status Register
**********************************************************************************************************************/
#if SD_ReadSD_Status_EN
uint8 SD_ReadSD_Status(uint8 sdslen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp[2],ret;
    
    ret = SD_SendCmd(CMD55, param, CMD55_R, resp);			/* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR)
    	return ret;											/* command that the followed commnad is a specific application */
    												 
    if (resp[0] != 0)
        return SD_ERR_CMD_RESP;								/* ��Ӧ���� response is error */
    
    ret = SD_SendCmd(ACMD13, param, ACMD13_R, resp);		/* �� SD_Status ���� */
    if (ret != SD_NO_ERR)
    	return ret;											/* command that read SD_Status register */
   												
    if ((resp[0] != 0) || (resp[1] != 0))
        return SD_ERR_CMD_RESP;								/* ��Ӧ���� response is error */
        
	return (SD_ReadBlockData(sdslen, recbuf));				/* �����Ĵ������� read the content of the register */
}
#endif

/*******************************************************************************************************************
** ��������: uint8 SD_ReadSCR()							Name:	  uint8 SD_ReadSCR()
** ��������: ��SD���� SCR �Ĵ���						Function: read SCR register of SD card 
** �䡡  ��: uint8 scrlen  		: �Ĵ�������(�̶�Ϊ8) 	Input:    uint8 scrlen		 : len of register (fixed,is 8)
			 uint8 *recbuf 		: ���ջ�����					  uint8 *recbuf		 : recieve buffer	
** �� �� ��: 0:   ��ȷ    >0:   ������		  			Output:	  0:  right		>0:  error code
** ��	 ע: MMC��û�иüĴ���							Note:	  MMC Card have not this register
********************************************************************************************************************/
#if SD_ReadSCR_EN
uint8 SD_ReadSCR(uint8 scrlen, uint8 *recbuf)
{
    uint8 param[4] = {0,0,0,0},resp,ret;
    
    ret = SD_SendCmd(CMD55, param, CMD55_R, &resp);		/* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR)								/* command that the followed commnad is a specific application */
    	return ret;													
    												 
    if (resp != 0)
        return SD_ERR_CMD_RESP;							/* ��Ӧ���� response is error */
    
    ret = SD_SendCmd(ACMD51, param, ACMD51_R, &resp);   /* ���Ͷ� SD Status ����*/
    if (ret != SD_NO_ERR)								/* command that read SD Status register */
   		return ret;													
				    															
    if (resp != 0)
        return SD_ERR_CMD_RESP;						 	/* ��Ӧ���� response is error */
        
	return (SD_ReadBlockData(scrlen, recbuf));	 		/* �����Ĵ������� read the content of the register */
}
#endif

/********************************************************************************************************************
** ��������: uint8 SD_GetNumWRBlcoks()				Name:	  uint8 SD_GetNumWRBlcoks()
** ��������: �õ���ȷд��Ŀ���						Function: get the block numbers that written correctly
** �䡡  ��: uint32 *blocknum: ���صĿ���			Input:	  uint32 blocknum	: the block numbers returned
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
** ע	 ��: MMC��û�и�����						Note:     MMC Card have no this command
*********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
uint8 SD_GetNumWRBlcoks(uint32 *blocknum)
{
    uint8 tmp[4] = {0,0,0,0},resp,ret;
  
    ret = SD_SendCmd(CMD55, tmp, CMD55_R, &resp);	  	 /* ��������Ϊһ��Ӧ������ */
    if (ret != SD_NO_ERR) 								 /* command that the followed commnad is a specific application */
    	return ret;
    	 
    if (resp != 0)
    	return SD_ERR_CMD_RESP;    	
 											
   	ret = SD_SendCmd(ACMD22, tmp, ACMD22_R, &resp);  	 /* ��ȡ��ȷд��Ŀ������� */
   	if (ret != SD_NO_ERR)								 /* command that read the numbers of block written correctly */
   		return ret;											    
   		 														
	if (resp != 0)
    	return SD_ERR_CMD_RESP;							 /* ��Ӧ���� response is error */
    		
    ret = SD_ReadBlockData(4, tmp);						 /* ������ read the numbvers of block */
    if (ret != SD_NO_ERR)
    	return ret;
    	
    *blocknum = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];	
    													 /* ת��Ϊ32λ change to 32 bits */

	return SD_NO_ERR;    								 /* ����ִ�гɹ� return perform sucessfully */		
}
#endif

		/*********************************************************
		
		    			����ΪһЩ���ݴ��亯��
		
		**********************************************************/

/********************************************************************************************************************
** ��������: uint8 SD_ReadRegister()				Name:	  uint8 SD_ReadRegister()
** ��������: ��SD����ȡ����							Function: read data from SD card
** �䡡  ��: uint32 len	  : ����					Input:	  uint32 len   : length
			 uint8 *recbuf: ���ջ�����					 	  uint8 *recbuf: receive buffer
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
*********************************************************************************************************************/
uint8 SD_ReadRegister(uint32 len, uint8 *recbuf)
{	
	uint32 i = 0;
	uint8 resp;

    MMC_Enable();
    do{    											/* �ȴ�������ʼ���� wait for data start token */
        resp = mmc_read_byte();
    	i++;
    }while((resp == 0xFF) && (i < SD_READREG_TIMEOUT));    
    
    if (i >= SD_READREG_TIMEOUT)
    {
    	MMC_Disable();
    	return SD_ERR_TIMEOUT_READ;					/* ��ʱ, ���ش��� timeout, return error */
  	}
  	
   	if (resp != SD_TOK_READ_STARTBLOCK)				
   	{												/* �����յ�������ʼ���� not receive data start token */
		recbuf[0] = resp;							
		i = 1;										/* ����len - 1���ֽ�Ҫ���� still len - 1 bytes will be received */
   	}
   	else
   		i = 0;										/* �յ�������ʼ����,����len���ֽ�Ҫ���� received data start token,still len bytes will be received */
   	  	
    for (; i < len; i++)
   		recbuf[i] = mmc_read_byte();					/* �������� receive data */
   		
    i = mmc_read_byte();								
    i = (i << 8) + mmc_read_byte();    				/* ��ȡ16λCRC get 16-bit CRC */	
  
  
    mmc_write_byte(0xFF);								/* ����֮ǰ����8��clock  clock out 8 clk before return */
    MMC_Disable();	
    	
	return SD_NO_ERR;
}	

/*******************************************************************************************************************
** ��������: uint8 SD_ReadBlockData()			Name:	  uint8 SD_ReadBlockData()
** ��������: �ӿ��ж�ȡ���ݿ�					Function: read block data from card
** �䡡  ��: uint32 len    : ����				Input:	  uint32 len    : length
			 uint8 *recbuf : ���ջ�����					  uint8 *recbuf : the buffer of receive
** �� �� ��: 0:   ��ȷ    >0:   ������		  	Output:	  0:  right		>0:  error code
*******************************************************************************************************************/
uint8 SD_ReadBlockData(uint32 len, uint8 *recbuf)
{
    uint8 tmp;
    uint32 i = 0,timeout;
    
	timeout = sds.timeout_read;							/* �ȴ��������ݿ�ʼ�����ʱ�� wait time that receive data start token */
    
    MMC_Enable();    
    do
    { 											    	/* �ȴ��������ݿ�ʼ����0xFE  wait for receiving data start token 0xFE */
        tmp = mmc_read_byte();
        i++;
    }while((tmp == 0xFF) && (i < timeout));
	

	if (i >= timeout)
	{
		MMC_Disable();
       	return SD_ERR_TIMEOUT_READ;						/* ���ض���ʱ������  return error timeout error code of reading */
	}
	
	if (tmp != SD_TOK_READ_STARTBLOCK)					/* �����ʼ���ƴ��� read start block token is error */
	{
		mmc_write_byte(0xFF);
		MMC_Disable();
		return SD_ERR_DATA_START_TOK;
	}
	
	for (i = 0; i < len; i++)
   		recbuf[i] = mmc_read_byte();						/* �������� receive data */
   		
    i = mmc_read_byte();								
    i = (i << 8) + mmc_read_byte();    					/* ��ȡ16λCRC get 16-bit CRC */	  

	mmc_write_byte(0xFF); 
	MMC_Disable();

  	return SD_NO_ERR;									/* ���غ���ִ�гɹ� return function perform sucessfully */
}

/*******************************************************************************************************************
** ��������: uint8 SD_WriteBlockData()				Name:	  uint8 SD_WriteBlockData()
** ��������: ��д���ݿ�							Function: write block data to card
** �䡡  ��: uint8 bmulti  : �Ƿ�Ϊ������1:��0:�� Input:	  uint8 bmulti   : multi blocks operate 1:Y 0:N 
			 uint32 len    : ����						  	  uint32 len     : length
			 uint8 *sendbuf: ���ͻ�����					 	  uint8 *sendbuf : the buffer of send
** �� �� ��: 0:   ��ȷ    >0:   ������		  		Output:	  0:  right		>0:  error code
********************************************************************************************************************/
uint8 SD_WriteBlockData(uint8 bmulti, uint32 len, uint8 *sendbuf)
{
	uint16 i;
	uint8 tmp;

	MMC_Enable();
		
    mmc_write_byte(0xFF);								/* ��ʼ��������֮ǰ����8��clock clock out 8 clk before start */
    
    if (bmulti == 1)
        mmc_write_byte(SD_TOK_WRITE_STARTBLOCK_M);	/* д��鿪ʼ���� start token of write multi blocks */
	else
		mmc_write_byte(SD_TOK_WRITE_STARTBLOCK);		/* д���鿪ʼ���� start token of write single block */

	for (i = 0; i < len; i++)
        mmc_write_byte(sendbuf[i]);					/* �������� send data */


	mmc_write_byte((i >> 8) & 0xFF);
	mmc_write_byte(i & 0xFF); 						/* ����CRC16У���� send CRC16 check code */
			    
	tmp = mmc_read_byte();
  	if ((tmp & SD_RESP_DATA_MSK) != SD_RESP_DATA_ACCETPTED)	
  	{		
   		mmc_write_byte(0xFF);							/* ����֮ǰ����8��clock  clock out 8 clk before return */
   		MMC_Disable();
   		return SD_ERR_DATA_RESP;					/* ������Ӧ���� data response error */
    }
        
    MMC_Disable();
     		
    if (SD_WaitBusy(SD_WAIT_WRITE) != SD_NO_ERR)			
    	return SD_ERR_TIMEOUT_WRITE;				/* д�볬ʱ write time out */
    return SD_NO_ERR; 							/* д����ȷ write right */
 }

/*******************************************************************************************************************
** ��������: void SD_StopMultiToken()				Name:	  void SD_StopMultiToken(void)
** ��������: ���Ͷ��дֹͣ����						Function: send the token that stop multiple block write
** �䡡  ��: ��									    Input:	  NULL
** �� �� ��: ��										Output:	  NULL
********************************************************************************************************************/
void SD_StopMultiToken(void)
{
	MMC_Enable();
	
	mmc_write_byte(0xFF);								/* �ȷ���8��clock send 8 clock first */
	mmc_write_byte(SD_TOK_STOP_MULTI);				/* ����ֹͣ���ݴ������� send stop transmission token */
	mmc_read_byte();
	
    MMC_Disable();
}


/********************************************************************************************************************
** ��������: void SD_WaitBusy()						Name:	  void SD_WaitBusy()
** ��������: ��ѯSD���Ƿ���æ״̬					Function: poll SD Card wheather it is busy
** �䡡  ��: uint32 waittype: ��ʱ����				Input:	  uint32 timeout: time out type
** �� �� ��: 0: δ��ʱ  >0: ������					Output:	  0: not time out   > 0: error code
*********************************************************************************************************************/
uint8 SD_WaitBusy(uint8 waittype)
{
    uint32 timeout, i = 0;
    uint8 tmp;
    
  	if (waittype == SD_WAIT_WRITE)
  		timeout = sds.timeout_write;				/* �ȴ�����Ϊд���� wait type is write operation */
  	else
  		timeout = sds.timeout_erase;   				/* �ȴ�����Ϊ�������� wait type is erase operation */
    	
   
	MMC_Enable();
   	do
   	{ 												/* �ȴ�æ���� wait for being busy end */
        tmp = mmc_read_byte();
        i++;
    }while ((tmp != 0xFF) && (i < timeout));		/* æʱ�յ���ֵΪ0 always receive 0 when card is busy */    


	if(i < timeout) 
		tmp = SD_NO_ERR;							/* ����0,��ʾû��ʱ return 0 indicate that operation is not time out */
	else 
		tmp = SD_ERR_TIMEOUT_WAIT;					/* ���ش�����,��ʾ��ʱ return error code indicate that operation is time out */

	mmc_write_byte(0xFF);
	MMC_Disable();								
	return tmp;										/* ����ִ�н�� */
}

/********************************************************************************************************************
** ��������: void SD_SPIDelay()						Name:	  void SD_SPIDelay()
** ��������: SPI������ʱ							Function: SPI Bus delay 
** �䡡  ��: uint8 value: ��ʱֵ,������255		    Input:	  uint8 value : delay value,do not beyond 255
** �� �� ��: ��										Output:	  NULL
*********************************************************************************************************************/
void SD_SPIDelay(uint8 value)
{
    uint8 i;

    for (i = 0; i < value; i++)
        mmc_write_byte(0xFF);						 	/* ����0xFF clock out 0xFF */
}








