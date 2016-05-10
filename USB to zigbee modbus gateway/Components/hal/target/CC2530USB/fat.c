
#include "mmc.h"
#include <string.h>
//#include "NC_IO.h" 
//#include "sd.h"
#include "fat.h"


#define BlockSize			512
#define MASTER_BOOT_RECORD	0
#define VBR_ADDR 			0x1C6

#if MMC_WRITE_ENABLE
  uint8    SectorBuffer[520];
#else
  uint8 SectorBuffer[1];
#endif
  
uint32    Fat1Address=0;            //  FAT1  表的绝对扇区地址
uint16    FatLength;
uint8 	  FatNumber;
uint32    RootDirAddress=0;       //  根目录的绝对扇区地址
uint32    DataAddress=0;          //  数据区的绝对扇区地址
uint8    cluster_size;               //  簇的大小，即一个簇多少个扇区。
uint16    sector_size;                //  簇的大小，即一个簇多少个扇区。


//  写文件操作
uint32         FileNameSectorNumber;
uint16         FileNamePageLocate;
uint16         FileCluster;
uint32       FileSize;
uint8      FileAttrib;
uint32       FileWriteSectorAdd;


uint16  fat_offset;            //  保留扇区数量
uint16  cluster_offset;      //  数据扇区的地址
uint16  volume_boot_record_addr;           //  系统引导记录的绝对扇区地址。

uint8 CompareString(uint8 *str1 ,uint8 *str2 ,uint16 leng);
void  UlongToByte(uint32 ulVal  , uint8 *uVal);


unsigned long FirstSectorofCluster(unsigned int clusterNum)
{
	unsigned long temp;
	temp=clusterNum-2;
	temp=temp*cluster_size;
	temp=temp+DataAddress;
	return temp;
};




unsigned long ThisFatSecNum(unsigned int clusterNum)
{
   unsigned long temp;
   temp=clusterNum*2;
   temp=temp/sector_size;             
   temp=temp+Fat1Address;
   return temp;
};



unsigned int ThisFatEntOffset(unsigned int clusterNum)
{
	unsigned long temp1,temp2;
	temp1=2*clusterNum;
	temp2=temp1/sector_size;
	temp1=temp1-temp2*sector_size;
	return temp1;
}



unsigned int GetNextClusterNum(unsigned int clusterNum)
{
	unsigned long FatSecNum,FatEntOffset;
	
	FatSecNum=ThisFatSecNum(clusterNum);
	FatEntOffset=ThisFatEntOffset(clusterNum);
	mmc_read_sector(FatSecNum,SectorBuffer);
	
	clusterNum=SectorBuffer[FatEntOffset+1];
	clusterNum=clusterNum<<8;
	clusterNum+=SectorBuffer[FatEntOffset];	
	return clusterNum;
}


unsigned int GetFreeCusterNum(void)
{
	unsigned int clusterNum,i;
	unsigned long sectorNum;
	unsigned char j;
	clusterNum=0;
	sectorNum=Fat1Address;
	while(sectorNum<FatLength+Fat1Address)
	{
		
		mmc_read_sector(sectorNum,SectorBuffer);
		for(i=0;i<sector_size;i=i+2)
	  	{
		  	 if((SectorBuffer[i]==0)&&(SectorBuffer[i+1]==0))
	  	 	{	
		  	 	SectorBuffer[i]=0xff;
		  	 	SectorBuffer[i+1]=0xff;

				//   写Fat 表
				for(j=0;j<FatNumber ;j++)
				{
					mmc_write_sector(sectorNum+j*FatLength,SectorBuffer);
				}	
		  	  	return	clusterNum; 
	  	 	}
		  	 clusterNum++;
	  	}	
		sectorNum=((2*clusterNum)/sector_size)+Fat1Address;	
	}
	
	return 0x0;
}


unsigned int CreateClusterLink(unsigned int currentCluster)
{
	unsigned int newCluster;
	unsigned long  FatSecNum;
	unsigned int     FatEntOffset;
	unsigned char i;

	newCluster=GetFreeCusterNum();

	if(newCluster==0)
		return 0x00;
	FatSecNum=ThisFatSecNum(currentCluster);
	FatEntOffset=ThisFatEntOffset(currentCluster);
	mmc_read_sector(FatSecNum, SectorBuffer);
	SectorBuffer[FatEntOffset]=newCluster;
	SectorBuffer[FatEntOffset+1]=newCluster>>8;
	for(i=0;i<FatNumber ;i++)
	{
		mmc_write_sector(FatSecNum+i*FatLength,SectorBuffer);
	}	
	return newCluster;
}




uint16 fat_DBR_addr (void)
{
	uint16 volume_boot_record_addr;
	mmc_read_sector (MASTER_BOOT_RECORD,SectorBuffer); //Read Master Boot Record    MBR
	if((SectorBuffer[0]==0xEB)&&(SectorBuffer[1]==0x3C)&&(SectorBuffer[2]==0x90))
	{
		volume_boot_record_addr=0;
	}
	else
	{
	    volume_boot_record_addr = SectorBuffer[VBR_ADDR] + (SectorBuffer[VBR_ADDR+1] << 8);
	}
    return (volume_boot_record_addr);
}


//  读出系统文件目录表的地址
uint16 fat_root_dir_addr (void) 
{
	uint16 FirstRootDirSecNum;
	mmc_read_sector (volume_boot_record_addr,SectorBuffer);
	FirstRootDirSecNum = ( SectorBuffer[14]+SectorBuffer[15]*256 +  SectorBuffer[16] * (SectorBuffer[22]+SectorBuffer[23]*256));      //James@070607
	FirstRootDirSecNum+= volume_boot_record_addr;
	return(FirstRootDirSecNum);
}

//  分析DBR 数据
void fat_cluster_data_store (void)
{
	BootSec *bootp; //Zeiger auf Bootsektor Struktur

	volume_boot_record_addr = fat_DBR_addr ();	//读绝对地址数据分析
	mmc_read_sector (volume_boot_record_addr,SectorBuffer);
    bootp=(BootSec *)SectorBuffer;
	cluster_size = bootp->BPB_SecPerClus;            //    每个簇的扇区数。
	fat_offset = SectorBuffer[14]+SectorBuffer[15]*256 ;              //    保留扇区数       //James@070607
	sector_size=SectorBuffer[11]+SectorBuffer[12]*256;
	cluster_offset = ((( SectorBuffer[11]+SectorBuffer[12]*256)* 32)/BlockSize);	  
	RootDirAddress= fat_root_dir_addr();       // 文件目录表位置
	Fat1Address=SectorBuffer[14]+SectorBuffer[15]*256+volume_boot_record_addr;
	cluster_offset += RootDirAddress;      
	DataAddress=cluster_offset;
	FatNumber=SectorBuffer[16];
	FatLength=SectorBuffer[22]+SectorBuffer[23]*256;
}



void fat_load (	uint16 Cluster, 		//Angabe Startcluster
				uint32 *Block)
{
	uint16 FAT_Block_Store = 0;	
	uint16 a;
	uint16 FAT_Byte_Addresse;	
	uint16 FAT_Block_Addresse;
	for ( a = 0;;a++)
	{	
		if (a == *Block)
		{
			*Block = (0x0000FFFF & Cluster);
			return;
		}
		
		if (Cluster == 0xFFFF)
		{
			break; // End  File
		}
		FAT_Byte_Addresse = (Cluster*2) % BlockSize;
			
		FAT_Block_Addresse = ((Cluster*2) / BlockSize) + volume_boot_record_addr + fat_offset;	

		if (FAT_Block_Addresse != FAT_Block_Store)
		{
			FAT_Block_Store = FAT_Block_Addresse;

			mmc_read_sector (FAT_Block_Addresse,SectorBuffer);	
		}

		Cluster = (SectorBuffer[FAT_Byte_Addresse + 1] << 8) + 	SectorBuffer[FAT_Byte_Addresse];		
	}
	return;
}



uint16 fat_read_dir_ent (uint16 dir_cluster,	uint8 * filename, uint32 *Size, 	uint8 *Dir_Attrib)  
{
	uint32 Block = 0;
	uint32 blk ;
	uint16 a ;
	DirEntry *dir; 
	if (dir_cluster == 0)
	{
		Block = fat_root_dir_addr();
	}
	else
	{
		fat_load (dir_cluster,&Block);			 
		Block = ((Block-2) * cluster_size) + cluster_offset;
	}
	for ( blk = Block;blk<Block+32;blk++)
	{
		mmc_read_sector (blk,SectorBuffer);	
		for ( a=0;a<BlockSize; a = a + 32)
		{
			 dir=(DirEntry *)&SectorBuffer[a]; 
		 
			if (dir->DIR_Name[0] == 0) 
			{
				return (0xFFFF);
			}
   			if ((dir->DIR_Attr != ATTR_LONG_NAME) && (dir->DIR_Name[0] != DIR_ENTRY_IS_FREE)) 
			{
				if(CompareString(dir->DIR_Name,filename,11))
				{
					*Dir_Attrib = dir->DIR_Attr;
					*Size=SectorBuffer[a+28]+SectorBuffer[a+29]*256+(uint32)SectorBuffer[a+30]*65536+(uint32)SectorBuffer[a+31]*16777216;
					dir_cluster = SectorBuffer[a+26]+SectorBuffer[a+27]*256;           //James@070607
					return(dir_cluster);
				}
			}
		}
	}
	return (0xFFFF); 
}





uint16    gcluster ;
uint8    gpage ;
uint16      glocate ;

uint8  fat_read_file(uint16 Cluster, uint32  offset, uint16  length)
{
	uint32  Block;
	uint16     i;

	gcluster=(offset/512)/cluster_size;
	gpage=(offset%(512*cluster_size))/512;
	glocate=(offset%(512*cluster_size))%512;
	if(length>(512-glocate))
		length=(512-glocate	);
	for(i=0;i<gcluster;i++)
	{
		Cluster=GetNextClusterNum(Cluster);
	}
	Block=FirstSectorofCluster(Cluster);
	Block=Block+gpage;
	mmc_read_sector (Block,SectorBuffer);	
	if(glocate)
	{
		for(i=0;i<length;i++)
		{
			SectorBuffer[i]=SectorBuffer[i+glocate];
		}
	}
	return 1;
}


uint16   fat_Next_lock(uint16 Cluster, uint32  offset)
{
	uint16     i;
	uint16     oldcluster;

	gcluster=(offset/512)/cluster_size;
	gpage=(offset%(512*cluster_size))/512;
	glocate=(offset%(512*cluster_size))%512;
	if(offset==0)
	{
		FileWriteSectorAdd=FirstSectorofCluster(Cluster);
		return  Cluster;
	}
	for(i=0;i<gcluster;i++)
	{
		oldcluster=Cluster;
		Cluster=GetNextClusterNum(Cluster);
	}
	if(gpage==0)
	{  //  要查找新的簇
		FileCluster=CreateClusterLink(oldcluster);
		if(FileCluster==0)
		{  
			//  没有簇可以分配
			return 0;
		}
		FileWriteSectorAdd=FirstSectorofCluster(FileCluster);
	}
	else
	{
		FileWriteSectorAdd=FirstSectorofCluster(FileCluster);
		FileWriteSectorAdd=FileWriteSectorAdd+gpage ;
	}
	return FileCluster;
}




uint8 fat_search_file (uint8 *File_Name,uint16 *Cluster, 	uint32 *Size, uint8 *Dir_Attrib)
{
	uint16 Dir_Cluster_Store = *Cluster;

	*Cluster = fat_read_dir_ent(Dir_Cluster_Store,File_Name,Size,Dir_Attrib);
	if (*Cluster == 0xffff)
	{
		return 0; //File not Found
	}
	return 1; //File Found
}


uint8   fat_write_file_lock(uint8   * file_name)
{
	uint32 Block = 0;
	uint32 blk ;
	uint16 a ;
	DirEntry *dir; 
	Block = fat_root_dir_addr();
	for ( blk = Block;blk<Block+32;blk++)
	{
		mmc_read_sector (blk,SectorBuffer);	
		for ( a=0;a<BlockSize; a = a + 32)
		{
			 dir=(DirEntry *)&SectorBuffer[a]; 
		 
			if (dir->DIR_Name[0] == 0) 
			{  
				FileNameSectorNumber=	blk;	
				FileNamePageLocate=a;
				FileSize=0;
				FileAttrib=0x20;  //  目录属性
				FileCluster=GetFreeCusterNum();
				if(FileCluster==0)
				{  
					return 0;
				}
				FileWriteSectorAdd=FirstSectorofCluster(FileCluster);
				mmc_read_sector (FileNameSectorNumber,SectorBuffer);	    //  读出根目录数据，应为SectorBuffer中数据已经改变了
				SectorBuffer[a]=file_name[0];
				SectorBuffer[a+1]=file_name[1];
				SectorBuffer[a+2]=file_name[2];
				SectorBuffer[a+3]=file_name[3];
				SectorBuffer[a+4]=file_name[4];
				SectorBuffer[a+5]=file_name[5];
				SectorBuffer[a+6]=file_name[6];
				SectorBuffer[a+7]=file_name[7];
				SectorBuffer[a+8]=file_name[8];
				SectorBuffer[a+9]=file_name[9];
				SectorBuffer[a+10]=file_name[10];
				SectorBuffer[a+11]=0x20;  //  目录属性
				SectorBuffer[a+26]=FileCluster;  //  
				SectorBuffer[a+27]=FileCluster/256;  //  
				SectorBuffer[a+28]=0x00;  //  
				SectorBuffer[a+29]=0x00;  //  
				SectorBuffer[a+30]=0x00;  //  
				SectorBuffer[a+31]=0x00;  //  
				mmc_write_sector (FileNameSectorNumber,SectorBuffer);	    
				return (2);
			}
			
   			if ((dir->DIR_Attr != ATTR_LONG_NAME) && (dir->DIR_Name[0] != DIR_ENTRY_IS_FREE)) 
			{
				if(CompareString(dir->DIR_Name,file_name,11))
				{
					FileAttrib = dir->DIR_Attr;
					FileSize=SectorBuffer[a+28]+SectorBuffer[a+29]*256+(uint32)SectorBuffer[a+30]*65536+(uint32)SectorBuffer[a+31]*16777216;
					FileCluster = SectorBuffer[a+26]+SectorBuffer[a+27]*256;           //James@070607
					FileNameSectorNumber=	blk;	
					FileNamePageLocate=a;
					fat_Next_lock(FileCluster,FileSize);					
					return(1);
				}
			}
		}
	}
	return (0); 
}



void    fat_write_file(void)
{
	mmc_write_sector (FileWriteSectorAdd,SectorBuffer);	    
	FileSize=FileSize+512;
	mmc_read_sector (FileNameSectorNumber,SectorBuffer);	    //  读出根目录数据，因为SectorBuffer中数据已经改变了
	UlongToByte(FileSize, &SectorBuffer[FileNamePageLocate+28]);
	mmc_write_sector (FileNameSectorNumber,SectorBuffer);	    
}



uint8 CompareString(uint8 *str1 ,uint8 *str2 ,uint16 leng)
{
	uint16   i;
	for(i=0;i<leng;i++)
	{
		if(str1[i]!=str2[i])
		{
			return 0;
		}
	}
	return 1;
}



void  UlongToByte(uint32 ulVal  , uint8 *uVal)
{
	uint8 temp;
	uVal[0]=ulVal%256;
	temp=ulVal%256;
	ulVal=ulVal/256;
	uVal[1]=ulVal%256;
	temp=ulVal%256;
	ulVal=ulVal/256;
	uVal[2]=ulVal%256;
	temp=ulVal%256;
	ulVal=ulVal/256;
	uVal[3]=ulVal%256;
	temp=ulVal%256;

}



