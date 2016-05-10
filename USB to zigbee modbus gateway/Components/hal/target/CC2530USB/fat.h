/*#######################################################################################
Connect ARM to MMC/SD 

Copyright (C) 2004 Ulrich Radig
#######################################################################################*/

#ifndef _FAT_H_
 #define _FAT_H_

#include <string.h>
#include "mmc.h"

extern uint8 SectorBuffer[];             //  640  +4 +5

//Prototypes

uint16 fat_DBR_addr (void);
uint16 fat_root_dir_addr (void ) ;
uint16 fat_read_dir_ent (uint16 dir_cluster,	uint8 * filename, uint32 *Size, 	uint8 *Dir_Attrib)  ;


void fat_load (	uint16 Cluster,	uint32 *Block);

void fat_cluster_data_store (void);
uint8  fat_read_file(uint16 Cluster, uint32  filesize, uint16  length);
//ulong  fat_read_file_Compare(uint16 Cluster, uint32  filesize);

//void fat_write_file (uint16 cluster,	uint8 *buffer,	uint32 blockCount);

uint8 fat_search_file (uint8 *File_Name,uint16 *Cluster, uint32 *Size, uint8 *Dir_Attrib);


uint8   fat_write_file_lock(uint8   * file_name);

void    fat_write_file(void);
extern unsigned long FirstSectorofCluster(unsigned int clusterNum);
extern unsigned long ThisFatSecNum(unsigned int clusterNum);
unsigned int ThisFatEntOffset(unsigned int clusterNum);
unsigned int GetNextClusterNum(unsigned int clusterNum);
unsigned int GetFreeCusterNum(void);
unsigned int CreateClusterLink(unsigned int currentCluster);
uint16   fat_Next_lock(uint16 Cluster, uint32  offset);


#endif //_FAT_H_
