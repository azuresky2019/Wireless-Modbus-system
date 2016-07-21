//#include	"types.h"
#include "hal_board.h"
#include "hal_types.h"
  
//typedef uint8 uint8;
//typedef uint16 uint16;
//typedef uint32 uint32;

#define MMC_Chip_Select P0_4
#define SPI_Clock P0_5
#define SPI_DI  P0_2
#define SPI_DO  P0_3

//define ASCII
#define SPACE 				0x20
#define DIR_ENTRY_IS_FREE   0xE5
#define FIRST_LONG_ENTRY	0x01
#define SECOND_LONG_ENTRY	0x42

//define DIR_Attr
#define ATTR_LONG_NAME		0x0F
#define ATTR_READ_ONLY		0x01
#define ATTR_HIDDEN			0x02
#define ATTR_SYSTEM			0x04
#define ATTR_VOLUME_ID		0x08
#define ATTR_DIRECTORY		0x10
#define ATTR_ARCHIVE		0x20


typedef struct 
{
	uint8 BS_jmpBoot[3];
	uint8 BS_OEMName[8];
	uint16 BPB_BytesPerSec;  //2 bytes
	uint8	BPB_SecPerClus;
	uint16	BPB_RsvdSecCnt; //2 bytes
	uint8	BPB_NumFATs;
	uint16	BPB_RootEntCnt; //2 bytes
	uint16	BPB_TotSec16; //2 bytes
	uint8	BPB_Media;
	uint16	BPB_FATSz16; //2 bytes
	uint16	BPB_SecPerTrk; //2 bytes
	uint16	BPB_NumHeads; //2 bytes
	uint32	BPB_HiddSec; //4 bytes
	uint32	BPB_TotSec32; //4 bytes
} BootSec;

//FAT12 and FAT16 Structure Starting at Offset 36
#define BS_DRVNUM			36
#define BS_RESERVED1		37
#define BS_BOOTSIG			38
#define BS_VOLID		       	39
#define BS_VOLLAB			43
#define BS_FILSYSTYPE		54

//FAT32 Structure Starting at Offset 36
#define BPB_FATSZ32			36
#define BPB_EXTFLAGS		40
#define BPB_FSVER			42
#define BPB_ROOTCLUS		44
#define BPB_FSINFO			48
#define BPB_BKBOOTSEC		50
#define BPB_RESERVED		52

#define FAT32_BS_DRVNUM		64
#define FAT32_BS_RESERVED1	65
#define FAT32_BS_BOOTSIG	66
#define FAT32_BS_VOLID		67
#define FAT32_BS_VOLLAB		71
#define FAT32_BS_FILSYSTYPE	82
//End of Boot Sctor and BPB Structure

typedef struct  {
	uint8	DIR_Name[11];     //8 chars filename
	uint8	DIR_Attr;         //file attributes RSHA, Longname, Drive Label, Directory
	uint8	DIR_NTRes;        //set to zero
	uint8	DIR_CrtTimeTenth; //creation time part in milliseconds
	uint16	DIR_CrtTime;      //creation time
	uint16	DIR_CrtDate;      //creation date
	uint16	DIR_LastAccDate;  //last access date
	uint16	DIR_FstClusHI;    //first cluster high word                 
	uint16	DIR_WrtTime;      //last write time
	uint16	DIR_WrtDate;      //last write date
	uint16	DIR_FstClusLO;    //first cluster low word                 
	uint32	DIR_FileSize;     
	}DirEntry; 