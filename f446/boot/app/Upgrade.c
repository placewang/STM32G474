#include "Upgrade.h"
#include "main.h"
#include "string.h"
#include "bsp_flash.h"

#define UPDATA_INFO_LEN   5

enum
{
  UPGRADE_JUDMENT=0,
  UPGRADE_CRC,	
  UPGRADE_BURN,
  UPGRADE_SUCCESS,
  UPGRADE_FAIL,		
};

#pragma pack(push, 1)
// 协议帧结构体（按实际字节顺序排列）
typedef struct 
{
   volatile unsigned int  OperationFlag;              /*运行区标志*/
   volatile unsigned int  crc32;                      /*整个文件的检验*/
   volatile unsigned int  FileSize;                   /*整个文件有效数据的字节总数*/  
   volatile unsigned int  SoftwareVersion;            /*软件版本*/ 
   volatile unsigned int  HardwareVersion;            /*硬件版本*/	
} FlashInfo;
#pragma pack(pop)

typedef union {
    FlashInfo frame;	
    unsigned  int     bytes[UPDATA_INFO_LEN];
}upgrade_Info;


typedef struct 
{
	uint8_t Process;
} upgrade_Process;

upgrade_Process upgrade_Prs={0};
upgrade_Info upDataInfo={0};

extern uint32_t ledTime;
/**
 * 计算 CRC-32 校验值（行为与 Python 的 zlib.crc32一致）
 * @param data  输入字节数据指针
 * @param len   数据长度（字节）
 * @return      32 位无符号 CRC 校验值
 */
uint32_t crc32_zlib(const unsigned char *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;        /* 初始值*/
    const uint32_t poly = 0xEDB88320u; /* 反射多项式*/
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1u)
                crc = (crc >> 1) ^ poly;
            else
                crc >>= 1;
        }
    }/*最终异或*/
    return crc ^ 0xFFFFFFFFu;        
}
/*
升级成功写参数标记到存储区地址
*/
void Update_writeflag(uint32_t FLAG_addr,uint32_t *dataF,uint32_t DataLen,uint16_t indx)
{
	uint64_t  writeLen=16;
	uint32_t  flagData[16]={0};
	
	bsp_STMFLASH_Read(FLAG_addr,flagData,writeLen);
	bsp_IndividualSectorErasure(FLAG_addr);
	for(int i=0;i<DataLen;i++)
	{
		flagData[indx+i]=dataF[i];
	}
	bsp_STMFLASH_Write(FLAG_addr,flagData,writeLen);
}
/*读取指定地址的字(32位数据) 
faddr:读地址 
返回值:对应数据.
*/
unsigned int STMFLASH_ReadWord(unsigned int faddr)
{
	return *(volatile unsigned int*)faddr; 
}

/*
获取升级参数信息
*/
void Upgrade_getInformation(void)
{
  bsp_STMFLASH_Read(BOOT_FLASH_UPGRADE_ADDRESS,upDataInfo.bytes,UPDATA_INFO_LEN);
}

/*
计算备份区数据CRC
*/
int Upgrade_BackupCRC32(void)
{
	uint32_t CRC32=0;
    uint8_t *flash_addr=(unsigned char *)FLASH_APP_START_ADDRESS_BACKUP;
	if(upDataInfo.frame.FileSize>0X34000)
	{
		return 0;
	}
	CRC32=crc32_zlib(flash_addr,upDataInfo.frame.FileSize);
	if(upDataInfo.frame.crc32==CRC32)
	{
		return 1;
	}
	else
	{
		return 0;
	}	
}
/*
判断升级状态
*/
int Upgrade_JudgmentSta(void)
{
	/*意外断电等因素导致上次升级未搬运完成*/
	if(upDataInfo.frame.OperationFlag==CMD_UPGRADE_BURN)
	{
		return UPGRADE_CRC;
	}/*正常升级流程*/
	else if(upDataInfo.frame.OperationFlag==CMD_UPGRADE_RCV_SUCCESS)
	{
		return UPGRADE_CRC;
	}
	else
	{
		return UPGRADE_FAIL;
	}
}
/*
数据从备份区搬运到APP区
**/
int Upgrade_Burn_BackupToApp(void)
{
	uint32_t CRC_32=0;
	uint32_t appAddr=FLASH_APP_START_ADDRESS_APP;
	uint32_t* backupAddr=(uint32_t*)FLASH_APP_START_ADDRESS_BACKUP;
	uint32_t wordCount=(upDataInfo.frame.FileSize + 3U) / 4U;
	/*擦除APP数据*/
	bsp_FlashEraseSectors(FLASH_APP_START_ADDRESS_APP,3);
	/*搬数据*/
	bsp_STMFLASH_Write(appAddr,backupAddr,wordCount);
	/*CRC*/
	CRC_32=crc32_zlib((const uint8_t *)appAddr,upDataInfo.frame.FileSize);
	if(upDataInfo.frame.crc32==CRC_32)
	{
		return 1;
	}
	else
	{
		return 0;
	}
	return 0;
}
void Upgrade_Process(void)
{
	uint32_t flagData=0;
	/*CRC未通过或文件损坏LED快闪*/	
	if(upgrade_Prs.Process==UPGRADE_FAIL)
	{
		ledTime=1000*80;	
		return;
	}
	switch (upgrade_Prs.Process)
	{
		case UPGRADE_JUDMENT :
			upgrade_Prs.Process=Upgrade_JudgmentSta();
			break;
		case UPGRADE_CRC:	
			if(Upgrade_BackupCRC32())
			{
				flagData=CMD_UPGRADE_BURN;
				upgrade_Prs.Process=UPGRADE_BURN;
				Update_writeflag(BOOT_FLASH_UPGRADE_ADDRESS,&flagData,1,0);
			}
			else
			{
				upgrade_Prs.Process=UPGRADE_FAIL;
			}
			break;
		case UPGRADE_BURN :	
			if(Upgrade_Burn_BackupToApp())
			{/*数据搬运升级完成*/
				flagData=CMD_UPGRADE_SUCCESS;
				upgrade_Prs.Process=UPGRADE_SUCCESS;
			}
			else
			{
				flagData=CMD_UPGRADE_FAIL;
				upgrade_Prs.Process=UPGRADE_FAIL;
			}
			Update_writeflag(BOOT_FLASH_UPGRADE_ADDRESS,&flagData,1,0);
			break;	
		case UPGRADE_SUCCESS:	
		  	__set_PRIMASK(1);  
		     NVIC_SystemReset();
		  break;	
	 }
}


void GoApp(unsigned int FLASH_APP_START_ADDRESS)
{
    uint32_t stack_addr = *(uint32_t*)FLASH_APP_START_ADDRESS;
    // 检查栈顶是否在 RAM 范围内
    if (stack_addr < 0x20000000UL || stack_addr >= 0x20040000UL) {
        return;   // 非法 APP
    }

    /* 关闭全局中断 */
    __set_PRIMASK(1);

    /* 关闭滴答定时器 */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* 复位 RCC 时钟配置 */
    HAL_RCC_DeInit();

    /* 禁用所有外设中断并清除挂起标志 */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* 设置 CONTROL 寄存器（特权线程模式，MSP 堆栈） */
    __set_CONTROL(0);
    __ISB();
    __DSB();

    /* 获取 APP 复位向量地址并跳转 */
    uint32_t jump_addr = *(uint32_t*)(FLASH_APP_START_ADDRESS + 4);
    pFunction Jump_To = (pFunction)jump_addr;

    /* 设置 APP 的栈指针 */
    __set_MSP(stack_addr);
	/* 重新使能全局中断*/
	__set_PRIMASK(0);   
    /* 注意：此处全局中断仍为关闭状态，APP 启动后需自己开启 */
    Jump_To();

}



void RestStarMCU(void)
{
    __set_PRIMASK(1);   //关闭所有中端
    NVIC_SystemReset(); //复位    
}











