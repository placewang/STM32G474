#include "string.h"
#include "queue.h"

#include "bsp_tim2.h"
#include "bsp_spi.h"
#include "bsp_dcan.h"
#include "bsp_flash.h"

#include "app_IapUpData.h"


UpgradeProcess  iap_ps;
UpDataPack      iapRxbuff,iapTxbuff;

#define CHECKSUM_LEN            63            /*数据校验长度*/
// FLASH 升级标志
#define CMD_UPGRADE_START		0xA050AA00	  /* 启动升级的标志*/
#define CMD_UPGRADE_RCV_FAIL	0xA050AA55	  /* 接收升级数据失败*/
#define CMD_UPGRADE_RCV_SUCCESS	0xAA5A5A5A	  /* 接收升级数据完成*/
#define CMD_UPGRADE_BURN	    0xA0505A5A	  /* 搬运数据标志*/
#define CMD_UPGRADE_SUCCESS		0x55AA55AA	  /* 升级完成标志*/
#define CMD_UPGRADE_FAIL		0xBBAA55AA	  /* 升级完成失败*/
#define __RebootTime            (1000*1000)   /*升级成功后重启计时*/

uint32_t XORCheck(uint8_t *data, int len);
/*
主机通讯初始化
*/
void app_IapUpData_init(void)
{

	memset(iapRxbuff.bytes,0,PACK_UPDATA_LEN);
	memset(iapTxbuff.bytes,0xFF,PACK_UPDATA_LEN);
}

/**
 * 计算 CRC-32 校验值（zlib.crc32一致）
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
数据格式转化
*/
static inline uint32_t get_le32(const uint8_t *ptr, size_t offset) {
    return ((uint32_t)ptr[offset])       |
           ((uint32_t)ptr[offset+1]<<8)  |
           ((uint32_t)ptr[offset+2]<<16) |
           ((uint32_t)ptr[offset+3]<<24);
}
/*
擦除待写入程序存储区
*/
static inline void EraseSectors_flashBlock(uint32_t flag)
{
	if(0!=flag)
	{
		iap_ps.flash_address=FLASH_RUNING_AREA_BACKUP;
		bsp_FlashEraseSectors(FLASH_RUNING_AREA_BACKUP,2);
	}
//	else
//	{
//		iap_ps.flash_address=FLASH_RUNING_AREA_APP;
//		bsp_FlashEraseSectors(FLASH_RUNING_AREA_APP,3);
//	}
}
/*
回包填充
*/
static inline void ReturnPackageFill(UpDataPack*txRpack,\
									const unsigned short cmd,\
									const unsigned int   num,\
									const unsigned short len,\
									const unsigned char * data)
{
	if(txRpack==NULL) {return;}
	memset(txRpack->bytes,0xff,PACK_UPDATA_LEN);
	txRpack->frame.header=UPDATA_HEADER;
	txRpack->frame.footer=UPDATA_FOOTER;
	txRpack->frame.cmd=cmd;
	txRpack->frame.packnum=num;
	txRpack->frame.len=len;
    /*
	 txRpack->frame.data=0xff;
	*/
	txRpack->frame.checksum=XORCheck(txRpack->bytes,CHECKSUM_LEN);
	iap_ps.TxFlag=1;
}
/*
	满512字节先写入内部FLASH扇区写操作
	address：写的地址
	data_p：写的数据
*/
static inline void UpgradeData_WriteFlash(unsigned int* address, Rxbuff *data_p,const uint32_t wlen)
{	
    if(address==NULL||data_p==NULL||128!=wlen)
	{
		return;
	}
	bsp_STMFLASH_Write(*address,data_p->flash,wlen);
	*address+=wlen*4;
	memset(data_p->data,0xFF, wlen*4); 
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
/*
升级请求数据包处理
*/
void UpdateRequestDataPack(UpDataPack *rec,UpDataPack* trm)
{
	static uint32_t writeflash_data[5]={0};
	 
    /*清标志*/
	 iap_ps.rx_buf_len=0;
	 iap_ps.recv_all_data_len=0;
	 memset(iap_ps.rx_buf.data,0xFF, 512); 
	/*存参数信息*/
	iap_ps.crc32=get_le32(rec->frame.data,0);
	iap_ps.FileSize=get_le32(rec->frame.data,4);
	iap_ps.SoftwareVersion=get_le32(rec->frame.data,8);
	iap_ps.HardwareVersion=get_le32(rec->frame.data,12);
	/*存参数信息写入标志区*/
	writeflash_data[0]=CMD_UPGRADE_START; /*写入启动标志*/
	writeflash_data[1]=iap_ps.crc32;
	writeflash_data[2]=iap_ps.FileSize; 
	writeflash_data[3]=iap_ps.SoftwareVersion; 
	writeflash_data[4]=iap_ps.HardwareVersion; 
	Update_writeflag(FLASH_OPERATION_FLAG_BASS,writeflash_data,5,0);
	/*选择备份区*/
	iap_ps.OperationFlag=1;
	/*擦除待写入程序存储区*/
	EraseSectors_flashBlock(iap_ps.OperationFlag);
	/* 准备应答包 */
	ReturnPackageFill(trm,Update_ACK,0xff,UPDATA_LEN,NULL);
}
/*
升级数据包写入flash
*/
void UpdateDataPackWriteToFlash(UpDataPack *rxf,UpDataPack* txf)
{
	int data_len=128;
	for(int i=0;i<rxf->frame.len;i++)
	{
		 iap_ps.rx_buf.data[iap_ps.rx_buf_len]=rxf->frame.data[i];
		 iap_ps.rx_buf_len++;
		 iap_ps.recv_all_data_len++;  
		 if(iap_ps.rx_buf_len==512)
		 {//缓存512满写入FLASH
			 iap_ps.rx_buf_len=0;
			 UpgradeData_WriteFlash(&iap_ps.flash_address,&iap_ps.rx_buf,data_len);   
		 }
	}
	/*填充返回包*/
	ReturnPackageFill(txf,Update_Data,rxf->frame.packnum,UPDATA_LEN,NULL);
}

	
/*
升级数据最后一包写入flash
*/
void LastBatchData(UpDataPack *rxl,UpDataPack* txl)
{
	int packlen=128;
	uint8_t *flash_addr=NULL;
	uint32_t Successflag=0;
	for(int i=0;i<rxl->frame.len;i++)
	{
		 iap_ps.rx_buf.data[iap_ps.rx_buf_len]=rxl->frame.data[i];
		 iap_ps.rx_buf_len++;
		 iap_ps.recv_all_data_len++;
		 if(iap_ps.rx_buf_len==512)
		 {//缓存512满写入FLASH
			 iap_ps.rx_buf_len=0;
			 UpgradeData_WriteFlash(&iap_ps.flash_address,&iap_ps.rx_buf,packlen);  
		 }
	}
	if(0!=iap_ps.rx_buf_len)
	{
		iap_ps.rx_buf_len=0;
		UpgradeData_WriteFlash(&iap_ps.flash_address,&iap_ps.rx_buf,packlen);
	}
	if(0!=iap_ps.OperationFlag)
	{
		Successflag=CMD_UPGRADE_RCV_SUCCESS;
		flash_addr=(unsigned char *)FLASH_RUNING_AREA_BACKUP;
	}

	/*校验整个升级文件，成功更新运行区标价，返回成功包，定时重启*/	
	if(iap_ps.crc32==crc32_zlib(flash_addr,iap_ps.recv_all_data_len))
	{
		ReturnPackageFill(txl,Update_Success,rxl->frame.packnum,UPDATA_LEN,NULL);
		Update_writeflag(FLASH_OPERATION_FLAG_BASS,&Successflag,1,0);
		bsp_tim2Start(&TIM2_T2,__RebootTime);
	}/*失败返回*/
	else
	{
		Successflag=CMD_UPGRADE_RCV_FAIL;
		Update_writeflag(FLASH_OPERATION_FLAG_BASS,&Successflag,1,0);
		ReturnPackageFill(txl,Update_Fail,rxl->frame.packnum,UPDATA_LEN,NULL);
	}
	iap_ps.rx_buf_len=0;
	iap_ps.recv_all_data_len=0;
}
/*
数据解析拆包
*/
void app_UpDataPacketDisassembly_loop(void)
{
	if(0!=iap_ps.RecFlag)
	{
		iap_ps.RecFlag=0;
		switch(iapRxbuff.frame.cmd)
		{
			case Update_Request:
				UpdateRequestDataPack(&iapRxbuff,&iapTxbuff);
				break;
			case Update_Data:
				 UpdateDataPackWriteToFlash(&iapRxbuff,&iapTxbuff);
				 break ;
			case Update_LastPack:
				 LastBatchData(&iapRxbuff,&iapTxbuff);
				 break;
		}
		/* 提交准备好的应答帧到 SPI 发送影子区 */
		bsp_SPI1_SubmitTxData(iapTxbuff.bytes);			
	 }

	/*升级完成重启  关闭总中断*/
	if(bsp_tim2GetSta(&TIM2_T2))
	{
		__set_PRIMASK(1);  
		NVIC_SystemReset();
	}
}




