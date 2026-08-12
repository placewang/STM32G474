#ifndef _APP_IAPUPDATA_H__
#define _APP_IAPUPDATA_H__

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_OPERATION_FLAG_BASS   0x08008000UL  /*标志存储区地址*/
#define FLASH_RUNING_AREA_APP       0x0800C000UL  /* A运行区地址 */
#define FLASH_RUNING_AREA_BACKUP    0x08040000UL  /* B运行区地址*/


#define PACK_UPDATA_LEN   68
#define UPDATA_LEN        54
#define UPDATA_HEADER     0xAA
#define UPDATA_FOOTER     0xBB


typedef enum
{
    Update_Request=0xff,   /*升级请求进入升级状态*/
    Update_Data=0xfe,      /*数据包，写入flash(满512by) */
    Update_LastPack=0xfd,  /*最后一包数据,校验后返回成功或失败*/
    Update_Success=0xfc,   /*返回升级成功*/
    Update_Fail=0xfb,      /*返回升级失败*/   
	Update_ACK =0xfa       /*应答返回*/   	
}UPDATA_CMD;

// 使用 __attribute__((packed)) 或 #pragma pack(1) 禁止字节对齐
#pragma pack(push, 1)
// 协议帧结构体（按实际字节顺序排列）
typedef struct 
{
    unsigned char    header;                		 /*协议头*/	
    unsigned short   cmd;                            /*指令码*/     
    unsigned int     packnum;                        /*数据包号*/	
    unsigned short   len;                            /*数据长度*/
	unsigned char    data[UPDATA_LEN];               /*数据*/
	unsigned int     checksum;                       /*校验和*/
    unsigned char    footer;                         /*协议尾*/
} IAPStruct;
#pragma pack(pop)

// 共用体：支持字节数组到结构体的直接转换
typedef union {
    IAPStruct frame;	
    unsigned char     bytes[PACK_UPDATA_LEN];
} UpDataPack;


//升级数据缓存
typedef union {
   unsigned char     data[512];			      	
    unsigned int     flash[128];
} Rxbuff;

// 
typedef struct 
{
   volatile char          TxFlag;                     /*数据回传发送标记*/	
   volatile char          RecFlag;                    /*数据到位标记*/
   unsigned char *        data_p;                     /*数据地址*/	
   signed char            retry_state;			      /*接收状态 1：满512字节  2:最后一包 收到一包 3 -1：接收超时  */
   unsigned short         retry_timer;			      /*接收计时超时 退出升级跳转APP*/
   unsigned short         rx_buf_len;			      /*当前的接收长度*/
   Rxbuff                 rx_buf;		    	      /*接收到的包（满存一次备份区FLASH*/	
   unsigned int           recv_all_data_len;	      /*收到的数据总长度*/
   unsigned int           flash_address;		      /*当前操作的flash地址*/	
   volatile unsigned int  OperationFlag;              /*运行区标志*/
   volatile unsigned int  crc32;                      /*整个文件的检验*/
   volatile unsigned int  FileSize;                   /*整个文件有效数据的字节总数*/  
   volatile unsigned int  SoftwareVersion;            /*软件版本*/ 
   volatile unsigned int  HardwareVersion;            /*硬件版本*/	
}UpgradeProcess;


extern UpgradeProcess  iap_ps;
extern UpDataPack    iapRxbuff,iapTxbuff;
void app_IapUpData_init(void);
void app_UpDataPacketDisassembly_loop(void);
void app_IapReturnPackageFill(void);

#ifdef __cplusplus
}
#endif

#endif
