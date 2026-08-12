#include "string.h"
#include "queue.h"
#include "bsp_tim2.h"
#include "bsp_spi.h"
#include "bsp_dcan.h"
#include "app_HostComm.h"
#define PACK_LEN         68
#define FRAME_HEADER     0xAA
#define FRAME_FOOTER     0xBB
#define DATA_BLOCK_SIZE  8

// 使用 __attribute__((packed)) 或 #pragma pack(1) 禁止字节对齐
#pragma pack(push, 1)
// 协议帧结构体（按实际字节顺序排列）
typedef struct 
{
    uint8_t   header;                
    uint16_t  num;          
    uint16_t  can1Id_01;   
    uint8_t   can1data_01[DATA_BLOCK_SIZE];
    uint16_t  can1Id_02;
    uint8_t   can1data_02[DATA_BLOCK_SIZE];
    uint16_t  can1Id_03;
    uint8_t   can1data_03[DATA_BLOCK_SIZE];
    uint16_t  can2Id_01;
    uint8_t   can2data_01[DATA_BLOCK_SIZE];
    uint16_t  can2Id_02;
    uint8_t   can2data_02[DATA_BLOCK_SIZE];
    uint16_t  can2Id_03;
    uint8_t   can2data_03[DATA_BLOCK_SIZE];
    uint32_t  checksum; 
    uint8_t   footer;                
} FrameStruct;
#pragma pack(pop)

// 共用体：支持字节数组到结构体的直接转换
typedef union {
    FrameStruct frame;	
    uint8_t     bytes[PACK_LEN];
} DataPack;


typedef struct
{
	int can1_fillcnt;   /*can1回传数据填充计数*/
	int can2_fillcnt;   /*can2回传数据填充计数*/
}Host_Comm;



Host_Comm  HostComm;
DataPack    rxDataPack,txDataPack;

/*
主机通讯初始化
*/
void appHostData_init(void)
{
	HostComm.can1_fillcnt=0;
	HostComm.can2_fillcnt=0;
	memset(rxDataPack.bytes,0,PACK_LEN);
	memset(txDataPack.bytes,0,PACK_LEN);
}

/*
接收数据包XOR校验和
*/
static uint32_t  DataPack_XORCheck(uint8_t*data,int len)
{
	uint32_t checksum = 0;
    uint32_t* doubleWordData = (uint32_t*)data;
    uint32_t doubleWordLen = len / 4;  

	/* 逐个 32-bit 字 XOR*/
    for (size_t i = 0; i < doubleWordLen; i++) {
        checksum ^= doubleWordData[i];  
    }
    /*qr_wl 协议特征：XOR 结果 + 0xAA*/
     return (checksum+0xAA);  
}	

/*
数据拷贝
*/ 
static inline void HostCommDataCopy(uint8_t * cdata,const uint8_t* pdata,const int len)
{
	for (size_t i = 0; i <len; i++){
		cdata[i]=pdata[i];
	}
}

/*
电机返回数据填充
Candata:队列数据
cNum: 0 can1 !0-can2
return 0
*/
int MotorDataFilling(QUEUE_DATA_T *Candata, int cNum)
{
	int clen=Candata->DLC;
	/*如果电机回数据不足8B填充0*/
	if(Candata->DLC<8)
	{
		for(int c=clen;c<DATA_BLOCK_SIZE;c++)
		{
			Candata->Data[c]=0;
		}
	}
	/*can1*/
	if(0==cNum)
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN1ID:
				HostComm.can1_fillcnt|=1;
				txDataPack.frame.can1Id_01=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can1data_01,Candata->Data,
															DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN1ID:
				HostComm.can1_fillcnt|=2;
				txDataPack.frame.can1Id_02=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can1data_02,Candata->Data,
			                                                  DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN1ID:
				HostComm.can1_fillcnt|=4;
				txDataPack.frame.can1Id_03=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can1data_03,Candata->Data,
															  DATA_BLOCK_SIZE);
				break;
		}
	}/*can1*/
	else
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN2ID:
				HostComm.can2_fillcnt|=1;
				txDataPack.frame.can2Id_01=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can2data_01,Candata->Data,
															  DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN2ID:
				HostComm.can2_fillcnt|=2;
				txDataPack.frame.can2Id_02=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can2data_02,Candata->Data,
															  DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN2ID:
				HostComm.can2_fillcnt|=4;
				txDataPack.frame.can2Id_03=Candata->StdId;
				HostCommDataCopy(txDataPack.frame.can2data_03,Candata->Data,
			                                                  DATA_BLOCK_SIZE);
				break;
		}		
	}
	
	return 0;
}
/*
查找未收到的回包填充
*/
static  void hostCommReturnFill(void)
{
	int      check_len=63;
	uint16_t fillid=0xffff;
    uint8_t  filldata[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	txDataPack.frame.num=0x0303;
	txDataPack.frame.header=FRAME_HEADER;
	txDataPack.frame.footer=FRAME_FOOTER;
	/*can1未回复的节点填充错误码*/	
	if(!(HostComm.can1_fillcnt&0x01))	
	{
		txDataPack.frame.can1Id_01=fillid;
		HostCommDataCopy(txDataPack.frame.can1data_01,filldata,8);
	}
	if(!(HostComm.can1_fillcnt&0x02))
	{
		txDataPack.frame.can1Id_02=fillid;
		HostCommDataCopy(txDataPack.frame.can1data_02,filldata,8);
	}
	if(!(HostComm.can1_fillcnt&0x04))
	{
		txDataPack.frame.can1Id_03=fillid;
		HostCommDataCopy(txDataPack.frame.can1data_03,filldata,8);
	}	
    /*can2未回复的节点填充错误码*/
	if(!(HostComm.can2_fillcnt&0x01))	
	{
		txDataPack.frame.can2Id_01=fillid;
		HostCommDataCopy(txDataPack.frame.can2data_01,filldata,8);
	}
	if(!(HostComm.can2_fillcnt&0x02))
	{
		txDataPack.frame.can2Id_02=fillid;
		HostCommDataCopy(txDataPack.frame.can2data_02,filldata,8);
	}
	if(!(HostComm.can2_fillcnt&0x04))
	{
		txDataPack.frame.can2Id_03=fillid;
		HostCommDataCopy(txDataPack.frame.can2data_03,filldata,8);
	}/*写入校验*/				
	txDataPack.frame.checksum=DataPack_XORCheck(txDataPack.bytes,check_len);
	HostComm.can1_fillcnt=0;
	HostComm.can2_fillcnt=0;
}
/*
将电机回传数据打包后填充至SPI_DMA_TxBuff
*/
void appHostData_PackagingFillTxbuff_loop(void)
{
	uint8_t * txdata_p=NULL;
	QUEUE_DATA_T 	      rxCan1={0},rxCan2={0};
	/*can1接收数据取出填充*/	
	if(QueueDe(&Can1_RevQueue,&rxCan1)){
		MotorDataFilling(&rxCan1,0);
	}
	/*can2接收数据取出填充*/
	if(QueueDe(&Can2_RevQueue,&rxCan2)){
		MotorDataFilling(&rxCan2,1);
	}
	/*时间到填充回发包*/
	if (bsp_tim2GetSta(&TIM2_T1))
	{
		hostCommReturnFill();
		if(!SPI1_RxTx.txPING){
			SPI1_RxTx.txPING=1;
			txdata_p=bsp_SPI1_GetTxBlock(SPI1_DMA_BLOCK_PING);
		}
		else if(!SPI1_RxTx.txPONG){
			SPI1_RxTx.txPONG=1;
			txdata_p=bsp_SPI1_GetTxBlock(SPI1_DMA_BLOCK_PONG);
		}
		else{
			return;
		}
		memcpy(txdata_p,txDataPack.bytes,PACK_LEN);
	}
}

/*
数据解析拆包后通过CAN下发任务轮询
*/
void appHostData_PacketDisassembly_loop(void)
{
	uint8_t * Pdata=NULL;
	uint32_t aligned_len = 63;
	uint32_t received_checksum=0;
	
	if(1==SPI1_RxTx.rxPING)
	{
	    SPI1_RxTx.rxPING=0;
	    Pdata=bsp_SPI1_GetRxBlock(SPI1_DMA_BLOCK_PING);
		received_checksum = *((uint32_t*)&Pdata[aligned_len]);
		if(DataPack_XORCheck(Pdata,aligned_len)==received_checksum){
			goto CanSendMessage;
		}
	}
	if(1==SPI1_RxTx.rxPONG)
	{
		SPI1_RxTx.rxPONG=0;
		Pdata=bsp_SPI1_GetRxBlock(SPI1_DMA_BLOCK_PONG);
		received_checksum = *((uint32_t*)&Pdata[aligned_len]);
		if(DataPack_XORCheck(Pdata,aligned_len)==received_checksum){
			goto CanSendMessage;
		}
	}
	return; 
	CanSendMessage:
	memcpy(rxDataPack.bytes,Pdata,PACK_LEN);
	bsp_CAN1_SendMessage(rxDataPack.frame.can1data_01,
						 rxDataPack.frame.can1Id_01,8);
	bsp_CAN1_SendMessage(rxDataPack.frame.can1data_02,
						 rxDataPack.frame.can1Id_02,8);
	bsp_CAN1_SendMessage(rxDataPack.frame.can1data_03,
						 rxDataPack.frame.can1Id_03,8);			
	bsp_CAN2_SendMessage(rxDataPack.frame.can2data_01,
						 rxDataPack.frame.can2Id_01,8);
	bsp_CAN2_SendMessage(rxDataPack.frame.can2data_02,
						 rxDataPack.frame.can2Id_02,8);
	bsp_CAN2_SendMessage(rxDataPack.frame.can2data_03,
						 rxDataPack.frame.can2Id_03,8);	
	bsp_tim2Start(&TIM2_T1,__WaitPackTime);	
}


















