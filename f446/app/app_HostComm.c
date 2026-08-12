#include "string.h"
#include "queue.h"
#include "bsp_tim2.h"
#include "bsp_spi.h"
#include "bsp_dcan.h"
#include "app_HostComm.h"


typedef enum
{
    MTR1_CAN1ID=0x01,
    MTR2_CAN1ID=0x02,  
    MTR3_CAN1ID=0x03,
    MTR4_CAN1ID=0x04,	
    MTR1_CAN2ID=0x01,
    MTR2_CAN2ID=0x02,  
    MTR3_CAN2ID=0x03,
    MTR4_CAN2ID=0x04    	
}MOTORID;

Host_Comm        HostComm;

Host_DataPack    rxDataPack,txDataPack;


uint32_t XORCheck(uint8_t *data, int len);



/* ----- 填充发送帧的固定字段并计算校验和 ----- */
void hostCommReturnFill(uint8_t dev_type)
{
    int      check_len=0;
	
	if(FOOT_DEV_TYPE==dev_type)
	{
		check_len=FOOT_PACK_LEN-5;
		txDataPack.Foot.num=0x0303;
		txDataPack.Foot.header=FRAME_HEADER;
		txDataPack.Foot.footer=FRAME_FOOTER;
		/*写入校验*/				
		txDataPack.Foot.checksum=XORCheck(txDataPack.bytes,check_len);
	}
	else if(WHEEL_DEV_TYPE==dev_type)
	{
		check_len=WHEEL_PACK_LEN-5;
		txDataPack.Wheel.num=0x0404;
		txDataPack.Wheel.header=FRAME_HEADER;
		txDataPack.Wheel.footer=FRAME_FOOTER;
		/*写入校验*/				
		txDataPack.Wheel.checksum=XORCheck(txDataPack.bytes,check_len);		
	}
}

/*
主机通讯初始化
*/
void app_HostData_init(void)
{
	HostComm.RecFlag=0;
	HostComm.type=FOOT_DEV_TYPE;	


	memset(rxDataPack.bytes,0,WHEEL_PACK_LEN);
    memset(txDataPack.bytes,0xFF,WHEEL_PACK_LEN);
	
	hostCommReturnFill(FOOT_DEV_TYPE);	
   
	SPI1_RxTx.tx_reality_len=FOOT_PACK_LEN;
	bsp_SPI1_SubmitTxData(txDataPack.bytes);
}

/*
数据拷贝
*/ 
static inline void HostCommDataCopy(uint8_t * cdata,const uint8_t* pdata,const int len)
{
    switch (len)
    {
        case 8: cdata[7] = pdata[7]; 
        case 7: cdata[6] = pdata[6];
        case 6: cdata[5] = pdata[5];
        case 5: cdata[4] = pdata[4];
        case 4: cdata[3] = pdata[3];
        case 3: cdata[2] = pdata[2];
        case 2: cdata[1] = pdata[1];
        case 1: cdata[0] = pdata[0];
        case 0: break;  // len=0 或执行完上述赋值后退出
        default: break;
    }	
}


/*足式CAN数据填充*/
void FootCanDataFilling(FootFrameStruct *Foot_type,QUEUE_DATA_T *Candata, int cNum)
{
	if(0==cNum)
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN1ID:
				HostComm.can1_fillcnt|=1;
				Foot_type->can1Id_01=Candata->StdId;
				HostCommDataCopy(Foot_type->can1data_01,Candata->Data,DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN1ID:
				HostComm.can1_fillcnt|=2;
				Foot_type->can1Id_02=Candata->StdId;
				HostCommDataCopy(Foot_type->can1data_02,Candata->Data,DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN1ID:
				HostComm.can1_fillcnt|=4;
				Foot_type->can1Id_03=Candata->StdId;
				HostCommDataCopy(Foot_type->can1data_03,Candata->Data, DATA_BLOCK_SIZE);
				break;
		}
	}/*can2*/
	else
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN2ID:
				HostComm.can2_fillcnt|=1;
				Foot_type->can2Id_01=Candata->StdId;
				HostCommDataCopy(Foot_type->can2data_01,Candata->Data,DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN2ID:
				HostComm.can2_fillcnt|=2;
				Foot_type->can2Id_02=Candata->StdId;
				HostCommDataCopy(Foot_type->can2data_02,Candata->Data,DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN2ID:
				HostComm.can2_fillcnt|=4;
				Foot_type->can2Id_03=Candata->StdId;
				HostCommDataCopy(Foot_type->can2data_03,Candata->Data,DATA_BLOCK_SIZE);
				break;
		}		
	}	
}
/*轮式CAN数据填充*/
void WheelCanDataFilling(wheelFrameStruct *Wheel_type,QUEUE_DATA_T *Candata, int cNum)
{
	if(0==cNum)
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN1ID:
				HostComm.can1_fillcnt|=1;
				Wheel_type->can1Id_01=Candata->StdId;
				HostCommDataCopy(Wheel_type->can1data_01,Candata->Data,DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN1ID:
				HostComm.can1_fillcnt|=2;
				Wheel_type->can1Id_02=Candata->StdId;
				HostCommDataCopy(Wheel_type->can1data_02,Candata->Data,DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN1ID:
				HostComm.can1_fillcnt|=4;
				Wheel_type->can1Id_03=Candata->StdId;
				HostCommDataCopy(Wheel_type->can1data_03,Candata->Data, DATA_BLOCK_SIZE);
				break;
			case MTR4_CAN1ID:
				HostComm.can1_fillcnt|=8;
				Wheel_type->can1Id_04=Candata->StdId;
				HostCommDataCopy(Wheel_type->can1data_04,Candata->Data, DATA_BLOCK_SIZE);
				break;			
		}
	}/*can2*/
	else
	{
		switch (Candata->StdId)
		{
			case MTR1_CAN2ID:
				HostComm.can2_fillcnt|=1;
				Wheel_type->can2Id_01=Candata->StdId;
				HostCommDataCopy(Wheel_type->can2data_01,Candata->Data,DATA_BLOCK_SIZE);
				break;
			case MTR2_CAN2ID:
				HostComm.can2_fillcnt|=2;
				Wheel_type->can2Id_02=Candata->StdId;
				HostCommDataCopy(Wheel_type->can2data_02,Candata->Data,DATA_BLOCK_SIZE);			
				break;
			case MTR3_CAN2ID:
				HostComm.can2_fillcnt|=4;
				Wheel_type->can2Id_03=Candata->StdId;
				HostCommDataCopy(Wheel_type->can2data_03,Candata->Data,DATA_BLOCK_SIZE);
				break;
			case MTR4_CAN2ID:
				HostComm.can2_fillcnt|=8;
				Wheel_type->can2Id_04=Candata->StdId;
				HostCommDataCopy(Wheel_type->can2data_04,Candata->Data,DATA_BLOCK_SIZE);
				break;			
		}		
	}	
}


/*
电机返回数据填充
Candata:队列数据
cNum: 0 can1 !0-can2
return 0
*/
int MotorDataFilling(uint8_t F_type,QUEUE_DATA_T *Candata, int cNum)
{
	int clen=Candata->DLC;

	/*如果电机回数据不足8B填充0*/
	if(Candata->DLC<8)
	{
		for(int c=clen;c<DATA_BLOCK_SIZE;c++)
		{
			Candata->Data[c]=0;
		}
	}/*兼容海泰电机*/
	if(0x00==Candata->StdId)
	{
		Candata->StdId=Candata->Data[0];
	}
	if (FOOT_DEV_TYPE == F_type)
	{
		FootCanDataFilling(&txDataPack.Foot,Candata,cNum);
	}
	else if (WHEEL_DEV_TYPE == F_type) 
	{
		WheelCanDataFilling(&txDataPack.Wheel,Candata,cNum);
	}		
	
	return 0;
}


void bsp_SPI1_FillPack_XORCheck(uint8_t * SPItxData,const uint8_t len)
{
	int      check_len=0;
	Host_DataPack *txData=(Host_DataPack *)SPItxData;
	if(FOOT_DEV_TYPE==HostComm.type)
	{
		check_len=len-5;
		txData->Foot.header=FRAME_HEADER;
		txData->Foot.footer=FRAME_FOOTER;
		/*写入校验*/				
		txData->Foot.checksum=XORCheck(txData->bytes,check_len);
	}
	else if(WHEEL_DEV_TYPE==HostComm.type)
	{
		check_len=len-5;
		txData->Wheel.header=FRAME_HEADER;
		txData->Wheel.footer=FRAME_FOOTER;
		/*写入校验*/				
		txData->Wheel.checksum=XORCheck(txData->bytes,check_len);
	}	
}



void SendRxCanData(uint8_t S_type)
{
	if(FOOT_DEV_TYPE==S_type)
	{
		bsp_CAN1_SendMessage(rxDataPack.Foot.can1data_01,
							 rxDataPack.Foot.can1Id_01,8);
		bsp_CAN1_SendMessage(rxDataPack.Foot.can1data_02,
							 rxDataPack.Foot.can1Id_02,8);
		bsp_CAN1_SendMessage(rxDataPack.Foot.can1data_03,
							 rxDataPack.Foot.can1Id_03,8);			
		bsp_CAN2_SendMessage(rxDataPack.Foot.can2data_01,
							 rxDataPack.Foot.can2Id_01,8);
		bsp_CAN2_SendMessage(rxDataPack.Foot.can2data_02,
							 rxDataPack.Foot.can2Id_02,8);
		bsp_CAN2_SendMessage(rxDataPack.Foot.can2data_03,
							 rxDataPack.Foot.can2Id_03,8);	
	}
	else if(WHEEL_DEV_TYPE==S_type)
	{
		bsp_CAN1_SendMessage(rxDataPack.Wheel.can1data_01,
							 rxDataPack.Wheel.can1Id_01,8);
		bsp_CAN1_SendMessage(rxDataPack.Wheel.can1data_02,
							 rxDataPack.Wheel.can1Id_02,8);
		bsp_CAN1_SendMessage(rxDataPack.Wheel.can1data_03,
							 rxDataPack.Wheel.can1Id_03,8);	
		bsp_CAN1_SendMessage(rxDataPack.Wheel.can1data_04,
							 rxDataPack.Wheel.can1Id_04,8);			
		bsp_CAN2_SendMessage(rxDataPack.Wheel.can2data_01,
							 rxDataPack.Wheel.can2Id_01,8);
		bsp_CAN2_SendMessage(rxDataPack.Wheel.can2data_02,
							 rxDataPack.Wheel.can2Id_02,8);
		bsp_CAN2_SendMessage(rxDataPack.Wheel.can2data_03,
							 rxDataPack.Wheel.can2Id_03,8);
		bsp_CAN2_SendMessage(rxDataPack.Wheel.can2data_04,
							 rxDataPack.Wheel.can2Id_04,8);		
	}
}

/*
数据解析拆包后通过CAN下发任务轮询
*/
void app_HostDataPacketDisassembly_loop(void)
{
    QUEUE_DATA_T  rxCan1={0},rxCan2={0};
    static int    last_can1Fillcnt=0;
	static int    last_can2Fillcnt=0;
	if(0!=HostComm.RecFlag)
	{
		HostComm.RecFlag=0;
		last_can1Fillcnt = 0;
		last_can2Fillcnt = 0;
		HostComm.can1_fillcnt=0;
		HostComm.can2_fillcnt=0;
		SendRxCanData(HostComm.type);
		/*清空*/
		memset(txDataPack.bytes,0xff,WHEEL_PACK_LEN);
		hostCommReturnFill(HostComm.type);	
		bsp_SPI1_SubmitTxData(txDataPack.bytes);
	}
	bsp_Can1SendQueueMsg_loop();
	bsp_Can2SendQueueMsg_loop();	
	__disable_irq();
	/*can1接收数据取出填充*/	
	while(QueueDe(&Can1_RevQueue,&rxCan1))
	{
		MotorDataFilling(HostComm.type,&rxCan1,0);
	}
	/*can2接收数据取出填充*/
	while(QueueDe(&Can2_RevQueue,&rxCan2))
	{
		MotorDataFilling(HostComm.type,&rxCan2,1);
	}	
	 __enable_irq();
	if(0!=HostComm.can1_fillcnt&&
	   (HostComm.can1_fillcnt!=last_can1Fillcnt)
      )
	{
		SPI1_RxTx.dma_running =0;	
		last_can1Fillcnt=HostComm.can1_fillcnt;
		bsp_SPI1_SubmitTxData(txDataPack.bytes);
	}
	if(0!=HostComm.can2_fillcnt&&
	   (HostComm.can2_fillcnt!=last_can2Fillcnt)
      )
	{
		SPI1_RxTx.dma_running =0;	
		last_can2Fillcnt=HostComm.can2_fillcnt;
		bsp_SPI1_SubmitTxData(txDataPack.bytes);
	}	

}








