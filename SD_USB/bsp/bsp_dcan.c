#include "main.h"
#include "fdcan.h"
#include <string.h>
#include "bsp_dcan.h"
#include "queue.h"

Queue Can1_RevQueue;
Queue Can1_SendQueue;
QUEUE_DATA_T CAN1RevBuff[CAN1REVLEN];
QUEUE_DATA_T CAN1SendBuff[CAN1SENDLEN];

Queue Can2_RevQueue;
Queue Can2_SendQueue;
QUEUE_DATA_T CAN2RevBuff[CAN1REVLEN];
QUEUE_DATA_T CAN2SendBuff[CAN1SENDLEN];

void BSP_CAN_Init(void)
{
    QueueInit(&Can1_RevQueue, CAN1RevBuff, CAN1REVLEN);
    QueueInit(&Can1_SendQueue, CAN1SendBuff, CAN1REVLEN);
    QueueInit(&Can2_RevQueue, CAN2RevBuff, CAN1REVLEN);
    QueueInit(&Can2_SendQueue, CAN2SendBuff, CAN1REVLEN);
}
/**************************************************************/
void bsp_Can1SendQueueMsg_loop(void)
{   
    QUEUE_DATA_T 	        QCanData;
    FDCAN_TxHeaderTypeDef   txHeader;
	
//	if(0==HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
//		return; 	
//	if(!QueueDe(&Can1_SendQueue,&QCanData))
//		return;
	while(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
	{ 	
		if(!QueueDe(&Can1_SendQueue,&QCanData))
			break;	
		txHeader.Identifier =QCanData.StdId;        
		txHeader.IdType   =FDCAN_STANDARD_ID;              
		txHeader.TxFrameType   =FDCAN_DATA_FRAME;               
		txHeader.DataLength   =QCanData.DLC; 
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
		txHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;           /*CAN2.0格式 */
		txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
		txHeader.MessageMarker = 0;   
		
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, QCanData.Data) != HAL_OK)
		{
			#if __DEBUG
			printf("ERR:CAN2_AddTxMessage fail\n");
			#endif
		}
	}	
}
/*
*/
void bsp_Can2SendQueueMsg_loop(void)
{   
    QUEUE_DATA_T 	        Qdata2;
    FDCAN_TxHeaderTypeDef   txHeader2;
 
//	if(0==HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2))
//		return;
//	if(!QueueDe(&Can2_SendQueue,&Qdata2))
//		return;
	while(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2))
	{
		if(!QueueDe(&Can2_SendQueue,&Qdata2))
			break;
		txHeader2.Identifier =Qdata2.StdId;        
		txHeader2.IdType   =FDCAN_STANDARD_ID;              
		txHeader2.TxFrameType   =FDCAN_DATA_FRAME;               
		txHeader2.DataLength   =Qdata2.DLC; 
		txHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
		txHeader2.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
		txHeader2.FDFormat = FDCAN_CLASSIC_CAN;           /*CAN2.0格式 */
		txHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
		txHeader2.MessageMarker = 0;   
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader2, Qdata2.Data) != HAL_OK)
		{
			#if __DEBUG
			printf("ERR:CAN2_AddTxMessage fail\n");
			#endif
		}	
	}	
}
void bsp_CAN1_SendMessage(const uint8_t *data,const uint32_t id, const uint32_t len)
{
  QUEUE_DATA_T 	        endata;
//  FDCAN_TxHeaderTypeDef CAN1_TxHeader;

//  CAN1_TxHeader.Identifier =id;        
//  CAN1_TxHeader.IdType   =FDCAN_STANDARD_ID;              
//  CAN1_TxHeader.TxFrameType   =FDCAN_DATA_FRAME;               
//  CAN1_TxHeader.DataLength   =len; 
//  CAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
//  CAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
//  CAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;           /* CAN2.0格式 */
//  CAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
//  CAN1_TxHeader.MessageMarker = 0;  	
  /*入队数据*/
  endata.StdId=id;
  endata.DLC=len;
  for(int i=0;i<len;i++)
  {
       endata.Data[i]=data[i];
  }	
//  if(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
//  {
//	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN1_TxHeader, data) != HAL_OK)
//	{
//		#if __DEBUG
//		printf("ERR:CAN1_SendMessage fail\n");
//		#endif
//	}
//  }
//  else
//  {
	  QueueEn(&Can1_SendQueue,endata);
//  }
	bsp_Can1SendQueueMsg_loop();
}
/*
*/
void bsp_CAN2_SendMessage(const uint8_t *data,const uint32_t id, const uint32_t len)
{
  QUEUE_DATA_T 	      endata2;
//  FDCAN_TxHeaderTypeDef CAN2_TxHeader;
	
//  CAN2_TxHeader.Identifier =id;        
//  CAN2_TxHeader.IdType   =FDCAN_STANDARD_ID;              
//  CAN2_TxHeader.TxFrameType   =FDCAN_DATA_FRAME;               
//  CAN2_TxHeader.DataLength   =len; 
//  CAN2_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
//  CAN2_TxHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
//  CAN2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;           /* CAN2.0格式 */
//  CAN2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
//  CAN2_TxHeader.MessageMarker = 0;  
  /*入队数据*/
  endata2.StdId=id;
  endata2.DLC=len;
  for(int i=0;i<len;i++)
  {
	  endata2.Data[i]=data[i];
  }	
//  if(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2))
//  {
//	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN2_TxHeader, data) != HAL_OK)
//	{
//		#if __DEBUG
//		printf("ERR:CAN2_SendMessage fail\n");
//		#endif
//	}
//  }
//  else
//  {
	QueueEn(&Can2_SendQueue,endata2);
//  }
    bsp_Can2SendQueueMsg_loop();	
}

/*
*/
static void CanDataTransferToQueue(FDCAN_HandleTypeDef *CanHand,
                                  FDCAN_RxHeaderTypeDef *CanRxTyp,
                                  uint8_t *CanRevData)
{
    QUEUE_DATA_T rxEndata = {0};
    rxEndata.DLC = (CanRxTyp->DataLength);
    rxEndata.StdId = CanRxTyp->Identifier;
    for (uint8_t i = 0; i < rxEndata.DLC; i++)
    {
        rxEndata.Data[i] = CanRevData[i];
    }

    if (CanHand == &hfdcan1)
    {
        QueueEn(&Can1_RevQueue, rxEndata);
    }
    else if (CanHand == &hfdcan2)
    {
        QueueEn(&Can2_RevQueue, rxEndata);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        if (rxHeader.IdType == FDCAN_STANDARD_ID)
        {

            CanDataTransferToQueue(hfdcan, &rxHeader, rxData);
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader, rxData);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        if (rxHeader.IdType == FDCAN_STANDARD_ID)
        {
            CanDataTransferToQueue(hfdcan, &rxHeader, rxData);
        }
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}
