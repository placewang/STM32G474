#include "main.h"
#include "can.h"
#include <stdio.h>
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

void bsp_canQueue_int(void)
{
	QueueInit(&Can1_RevQueue,CAN1RevBuff,CAN1REVLEN);
	QueueInit(&Can1_SendQueue,CAN1SendBuff,CAN1REVLEN);
	QueueInit(&Can2_RevQueue,CAN2RevBuff,CAN1REVLEN);
	QueueInit(&Can2_SendQueue,CAN2SendBuff,CAN1REVLEN);
}
/**************************************************************/
void bsp_Can1SendQueueMsg_loop(void)
{   
    QUEUE_DATA_T 	      Qdata;
    CAN_TxHeaderTypeDef   txHeader;
    uint32_t              txMailbox;
	
	txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.TransmitGlobalTime = DISABLE;
	
    while (0!=HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
    {
      if(!QueueDe(&Can1_SendQueue,&Qdata))
        break;
      txHeader.StdId = Qdata.StdId;
      txHeader.DLC = Qdata.DLC;   
      if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, Qdata.Data, &txMailbox) != HAL_OK)
      {
        #if __DEBUG
        printf("ERR:CAN1_AddTxMessage fail\n");
        #endif
      }		
    }
}	

void bsp_Can2SendQueueMsg_loop(void)
{   
    QUEUE_DATA_T 	      Qdata2;
    CAN_TxHeaderTypeDef   txHeader2;
    uint32_t              txMailbox2;
 
	txHeader2.IDE = CAN_ID_STD;
    txHeader2.RTR = CAN_RTR_DATA;
    txHeader2.TransmitGlobalTime = DISABLE;
    while (0!=HAL_CAN_GetTxMailboxesFreeLevel(&hcan2))
    {
      if(!QueueDe(&Can2_SendQueue,&Qdata2))
        break;
      txHeader2.StdId = Qdata2.StdId;
      txHeader2.DLC = Qdata2.DLC;
      if(HAL_CAN_AddTxMessage(&hcan2, &txHeader2, Qdata2.Data, &txMailbox2) != HAL_OK)
      {
        #if __DEBUG
        printf("ERR:CAN2_AddTxMessage fail\n");
        #endif
      }		
    }
}

void bsp_CAN1_SendMessage(const uint8_t *data,const uint32_t id, const uint32_t len)
{
  QUEUE_DATA_T 	      endata;

  endata.StdId=id;
  endata.DLC=len;
  for(int i=0;i<len;i++)
  {
       endata.Data[i]=data[i];
  }	
  QueueEn(&Can1_SendQueue,endata);
  bsp_Can1SendQueueMsg_loop();
}

void bsp_CAN2_SendMessage(const uint8_t *data,const uint32_t id, const uint32_t len)
{
  QUEUE_DATA_T 	      endata2;

  endata2.StdId=id;
  endata2.DLC=len;
  for(int i=0;i<len;i++)
  {
	  endata2.Data[i]=data[i];
  }	
  QueueEn(&Can2_SendQueue,endata2);
  bsp_Can2SendQueueMsg_loop();	
}

static void  CanDataTransferToQueue(CAN_HandleTypeDef  *CanHand,
	                                CAN_RxHeaderTypeDef *CanRxTyp,
                                    uint8_t *CanRevData)
{
    QUEUE_DATA_T 	      rxEndata={0};
    rxEndata.DLC=CanRxTyp->DLC;
	  rxEndata.StdId=CanRxTyp->StdId;
    for(uint8_t i=0;i<CanRxTyp->DLC;i++)    
    {
        rxEndata.Data[i]=CanRevData[i];
    }
	
    if (CanHand->Instance == CAN1)
    {
		  QueueEn(&Can1_RevQueue,rxEndata);
    }
	if(CanHand->Instance == CAN2)
	{
		  QueueEn(&Can2_RevQueue,rxEndata);
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    return;
  }
  CanDataTransferToQueue(hcan,&rxHeader,rxData);

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t rxData[8];
  CAN_RxHeaderTypeDef rxHeader;
 
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) != HAL_OK){
	  return;
  }
  CanDataTransferToQueue(hcan,&rxHeader,rxData);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t error_code = HAL_CAN_GetError(hcan);
  if (hcan->Instance == CAN1)
  {
	#if __DEBUG
		printf("ERR:CAN1Bus %d\n",error_code);
	#endif
  }
  if (hcan->Instance == CAN2)
  {
	#if __DEBUG
		printf("ERR:CAN2Bus %d\n",error_code);
	#endif
  }
}




