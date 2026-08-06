#include "fdcan.h"

#include "initcall.h"
#include "platform/platform_can_bridge.h"

#define  True  1
#define  False 0
#define  CAN1REVLEN                     100
#define  CAN1SENDLEN                    32

#define  CAN2REVLEN                     CAN1REVLEN
#define  CAN2SENDLEN                    CAN1SENDLEN

typedef struct 
{  
  unsigned int    DLC;    
  unsigned int  StdId;   
  unsigned char Data[8];

}CAN_RecvTypeDef;


#define QUEUE_DATA_T  CAN_RecvTypeDef    //队列数据类型定义


// 定义循环队列结构体
typedef struct Queue{
	unsigned int front;	  //队列头下标
	unsigned int rear;	  //队列尾下标
	unsigned int MAXSIZE; //队列缓存长度（初始化时赋值）
	QUEUE_DATA_T *data;
}Queue;

static Queue Can1_RevQueue;
static Queue Can1_SendQueue;
static QUEUE_DATA_T CAN1RevBuff[CAN1REVLEN];
static QUEUE_DATA_T CAN1SendBuff[CAN1SENDLEN];

static Queue Can2_RevQueue;
static Queue Can2_SendQueue;
static QUEUE_DATA_T CAN2RevBuff[CAN2REVLEN];
static QUEUE_DATA_T CAN2SendBuff[CAN2SENDLEN];


static  platform_can_device _stm32_can1_bus;
static  platform_can_device _stm32_can2_bus;


/*
 * 初始化循环队列 
*/
static void QueueInit(Queue* q,QUEUE_DATA_T * buffer, unsigned int len)
{
    q -> front =0;
	q -> rear = 0;
	q->MAXSIZE=len;
	q->data=buffer;
}
/**  循环队列判满q 循环队列*/
static int QueueFull(Queue *q)
{
    return (q -> rear + 1) % q->MAXSIZE == q -> front;
}

/*  循环队列判空q 循环队列*/
static int QueueEmpty(Queue *q)
{
    return q -> front == q -> rear;
}

/* *  计算循环队列长度 q 循环队列*/
static  uint32_t QueueLength(Queue *q)
{
    return (q -> rear - q -> front + q->MAXSIZE) % q->MAXSIZE;
}

/*循环队列 入队 q 循环队列 data 入队元素*/
static int QueueEn(Queue* q, QUEUE_DATA_T data){
    if(QueueFull(q)){
        return False;
    }
   // 队尾入队
    q ->data[q -> rear] = data;
    // 更新队尾指针
    q -> rear = (q -> rear+1 )%q->MAXSIZE;
    return True;
}

/* 出队  q 循环队列 val 用来存出队元素的数据*/
static int QueueDe(Queue* q, QUEUE_DATA_T *val)
{
    if(QueueEmpty(q)){
        return False;
    }
    // 队头元素出队
    *val = q ->data[q -> front];
    // 更新队头指针
    q -> front = (q -> front + 1) % q->MAXSIZE;
    return True;
}

void _stm32_CAN_Init(void)
{
    QueueInit(&Can1_RevQueue, CAN1RevBuff, CAN1REVLEN);
    QueueInit(&Can1_SendQueue, CAN1SendBuff, CAN1SENDLEN); 
    QueueInit(&Can2_RevQueue, CAN2RevBuff, CAN2REVLEN);
    QueueInit(&Can2_SendQueue, CAN2SendBuff, CAN2SENDLEN);  
}

/**/
static void Can_SendQueue_POP(const FDCAN_HandleTypeDef *hfdcan,Queue * SendQueue)
{   
    QUEUE_DATA_T 	        QCanData;
    FDCAN_TxHeaderTypeDef   txHeader;
    uint32_t c1max_send = 3;  // 限制单次中断
	
	while(0!=HAL_FDCAN_GetTxFifoFreeLevel(hfdcan)&&(c1max_send--)>0)
	{ 	
        /*先检查队列是否为空*/
        if (QueueEmpty(SendQueue))
            break;	
		/*先查看队头数据(不出队)*/
        QCanData = SendQueue->data[SendQueue->front];	
		/* 准备发送头*/
		txHeader.Identifier =QCanData.StdId;        
		txHeader.IdType   =FDCAN_STANDARD_ID;              
		txHeader.TxFrameType   =FDCAN_DATA_FRAME;               
		txHeader.DataLength   =QCanData.DLC; 
		txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
		txHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
		txHeader.FDFormat = FDCAN_CLASSIC_CAN;           /*CAN2.0格式 */
		txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
		txHeader.MessageMarker = 0;   
		
		if(HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)hfdcan, &txHeader, QCanData.Data) == HAL_OK)
		{
			QueueDe(SendQueue,&QCanData);
		}
		else
		{/* 发送失败，保留在队头，退出循环等待下次机会*/
			break;
		}
	}	
}
/**/
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
		Can_SendQueue_POP(hfdcan,&Can1_SendQueue);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        Can_SendQueue_POP(hfdcan,&Can2_SendQueue);
    }
}
/**/
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
        if (rxHeader.IdType == FDCAN_STANDARD_ID)
        {
            CanDataTransferToQueue(hfdcan, &rxHeader, rxData);
        }
    }
}
/**/
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rxHeader, rxData);
        if (rxHeader.IdType == FDCAN_STANDARD_ID)
        {
            CanDataTransferToQueue(hfdcan, &rxHeader, rxData);
        }
    }
}
/**/
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
	uint32_t error_status;
	error_status = HAL_FDCAN_GetError(hfdcan);
   
    if (hfdcan->Instance == FDCAN1)
	{
		_stm32_can1_bus.ErrCode=error_status;
	} 
	else if (hfdcan->Instance == FDCAN2)
	{
		_stm32_can2_bus.ErrCode=error_status;
	}    	
}
/**/
static void _stm32CAN1_SendData(const uint8_t *data,const uint32_t id, const uint32_t len)
{
  uint32_t primask=0;
  QUEUE_DATA_T 	        endata;
  FDCAN_TxHeaderTypeDef CAN1_TxHeader;
  if(data==NULL&&len>64)
	return;
  CAN1_TxHeader.Identifier =id;        
  CAN1_TxHeader.IdType   =FDCAN_STANDARD_ID;              
  CAN1_TxHeader.TxFrameType   =FDCAN_DATA_FRAME;               
  CAN1_TxHeader.DataLength   =len; 
  CAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
  CAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
  CAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;           /* CAN2.0格式 */
  CAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
  CAN1_TxHeader.MessageMarker = 0;  	
  /*入队数据*/
  endata.StdId=id;
  endata.DLC=len;
  for(uint32_t i=0;i<len;i++)
  {
       endata.Data[i]=data[i];
  }	
  primask = __get_PRIMASK();
  __disable_irq();  
  if(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1)&&0==QueueLength(&Can1_SendQueue))
  {
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN1_TxHeader, data) != HAL_OK)
	{
		QueueEn(&Can1_SendQueue,endata);
	}
  }
  else
  {
	  QueueEn(&Can1_SendQueue,endata);

	  Can_SendQueue_POP(&hfdcan1,&Can1_SendQueue);
	  
  }
  __set_PRIMASK(primask);
  
}
/*
*/
static  void _stm32CAN2_SendData(const uint8_t *data,const uint32_t id, uint32_t len)
{
  uint32_t primask2=0;
  QUEUE_DATA_T 	      endata2;
  FDCAN_TxHeaderTypeDef CAN2_TxHeader;
  
  if(data==NULL&&len>64)
	return;	
  if(len>64)
	len=64;
	
  CAN2_TxHeader.Identifier =id;        
  CAN2_TxHeader.IdType   =FDCAN_STANDARD_ID;              
  CAN2_TxHeader.TxFrameType   =FDCAN_DATA_FRAME;               
  CAN2_TxHeader.DataLength   =len; 
  CAN2_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; /* 设置错误状态指示 */
  CAN2_TxHeader.BitRateSwitch = FDCAN_BRS_ON;           /* 开启可变波特率 */
  CAN2_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;           /* CAN2.0格式 */
  CAN2_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;/* 用于发送事件FIFO控制, 不存储 */
  CAN2_TxHeader.MessageMarker = 0;  
  /*入队数据*/
  endata2.StdId=id;
  endata2.DLC=len;
  for(uint32_t i=0;i<len;i++)
  {
	  endata2.Data[i]=data[i];
  }	
  primask2 = __get_PRIMASK();
  __disable_irq();  
  if(0!=HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2)&&0==QueueLength(&Can2_SendQueue))
  {
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN2_TxHeader, data) != HAL_OK)
	{
		QueueEn(&Can2_SendQueue,endata2);
	}
  }
  else
  {
	QueueEn(&Can2_SendQueue,endata2);	
	Can_SendQueue_POP(&hfdcan2,&Can2_SendQueue);	
  }
  __set_PRIMASK(primask2);   
}
static int _stm32CAN1_ReceiveData(uint8_t *data,uint32_t *id, uint32_t *len)
{
  QUEUE_DATA_T c1_rxdata = {0};
  
  if(!data || !id || !len)
	return -1;
	
  if(QueueDe(&Can1_RevQueue,&c1_rxdata))
  {
	  for(uint32_t i=0;i<c1_rxdata.DLC;i++)
	  {
		   data[i]=c1_rxdata.Data[i];
	  }	
	  *id=c1_rxdata.StdId;
	  *len=c1_rxdata.DLC;
  }
  else
  {
	return 0;
  }
  return 1;
}
static int _stm32CAN2_ReceiveData(uint8_t *data,uint32_t *id, uint32_t *len)
{
  QUEUE_DATA_T c2_rxdata = {0};
  
  if(!data || !id || !len)
	return -1;
	
  if(QueueDe(&Can2_RevQueue,&c2_rxdata))
  {
	  for(uint32_t i=0;i<c2_rxdata.DLC;i++)
	  {
		   data[i]=c2_rxdata.Data[i];
	  }	
	  *id=c2_rxdata.StdId;
	  *len=c2_rxdata.DLC;
  }
  else
  {
	return 0;
  }
  return 1;
}
static const  platform_can_ops _stm32_can1_ops = 
{
	.send=_stm32CAN1_SendData,
	.receive=_stm32CAN1_ReceiveData
};
static const  platform_can_ops _stm32_can2_ops = 
{
	.send=_stm32CAN2_SendData,
	.receive=_stm32CAN2_ReceiveData
};



void platform_hw_can_init(void);

void platform_hw_can_init(void)
{
	(void)platform_can_bus_register(&_stm32_can1_ops, &_stm32_can1_bus,CAN1);
	(void)platform_can_bus_register(&_stm32_can2_ops, &_stm32_can2_bus,CAN2);
}

