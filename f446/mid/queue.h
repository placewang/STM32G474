#ifndef __QUEUE__H
#define __QUEUE__H

#include "can.h"

#define  True  1
#define  False 0
#define  CAN1REVLEN                     100
#define  CAN1SENDLEN                    100

typedef struct 
{
  unsigned int  StdId;            
  unsigned int  DLC;      
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



extern Queue Can1_RevQueue;	    			  
extern Queue Can1_SendQueue;	    			  
extern QUEUE_DATA_T CAN1RevBuff[CAN1REVLEN];	  
extern QUEUE_DATA_T CAN1SendBuff[CAN1SENDLEN];

extern Queue Can2_RevQueue;	    			  
extern Queue Can2_SendQueue;	    			  
extern QUEUE_DATA_T CAN2RevBuff[CAN1REVLEN];	  
extern QUEUE_DATA_T CAN2SendBuff[CAN1SENDLEN];

void QueueInit(Queue* q,QUEUE_DATA_T * buffer, unsigned int len);
int  QueueFull(Queue *q);
int  QueueDe(Queue* , QUEUE_DATA_T *);
int  QueueEn(Queue* , QUEUE_DATA_T );
int  QueueLength(Queue *q);


#endif




