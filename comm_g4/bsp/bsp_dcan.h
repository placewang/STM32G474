#ifndef _BSP_DCAN_H_
#define _BSP_DCAN_H_



void BSP_CAN_Init(void);
void bsp_CAN1_SendMessage(const unsigned char * data,\
                         const unsigned int id,\
					     const unsigned int len);
void bsp_CAN2_SendMessage(const unsigned char * data,\
                         const unsigned int id,\
					     const unsigned int len);						 
void bsp_Can1SendQueueMsg_loop(void);
void bsp_Can2SendQueueMsg_loop(void);

#endif

