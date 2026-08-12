#ifndef _BSP_USARTQUE_H__
#define _BSP_USARTQUE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define USART_DMA_BLOCK_SIZE 512U
#define USART_DMA_BLOCK_COUNT 1U
#define UART_BUF_SIZE   (USART_DMA_BLOCK_SIZE*USART_DMA_BLOCK_COUNT)


typedef struct USART_DMA
{
    /* ---------- 接收队列 ---------- */
    volatile unsigned int rxFront;	        /*队列头下标*/
	volatile unsigned int rxRear;	        /*队列尾下标*/
	volatile unsigned int rxMAXSIZE;        /*队列缓存长度（初始化时赋值）*/
    volatile unsigned int rxOffset;         /*跟踪上次处理到的 DMA 缓冲区偏移*/ 
	volatile unsigned int overflow_cnt;     /*记录溢出*/
	unsigned char        rxQueuefifo[UART_BUF_SIZE];
    /*发送*/
	volatile unsigned char   sendFlag_end;     /*发送结束标志*/
	
	/* ---------- 错误恢复 ---------- */
	volatile uint8_t restart_pending;  /* DMA 传输是否正在运行 */
	volatile uint8_t dma_running;      /* 请求重新启动 DMA 传输 */ 
}USART_DMA;



int QueLen(USART_DMA *q);
void copy_to_queue_from_dma(USART_DMA *uart_que,uint8_t *data, uint32_t len);
void QueueCopyOut(USART_DMA *q, uint32_t start, uint8_t *buf, uint16_t len);
void update_dma_data(USART_DMA *uart_Que,uint8_t *DMA_RXbuff,uint32_t current_offset);

#ifdef __cplusplus
}
#endif

#endif 

