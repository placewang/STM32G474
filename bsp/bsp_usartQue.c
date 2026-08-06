#include "bsp_usartQue.h"
#include <string.h>

#define ut_True  1
#define ut_False 0



/*
 *  循环队列判满
 *  q 循环队列
 */
static int QueFull(USART_DMA *q)
{
    return (q->rxRear + 1U) % q->rxMAXSIZE == q->rxFront;
}

/* 循环队列判空 q 循环队列 */
static int QueEmpty(USART_DMA *q)
{
    return q->rxFront == q->rxRear;
}

/*
 *  计算循环队列长度
 *  q 循环队列
 */
int QueLen(USART_DMA *q)
{
    return (q->rxRear - q->rxFront + q->rxMAXSIZE) % q->rxMAXSIZE;
}

/*
 *  循环队列 入队
 *  q 循环队列
 *  data 入队元素
 */
static int QueEn(USART_DMA *q, uint8_t *data)
{
    if (QueFull(q)) {
        return ut_False;
    }

    q->rxQueuefifo[q->rxRear] = *data;
    q->rxRear = (q->rxRear + 1U) % q->rxMAXSIZE;
    return ut_True;
}

/*
 *  循环队列 出队
 *  q 循环队列
 *  *val 用来取出队元素的数据
 */
int QueDe(USART_DMA *q, uint8_t *val)
{
    if (QueEmpty(q)) {
        return ut_False;
    }

    *val = q->rxQueuefifo[q->rxFront];
    q->rxFront = (q->rxFront + 1U) % q->rxMAXSIZE;
    return ut_True;
}

/*
 *  从循环队列指定起点拷贝一段连续/回卷数据
 *  start 为起始下标，len 为要拷贝的总长度
 */
void QueueCopyOut(USART_DMA *q, uint32_t start, uint8_t *buf, uint16_t len)
{
    uint16_t first_len;
    uint16_t second_len;

    first_len = (uint16_t)(q->rxMAXSIZE - start);
    if (first_len > len) {
        first_len = len;
    }

    memcpy(buf, &q->rxQueuefifo[start], first_len);

    second_len = len - first_len;
    if (second_len > 0U) {
        memcpy(buf + first_len, &q->rxQueuefifo[0], second_len);
    }
}
/*
 *  从 DMA 缓冲区搬运数据到软件队列
 */
void copy_to_queue_from_dma(USART_DMA *uart_que,uint8_t *data, uint32_t len)
{
      uint32_t free_len;
      uint32_t copy_len;
      uint32_t first_len;
      uint32_t second_len;
      uint32_t rear;

      if ((len == 0U) || (data == NULL)) {
          return;
      }

      __disable_irq();

      free_len = uart_que->rxMAXSIZE - 1U - QueLen(uart_que);
      copy_len = (len <= free_len) ? len : free_len;

      if (copy_len < len) {
          uart_que->overflow_cnt++;
      }

      rear = uart_que->rxRear;

      first_len = uart_que->rxMAXSIZE - rear;
      if (first_len > copy_len) {
          first_len = copy_len;
      }

      memcpy(&uart_que->rxQueuefifo[rear], data, first_len);

      second_len = copy_len - first_len;
      if (second_len > 0U) {
          memcpy(&uart_que->rxQueuefifo[0], data + first_len, second_len);
      }

      uart_que->rxRear = (rear + copy_len) % uart_que->rxMAXSIZE;

      __enable_irq();
  }
/*
 *  根据 DMA 当前写入偏移，增量搬运新收到的数据
 */
void update_dma_data(USART_DMA *uart_Que,uint8_t *DMA_RXbuff,uint32_t current_offset)
{
    uint32_t new_len;

    if (current_offset == uart_Que->rxOffset) {
        return;
    }

    if (current_offset > uart_Que->rxOffset) {
        new_len = current_offset - uart_Que->rxOffset;
        copy_to_queue_from_dma(uart_Que,&DMA_RXbuff[uart_Que->rxOffset], new_len);
        uart_Que->rxOffset = current_offset;
    } 
	else {
        uint32_t len1 = UART_BUF_SIZE - uart_Que->rxOffset;
        copy_to_queue_from_dma(uart_Que,&DMA_RXbuff[uart_Que->rxOffset], len1);
        if (current_offset > 0U) {
            copy_to_queue_from_dma(uart_Que,&DMA_RXbuff[0], current_offset);
        }
        uart_Que->rxOffset = current_offset;
    }
}








