#include "usart.h"
#include <string.h>
#include "bsp_usartQue.h"

#define UART1_BUF_SIZE   UART_BUF_SIZE

USART_DMA  Uart1Que;
uint8_t    usart1_DMA_rxBuff[UART1_BUF_SIZE];


/*
 *  从接收队列读取数据
 *  len     期望读取长度
 *  返回值  实际有效长度
 *  说明：
 *  1. 队列为空返回 0
 *  2. 队列数据不足 len 时，返回当前有效长度
 *  3. 队列数据不少于 len 时，按 len 读取并出队
 */
uint16_t bsp_UART1_Read(uint8_t *buf, uint16_t len)
{
    uint16_t available;
    uint16_t Aclen=0;
    if ((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    __disable_irq();

    available = (uint16_t)QueLen(&Uart1Que);
    if (available == 0U) {
        __enable_irq();
        return 0U;
    }
	
	Aclen =	available>len?len:available;
    QueueCopyOut(&Uart1Que, Uart1Que.rxFront, buf, Aclen);
    Uart1Que.rxFront = (Uart1Que.rxFront + Aclen) % Uart1Que.rxMAXSIZE;

    __enable_irq();
    return Aclen;
}
/**
 * @brief 启动 USART1 DMA 循环接收
 */
void BSP_Usart1StartDma(void)
{
    /* 清零标志和队列状态 */
	Uart1Que.sendFlag_end = 0U;
    Uart1Que.rxOffset = 0U;
    Uart1Que.rxRear = 0U;
    Uart1Que.rxFront = 0U;
    Uart1Que.rxMAXSIZE = UART1_BUF_SIZE;
    memset(Uart1Que.rxQueuefifo, 0, sizeof(Uart1Que.rxQueuefifo));

    /* 启动 DMA + IDLE 接收 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_DMA_rxBuff, UART1_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }

    /* 使能 DMA 半满、全满和传输错误中断 */
    __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
}

uint32_t errorCode = 0;

/**
 * @brief UART 错误回调
 */
void bsp_UART1_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. 获取错误状态
    errorCode = HAL_UART_GetError(huart);
    // 2. 停止当前 DMA 接收
    HAL_UART_DMAStop(huart);
    // 3. 暂时关闭 UART，避免清标志过程中再次触发中断
    __HAL_UART_DISABLE(huart);
    // 4. 手动清除各类错误标志
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    // 5. 读 DR 完成最后一步清标志
    (void)huart->Instance->RDR;
    // 6. 重新使能并重启 DMA 接收
    __HAL_UART_ENABLE(huart);
    BSP_Usart1StartDma();
#if __DEBUG
//    log_e("ERR:UART_ErrorCallback %d\n", errorCode);
#endif
}

/**
 * @brief USART1 DMA 接收事件回调
 * @param huart 串口句柄
 * @param Size  HAL 给出的当前有效长度
 */
void bsp_UARTE1_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_RxEventTypeTypeDef evt;
    uint32_t current_offset;

    (void)Size;

    if (huart->Instance != USART1) {
        return;
    }

    evt = HAL_UARTEx_GetRxEventType(huart);
    current_offset = UART1_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    current_offset %= UART1_BUF_SIZE;

    /* 空闲或半传输时，按当前偏移增量搬运 */
    if ((evt == HAL_UART_RXEVENT_IDLE) || (evt == HAL_UART_RXEVENT_HT))
	{
        update_dma_data(&Uart1Que,usart1_DMA_rxBuff,current_offset);
    }
	else if (evt == HAL_UART_RXEVENT_TC)
	{
        /* 全传输时搬运尾段数据，再把偏移归零 */
        if (Uart1Que.rxOffset < UART1_BUF_SIZE) {
            copy_to_queue_from_dma(&Uart1Que,usart1_DMA_rxBuff,UART1_BUF_SIZE - Uart1Que.rxOffset);
        }
        Uart1Que.rxOffset = 0U;
    }
}
/**
 * @brief DMA 发送完成回调
 */
void bsp_UART1_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Uart1Que.sendFlag_end = 0U;
    }
}

/**
 * @brief USART1 DMA 发送接口
 */
#define    UART1_TxBUF_SIZE   64
uint8_t    usart1_DMA_txBuff[UART1_TxBUF_SIZE];
HAL_StatusTypeDef bsp_UART1_DMASend(const char *data, uint16_t len)
{
    uint32_t timeout = HAL_GetTick() + 500U;
	    /* 等待上一包发送完成 */
    while (Uart1Que.sendFlag_end && (HAL_GetTick() < timeout)) {
    }
    if (Uart1Que.sendFlag_end) {
        return HAL_TIMEOUT;
    }	
	memcpy(usart1_DMA_txBuff,data,len);
    Uart1Que.sendFlag_end = 1U;
	HAL_UART_Transmit_DMA(&huart1,(const uint8_t*) usart1_DMA_txBuff, len);
    return HAL_OK;
}



//extern USART_DMA  Uart2Que;
//void testUart1Send(void)
//{
//   static uint8_t val[128];
//    uint16_t lenn=0;
//	uint16_t lendata=32;
//	
//  if (!Uart1Que.sendFlag_end && QueLen(&Uart2Que) >= lendata) {
//      lenn = bsp_UART2_Read(val, lendata);
//      bsp_UART1_DMASend((const char *)val, lenn);
//  }
//}
