#include "usart.h"
#include <string.h>
#include "bsp_usartQue.h"


#define UART3_BUF_SIZE   UART_BUF_SIZE
USART_DMA  Uart3Que;
uint8_t    usart3_DMA_rxBuff[UART3_BUF_SIZE];


/*
 *  从接收队列读取数据
 *  len     期望读取长度
 *  返回值  实际有效长度
 *  说明：
 *  1. 队列为空返回 0
 *  2. 队列数据不足 len 时，返回当前有效长度
 *  3. 队列数据不少于 len 时，按 len 读取并出队
 */
uint16_t bsp_UART3_Read(uint8_t *buf, uint16_t len)
{
    uint16_t available;
    uint16_t Aclen=0;
    if ((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    __disable_irq();

    available = (uint16_t)QueLen(&Uart3Que);
    if (available == 0U) {
        __enable_irq();
        return 0U;
    }
	
	Aclen =	available>len?len:available;
    QueueCopyOut(&Uart3Que, Uart3Que.rxFront, buf, Aclen);
    Uart3Que.rxFront = (Uart3Que.rxFront + Aclen) % Uart3Que.rxMAXSIZE;

    __enable_irq();
    return Aclen;
}

/**
 * @brief 启动 USART1 DMA 循环接收
 */
void BSP_Usart3StartDma(void)
{
    /* 清零标志和队列状态 */
    Uart3Que.rxOffset = 0U;
    Uart3Que.rxRear = 0U;
    Uart3Que.rxFront = 0U;
    Uart3Que.rxMAXSIZE = UART3_BUF_SIZE;
    memset(Uart3Que.rxQueuefifo, 0, sizeof(Uart3Que.rxQueuefifo));

    /* 启动 DMA + IDLE 接收 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, usart3_DMA_rxBuff                                                                                                                                               , UART3_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }

    /* 使能 DMA 半满、全满和传输错误中断 */
    __HAL_DMA_ENABLE_IT(&hdma_usart3_rx, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
}

/**
 * @brief UART 错误回调
 */
void bsp_UART3_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t errCode = 0;
    // 1. 获取错误状态
    errCode = HAL_UART_GetError(huart);
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
    BSP_Usart3StartDma();
#if __DEBUG
    printf("ERR:UART_ErrorCallback %d\n", errCode);
#endif
}

/**
 * @brief USART1 DMA 接收事件回调
 * @param huart 串口句柄
 * @param Size  HAL 给出的当前有效长度
 */
void bsp_UARTE3_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_RxEventTypeTypeDef evt;
    uint32_t current_offset;

    (void)Size;

    if (huart->Instance != USART2) {
        return;
    }

    evt = HAL_UARTEx_GetRxEventType(huart);
    current_offset = UART3_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    current_offset %= UART3_BUF_SIZE;

    /* 空闲或半传输时，按当前偏移增量搬运 */
    if ((evt == HAL_UART_RXEVENT_IDLE) || (evt == HAL_UART_RXEVENT_HT))
	{
        update_dma_data(&Uart3Que,usart3_DMA_rxBuff,current_offset);
    }
	else if (evt == HAL_UART_RXEVENT_TC)
	{
        /* 全传输时搬运尾段数据，再把偏移归零 */
        if (Uart3Que.rxOffset < UART3_BUF_SIZE) {
            copy_to_queue_from_dma(&Uart3Que,&usart3_DMA_rxBuff[Uart3Que.rxOffset],
                                   UART3_BUF_SIZE - Uart3Que.rxOffset);
        }
        Uart3Que.rxOffset = 0U;
    }
}
/**
 * @brief DMA 发送完成回调
 */
void bsp_UART3_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        Uart3Que.sendFlag_end = 0U;
    }
}

/**
 * @brief USART1 DMA 发送接口
 */
HAL_StatusTypeDef bsp_UART3_DMASend(uint8_t *data, uint16_t len)
{
    uint32_t timeout = HAL_GetTick() + 500U;

    /* 等待上一包发送完成 */
    while (Uart3Que.sendFlag_end && (HAL_GetTick() < timeout)) {
    }

    if (Uart3Que.sendFlag_end) {
        return HAL_TIMEOUT;
    }

    Uart3Que.sendFlag_end = 1U;
    return HAL_UART_Transmit_DMA(&huart3, data, len);
}

/*
 *  测试函数：从接收队列取数据并回发
 */

//void testUart3Send(void)
//{
//   static uint8_t val[128];
//   uint16_t lenn=0;
//   uint16_t lendata=11;
//	
//	if (!Uart3Que.sendFlag_end && QueLen(&Uart3Que) >= lendata) {
//      lenn = bsp_UART3_Read(val, lendata);
//      bsp_UART3_DMASend(val, lenn);
//  }
//}
