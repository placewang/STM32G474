#include "usart.h"
#include <string.h>
#include "bsp_usartQue.h"


#define UART2_BUF_SIZE   UART_BUF_SIZE

USART_DMA  Uart2Que;
uint8_t    usart2_DMA_rxBuff[UART2_BUF_SIZE];


/*
 *  从接收队列读取数据
 *  len     期望读取长度
 *  返回值  实际有效长度
 *  说明：
 *  1. 队列为空返回 0
 *  2. 队列数据不足 len 时，返回当前有效长度
 *  3. 队列数据不少于 len 时，按 len 读取并出队
 */
uint16_t bsp_UART2_Read(uint8_t *buf, uint16_t len)
{
    uint16_t available;
    uint16_t Aclen=0;
    if ((buf == NULL) || (len == 0U)) {
        return 0U;
    }

    __disable_irq();

    available = (uint16_t)QueLen(&Uart2Que);
    if (available == 0U&&len!=available) {
        __enable_irq();
        return 0U;
    }
	
	Aclen =	available>len?len:available;
    QueueCopyOut(&Uart2Que, Uart2Que.rxFront, buf, Aclen);
    Uart2Que.rxFront = (Uart2Que.rxFront + Aclen) % Uart2Que.rxMAXSIZE;

    __enable_irq();
    return Aclen;
}

/**
 * @brief 启动 USART1 DMA 循环接收
 */
void BSP_Usart2StartDma(void)
{
    /* 清零标志和队列状态 */
    Uart2Que.rxOffset = 0U;
    Uart2Que.rxRear = 0U;
    Uart2Que.rxFront = 0U;
    Uart2Que.rxMAXSIZE = UART2_BUF_SIZE;
    memset(Uart2Que.rxQueuefifo, 0, sizeof(Uart2Que.rxQueuefifo));

    /* 启动 DMA + IDLE 接收 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_DMA_rxBuff, UART2_BUF_SIZE) != HAL_OK) {
        Error_Handler();
    }

    /* 使能 DMA 半满、全满和传输错误中断 */
    __HAL_DMA_ENABLE_IT(&hdma_usart2_rx, DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);
}



/**
 * @brief UART 错误回调
 */
void bsp_UART2_ErrorCallback(UART_HandleTypeDef *huart)
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
    BSP_Usart2StartDma();
#if __DEBUG
    printf("ERR:UART_ErrorCallback %d\n", errCode);
#endif
}

/**
 * @brief USART1 DMA 接收事件回调
 * @param huart 串口句柄
 * @param Size  HAL 给出的当前有效长度
 */
void bsp_UARTE2_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_RxEventTypeTypeDef evt;
    uint32_t current_offset;

    (void)Size;

    if (huart->Instance != USART2) {
        return;
    }

    evt = HAL_UARTEx_GetRxEventType(huart);
    current_offset = UART2_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    current_offset %= UART2_BUF_SIZE;

    /* 空闲或半传输时，按当前偏移增量搬运 */
    if ((evt == HAL_UART_RXEVENT_IDLE) || (evt == HAL_UART_RXEVENT_HT))
	{
        update_dma_data(&Uart2Que,usart2_DMA_rxBuff,current_offset);
    }
	else if (evt == HAL_UART_RXEVENT_TC)
	{
        /* 全传输时搬运尾段数据，再把偏移归零 */
        if (Uart2Que.rxOffset < UART2_BUF_SIZE) {
            copy_to_queue_from_dma(&Uart2Que,&usart2_DMA_rxBuff[Uart2Que.rxOffset],
                                   UART2_BUF_SIZE - Uart2Que.rxOffset);
        }
        Uart2Que.rxOffset = 0U;
    }
}
/**
 * @brief DMA 发送完成回调
 */
void bsp_UART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        Uart2Que.sendFlag_end = 0U;
    }
}

/**
 * @brief USART1 DMA 发送接口
 */
#define    UART2_TxBUF_SIZE   64
uint8_t    usart2_DMA_txBuff[UART2_TxBUF_SIZE];

HAL_StatusTypeDef bsp_UART2_DMASend(uint8_t *data, uint16_t len)
{
    uint32_t timeout = HAL_GetTick() + 500U;

    /* 等待上一包发送完成 */
    while (Uart2Que.sendFlag_end && (HAL_GetTick() < timeout)) {
    }

    if (Uart2Que.sendFlag_end) {
        return HAL_TIMEOUT;
    }
	memcpy(usart2_DMA_txBuff,data,len);
    Uart2Que.sendFlag_end = 1U;
    HAL_UART_Transmit_DMA(&huart2, usart2_DMA_txBuff, len);
	HAL_Delay(50);
}

/*
 *  测试函数：从接收队列取数据并回发
 */
//extern USART_DMA  Uart1Que;
//void testUart2Send(void)
//{
//   static uint8_t val[128];
//   uint16_t lenn=0;
//   uint16_t lendata=32;
//	
//	if (!Uart2Que.sendFlag_end && QueLen(&Uart1Que) >= lendata) {
//      lenn = bsp_UART1_Read(val, lendata);
//      bsp_UART2_DMASend(val, lenn);
//  }
//}
