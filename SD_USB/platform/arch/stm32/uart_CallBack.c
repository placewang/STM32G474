#include "uart_board.h"

/**
 * @brief UART 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
	{
        UART1_ErrorCallback(huart);
    }
}

/**
 * @brief USART1 DMA 接收事件回调
 * @param huart 串口句柄
 * @param Size  HAL 给出的当前有效长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) 
	{
       UARTE1_RxEventCallback(huart,Size);
    }
}
/**
 * @brief DMA 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
	{
		UART1_TxCpltCallback(huart);
    }	
}



















