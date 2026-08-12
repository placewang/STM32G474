#include "bsp_usart1.h"

#include "usart.h"


extern void bsp_UART1_ErrorCallback(UART_HandleTypeDef *huart);
extern void bsp_UART1_TxCpltCallback(UART_HandleTypeDef *huart);
extern void bsp_UARTE1_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

extern void bsp_UART2_ErrorCallback(UART_HandleTypeDef *huart);
extern void bsp_UART2_TxCpltCallback(UART_HandleTypeDef *huart);
extern void bsp_UARTE2_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

extern void bsp_UART3_ErrorCallback(UART_HandleTypeDef *huart);
extern void bsp_UART3_TxCpltCallback(UART_HandleTypeDef *huart);
extern void bsp_UARTE3_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief UART 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
	{
        bsp_UART1_ErrorCallback(huart);
    }
	else if (huart->Instance == USART2)
	{
		bsp_UART2_ErrorCallback(huart);
	}
	else if (huart->Instance == USART3)
	{
		bsp_UART3_ErrorCallback(huart);
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
       bsp_UARTE1_RxEventCallback(huart,Size);
    }
	else if (huart->Instance == USART2)
	{
		bsp_UARTE2_RxEventCallback(huart,Size);
	}
	else if (huart->Instance == USART3)
	{
		bsp_UARTE3_RxEventCallback(huart,Size);
	}	
}
/**
 * @brief DMA 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
	{
		bsp_UART1_TxCpltCallback(huart);
    }
    else if (huart->Instance == USART2)
	{
		bsp_UART2_TxCpltCallback(huart);
    }	
    else if (huart->Instance == USART3)
	{
		bsp_UART3_TxCpltCallback(huart);
    }		
}



