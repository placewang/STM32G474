#ifndef __UART_BOARD_H
#define __UART_BOARD_H

#include "../../../Core/Inc/usart.h"

void UART1_ErrorCallback(UART_HandleTypeDef *huart);
void UART1_TxCpltCallback(UART_HandleTypeDef *huart);
void UARTE1_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


#endif

