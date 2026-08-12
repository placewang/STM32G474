#ifndef _BSP_USART2_H__
#define _BSP_USART2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"





void BSP_Usart2StartDma(void);
uint16_t bsp_UART2_Read(uint8_t *buf, uint16_t len);
HAL_StatusTypeDef bsp_UART2_DMASend(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif 

