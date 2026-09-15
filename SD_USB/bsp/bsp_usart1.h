#ifndef _BSP_USART1_H__
#define _BSP_USART1_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"





void BSP_Usart1StartDma(void);
uint16_t bsp_UART1_Read(uint8_t *buf, uint16_t len);
HAL_StatusTypeDef bsp_UART1_DMASend(const char *data, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif 

