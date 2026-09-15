#ifndef _BSP_USART3_H__
#define _BSP_USART3_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"





void BSP_Usart3StartDma(void);
uint16_t bsp_UART3_Read(uint8_t *buf, uint16_t len);



#ifdef __cplusplus
}
#endif

#endif 

