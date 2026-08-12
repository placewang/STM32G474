#ifndef _BSP_SPI_H__
#define _BSP_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define SPI1_DMA_BLOCK_SIZE 68U
#define SPI1_DMA_BLOCK_COUNT 2U
#define SPI1_DMA_FRAME_SIZE (SPI1_DMA_BLOCK_SIZE * SPI1_DMA_BLOCK_COUNT)
/**
 * @brief SPI1 DMA 双缓冲块索引（Ping-Pong）
 *        SPI1_DMA_BLOCK_PING = 0   -> 前半块（半完成中断）
 *        SPI1_DMA_BLOCK_PONG = 1   -> 后半块（完成中断）
 */
typedef enum
{
  SPI1_DMA_BLOCK_PING = 0U,
  SPI1_DMA_BLOCK_PONG = 1U
} SPI1_DmaBlockIndex;


typedef struct SPI1_DMA
{
 uint8_t  rxPING;                   /*前缓存满状态*/
 uint8_t  rxPONG;				    /*后缓存满状态*/
 uint8_t  txPING;                   /*前缓存满状态*/
 uint8_t  txPONG;				    /*后缓存满状态*/	
 volatile uint8_t restart_pending;  /* DMA 传输是否正在运行 */
 volatile uint8_t dma_running;      /* 请求重新启动 DMA 传输 */ 
}SPI1_DMA;

extern SPI1_DMA SPI1_RxTx;

/* USER CODE BEGIN Prototypes */
void bsp_Spi1StartDmaExchange(void);
uint8_t *bsp_SPI1_GetRxBlock(SPI1_DmaBlockIndex block);
uint8_t *bsp_SPI1_GetTxBlock(SPI1_DmaBlockIndex block);




#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
