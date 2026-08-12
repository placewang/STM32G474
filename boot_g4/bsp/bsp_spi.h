#ifndef _BSP_SPI_H__
#define _BSP_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SPI1_DMA_BLOCK_SIZE 88U
#define SPI1_DMA_BLOCK_COUNT 1U
#define SPI1_DMA_FRAME_SIZE (SPI1_DMA_BLOCK_SIZE * SPI1_DMA_BLOCK_COUNT)

typedef enum
{
  SPI1_DMA_BLOCK_PING = 0U,
  SPI1_DMA_BLOCK_PONG = 1U
} SPI1_DmaBlockIndex;

typedef struct SPI1_DMA
{
  volatile uint8_t rx_ping_ready;
  volatile uint8_t rx_pong_ready;
  volatile uint8_t rx_reality_cnt;
  
  uint8_t          rx_shadow[SPI1_DMA_BLOCK_SIZE];

  volatile uint8_t tx_data_ready;
  volatile uint8_t tx_reality_len;
  uint8_t          tx_shadow[SPI1_DMA_BLOCK_SIZE];

  volatile uint8_t dma_err;
  volatile uint8_t dma_running;
} SPI1_DMA;

extern SPI1_DMA SPI1_RxTx;

void bsp_SPI1_Process(void);
void bsp_Spi1StartDmaExchange(void);
void bsp_SPI1_SubmitTxData(const uint8_t *data);
void bsp_SPI1_NssExtiCallback(void);
__weak void bsp_SPI1_FillPack_XORCheck(uint8_t *SPItxData,const uint8_t len);

#ifdef __cplusplus
}
#endif

#endif 

