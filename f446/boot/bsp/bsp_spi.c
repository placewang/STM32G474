/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "bsp_spi.h"
#include <string.h>

/* 收发 DMA 缓冲区*/
uint8_t spi1_rx_buffer[SPI1_DMA_FRAME_SIZE];
uint8_t spi1_tx_buffer[SPI1_DMA_FRAME_SIZE];


SPI1_DMA  SPI1_RxTx;

/**
 * @brief 获取指定缓冲区中某个块的基地址
 * @param buffer  缓冲区首地址（tx 或 rx）
 * @param block   块索引（PING=0, PONG=1）
 * @return 对应块的起始指针
 */
static uint8_t *SPI1_GetBlockBase(uint8_t *buffer, SPI1_DmaBlockIndex block)
{
  return &buffer[(uint32_t)block * SPI1_DMA_BLOCK_SIZE];
}
/**
 * @brief 获取指定 RX 块的指针
 */
uint8_t *bsp_SPI1_GetRxBlock(SPI1_DmaBlockIndex block)
{
  return SPI1_GetBlockBase(spi1_rx_buffer, block);
}
/**
 * @brief 获取指定 TX 块的指针
 */
uint8_t *bsp_SPI1_GetTxBlock(SPI1_DmaBlockIndex block)
{
  return SPI1_GetBlockBase(spi1_tx_buffer, block);
}
/**
 * @brief 当一个块（PING 或 PONG）传输完成时调用
 * @param block  完成的块索引
 * 功能：
 * 逻辑关系：该函数由中断回调 HAL_SPI_TxRxHalfCpltCallback（PING）和
 *           HAL_SPI_TxRxCpltCallback（PONG）分别调用。
 */
static void SPI1_HandleCompletedBlock(SPI1_DmaBlockIndex block)
{
  memcpy(SPI1_GetBlockBase(spi1_rx_buffer,block), 
	     SPI1_GetBlockBase(spi1_tx_buffer,block),
                            SPI1_DMA_BLOCK_SIZE);
}

/**
 * @brief 启动 SPI DMA 循环收发
 *        清除重启标志，加载默认模式，调用 HAL 库函数启动 DMA 传输
 *        传输总长度为 32 字节，由于 DMA 是循环模式，会不断重复这 32 字节
 */
void bsp_Spi1StartDmaExchange(void)
{
  SPI1_RxTx.rxPING=0;
  SPI1_RxTx.rxPONG=0;	
  SPI1_RxTx.txPING=0;
  SPI1_RxTx.txPONG=0;		
  SPI1_RxTx.restart_pending = 0U;
  SPI1_RxTx.dma_running = 0U;
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi1_tx_buffer, spi1_rx_buffer, SPI1_DMA_FRAME_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  SPI1_RxTx.dma_running = 1U;
}
/**
 * @brief 定期处理函数，需在主循环中调用
 *        当发生错误时，spi1_restart_pending 被置 1，此函数会停止当前 DMA，
 *        清除溢出标志，并重新启动传输
 */
void bsp_SPI1_Process(void)
{
  if (SPI1_RxTx.restart_pending != 0U)
  {
    if (SPI1_RxTx.dma_running != 0U)
    {
      (void)HAL_SPI_DMAStop(&hspi1);
       SPI1_RxTx.dma_running = 0U;
    }
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    bsp_Spi1StartDmaExchange();
  }
}

/**
 * @brief 将外部数据加载到指定的 TX 块中
 * @param block  目标块索引
 * @param src    源数据指针
 * @param len    有效数据长度（不能超过块大小）
 * @return 0 / !0
 * 逻辑：若长度不足块大小，剩余部分自动填充 0
 */
HAL_StatusTypeDef SPI1_LoadTxBlock(SPI1_DmaBlockIndex block, const uint8_t *src, uint16_t len)
{
  uint8_t *dst = NULL;

  if ((src == NULL) || (len > SPI1_DMA_BLOCK_SIZE))
  {
    return HAL_ERROR;
  }

  dst =SPI1_GetBlockBase(spi1_tx_buffer,block);
  memcpy(dst, src, len);

  if (len < SPI1_DMA_BLOCK_SIZE)
  {
    memset(&dst[len], 0, SPI1_DMA_BLOCK_SIZE - len);
  }

  return HAL_OK;
}
/**
 * @brief SPI 半完成中断回调（对应 DMA 传输到一半时）
 *        HAL 库在 DMA 循环模式下，传输完前半部分会触发此回调
 *        此时对应的块索引为 PING (0)
 */
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance == SPI1)
  {
	 	SPI1_RxTx.rxPING=1;   
	    SPI1_RxTx.txPING=0;
  }
}
/**
 * @brief SPI 完成中断回调（对应 DMA 传输完整）
 *        此时对应的块索引为 PONG (1)
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance == SPI1)
  {
	 SPI1_RxTx.rxPONG=1;
	 SPI1_RxTx.txPONG=0;	
  }
}
/**
 * @brief SPI 错误回调
 *        发生溢出、CRC错误等时触发
 *        记录错误计数，停止 DMA，触发重启标志，并调用用户错误处理
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance == SPI1)
  {

    SPI1_RxTx.dma_running = 0U;
    SPI1_RxTx.restart_pending = 1U;
	HAL_SPI_GetError(spiHandle);
  }
}

/* USER CODE END 1 */
