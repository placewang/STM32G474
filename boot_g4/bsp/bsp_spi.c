/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "bsp_spi.h"
#include <string.h>

uint8_t spi1_rx_buffer[SPI1_DMA_FRAME_SIZE];
uint8_t spi1_tx_buffer[SPI1_DMA_FRAME_SIZE];

SPI1_DMA SPI1_RxTx;


static uint8_t SPI1_ReadNssLevel(void)
{
  return (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
}

static uint16_t SPI1_GetActiveDmaLen(void)
{
  return (uint16_t)hspi1.RxXferSize;
}

static void SPI1_ResetPeripheralAfterNssHigh(void)
{
  volatile uint32_t tmp;

  (void)HAL_SPI_DMAStop(&hspi1);

  __HAL_SPI_DISABLE(&hspi1);

  tmp = hspi1.Instance->DR;
  tmp = hspi1.Instance->SR;
  (void)tmp;

  __HAL_RCC_SPI1_FORCE_RESET();
  __NOP();
  __NOP();
  __HAL_RCC_SPI1_RELEASE_RESET();

  hspi1.State = HAL_SPI_STATE_READY;
  hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
  (void)HAL_SPI_Init(&hspi1);

  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
  __HAL_SPI_ENABLE(&hspi1);
}

__weak void bsp_SPI1_FillPack_XORCheck(uint8_t *SPItxData,const uint8_t len)
{
}
static HAL_StatusTypeDef SPI1_ArmNormalDma(void)
{
  uint16_t arm_len = SPI1_RxTx.tx_reality_len;

  if ((arm_len == 0U) || (arm_len > SPI1_DMA_BLOCK_SIZE))
  {
    arm_len = SPI1_DMA_BLOCK_SIZE;
  }

  SPI1_ResetPeripheralAfterNssHigh();
  
  if (HAL_SPI_TransmitReceive_DMA(&hspi1,
                                  spi1_tx_buffer,
                                  spi1_rx_buffer,
                                  arm_len) != HAL_OK)
  {
    SPI1_RxTx.dma_err = 1U;
    return HAL_ERROR;
  }

  SPI1_RxTx.dma_running = 1U;
  return HAL_OK;
}
static void SPI1_LoadPendingTxIfIdle(void)
{
  if (SPI1_ReadNssLevel()!= GPIO_PIN_RESET)
  {
    if (SPI1_RxTx.tx_data_ready != 0U)
    {
      __disable_irq();
      memcpy(spi1_tx_buffer, SPI1_RxTx.tx_shadow, SPI1_RxTx.tx_reality_len);
      bsp_SPI1_FillPack_XORCheck(spi1_tx_buffer,SPI1_RxTx.tx_reality_len);
      SPI1_RxTx.tx_data_ready = 0U;
      __enable_irq();
    }
  }
}



void bsp_Spi1StartDmaExchange(void)
{
  SPI1_RxTx.rx_ping_ready  = 0U;
  SPI1_RxTx.rx_pong_ready  = 0U;
  SPI1_RxTx.rx_reality_cnt = 0u;
  
  memset(SPI1_RxTx.rx_shadow, 0, sizeof(SPI1_RxTx.rx_shadow));
  memset(spi1_rx_buffer, 0, sizeof(spi1_rx_buffer));

  SPI1_RxTx.dma_err = 0U;
  SPI1_RxTx.dma_running = 0U;

  /* Do not clear tx_shadow/spi1_tx_buffer here; app_HostData_init()*/

  if (SPI1_RxTx.tx_reality_len == 0U)
  {
    SPI1_RxTx.tx_reality_len = SPI1_DMA_BLOCK_SIZE;
  }

  SPI1_LoadPendingTxIfIdle();

  if (SPI1_ArmNormalDma() == HAL_ERROR)
  {
    SPI1_RxTx.dma_err = 1U;
  }
}

void bsp_SPI1_Process(void)
{
  if (SPI1_RxTx.dma_err != 0U)
  {
    HAL_SPI_DMAStop(&hspi1);
    SPI1_RxTx.dma_running = 0U;
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    SPI1_RxTx.dma_err = 0U;
	bsp_Spi1StartDmaExchange();
  }
  SPI1_LoadPendingTxIfIdle();
}

void bsp_SPI1_SubmitTxData(const uint8_t *data)
{
  if (data == NULL)
  {
    return;
  }

  __disable_irq();
  memcpy(SPI1_RxTx.tx_shadow, data, SPI1_RxTx.tx_reality_len);
  bsp_SPI1_FillPack_XORCheck(SPI1_RxTx.tx_shadow,SPI1_RxTx.tx_reality_len);
  SPI1_RxTx.tx_data_ready = 1U;
  __enable_irq();
  SPI1_LoadPendingTxIfIdle();
}


void bsp_SPI1_NssExtiCallback(void)
{
  uint16_t active_len;
  uint16_t remaining;
  uint16_t rx_cnt;

  if (SPI1_ReadNssLevel() != GPIO_PIN_RESET)
  {
	  __disable_irq();

      active_len = SPI1_GetActiveDmaLen();
      remaining = (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_spi1_rx);

      if ((active_len == 0U) || (active_len > SPI1_DMA_BLOCK_SIZE))
      {
        active_len = SPI1_DMA_BLOCK_SIZE;
      }

      if (remaining <= active_len)
      {
        rx_cnt = active_len - remaining;
      }
      else
      {
        rx_cnt = 0U;
      }

	  SPI1_RxTx.rx_reality_cnt = (uint8_t)rx_cnt;

      if (rx_cnt > 0U)
      {
	    memcpy(SPI1_RxTx.rx_shadow, spi1_rx_buffer, rx_cnt);
	    SPI1_RxTx.rx_ping_ready = 1U;
	    SPI1_RxTx.tx_reality_len = (uint8_t)rx_cnt;
      }

      SPI1_RxTx.dma_running = 0U;
      __enable_irq();

      SPI1_ArmNormalDma();
  }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spiHandle)
{
//  if (spiHandle->Instance == SPI1)
//  {
//    __disable_irq();
//    memcpy(SPI1_RxTx.rx_shadow[0], spi1_rx_buffer, SPI1_DMA_BLOCK_SIZE);
//    SPI1_RxTx.rx_ping_ready = 1U;
//    SPI1_RxTx.dma_running = 0U;
//    __enable_irq();
//  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance == SPI1)
  {
    SPI1_RxTx.dma_running = 0U;
    SPI1_RxTx.dma_err = 1U;
    (void)HAL_SPI_GetError(spiHandle);
  }
}


