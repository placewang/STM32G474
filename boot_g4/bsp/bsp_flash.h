#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "stm32g4xx_hal.h"

/* Base definitions ---------------------------------------------------------*/
#define STM32_FLASH_BASE              FLASH_BASE
#define STM32_FLASH_PAGE_SIZE         FLASH_PAGE_SIZE
#define STM32_FLASH_INVALID_PAGE      0xFFFFFFFFU
#define STM32_FLASH_INVALID_ADDRESS   0xFFFFFFFFU
#define FLASH_WAIT_TIMEOUT            50000U

/* Flash geometry -----------------------------------------------------------*/
uint32_t STMFLASH_GetFlashSize(void);
uint32_t STMFLASH_GetFlashEndAddr(void);
uint32_t STMFLASH_GetFlashPageSize(void);
uint32_t STMFLASH_GetFlashPageAddress(uint32_t page);
uint32_t STMFLASH_GetFlashSectorAddress(uint32_t sector);
uint8_t STMFLASH_IsValidRange(uint32_t addr, uint32_t bytes);

/* STM32G4 uses erase pages. DBANK=1 uses 2 KB pages, DBANK=0 uses 4 KB pages. */
#define ADDR_FLASH_PAGE(page)         STMFLASH_GetFlashPageAddress((uint32_t)(page))
#define ADDR_FLASH_PAGE_0             STM32_FLASH_BASE

/* Legacy compatibility: treat "sector" as an erase-page index on STM32G4. */
#define ADDR_FLASH_SECTOR(sector)     STMFLASH_GetFlashSectorAddress((uint32_t)(sector))
#define ADDR_FLASH_SECTOR_0           ADDR_FLASH_SECTOR(0U)
#define ADDR_FLASH_SECTOR_1           ADDR_FLASH_SECTOR(1U)
#define ADDR_FLASH_SECTOR_2           ADDR_FLASH_SECTOR(2U)
#define ADDR_FLASH_SECTOR_3           ADDR_FLASH_SECTOR(3U)
#define ADDR_FLASH_SECTOR_4           ADDR_FLASH_SECTOR(4U)
#define ADDR_FLASH_SECTOR_5           ADDR_FLASH_SECTOR(5U)
#define ADDR_FLASH_SECTOR_6           ADDR_FLASH_SECTOR(6U)
#define ADDR_FLASH_SECTOR_7           ADDR_FLASH_SECTOR(7U)

/* Page helpers -------------------------------------------------------------*/
uint32_t STMFLASH_GetFlashPage(uint32_t addr);
uint32_t STMFLASH_GetFlashSector(uint32_t addr);

/* Erase operations ---------------------------------------------------------*/
HAL_StatusTypeDef bsp_IndividualPageErasure(uint32_t PageAddr);
HAL_StatusTypeDef bsp_IndividualSectorErasure(uint32_t SectorAddr);
HAL_StatusTypeDef bsp_FlashErasePages(uint32_t StartPageAddr, uint32_t NumPages);
HAL_StatusTypeDef bsp_FlashEraseSectors(uint32_t StartSectorAddr, uint8_t NumSectors);

/* Program and read operations ---------------------------------------------*/
HAL_StatusTypeDef bsp_STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumWords);
HAL_StatusTypeDef bsp_STMFLASH_ReadChecked(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords);
void bsp_STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords);
uint8_t FLASH_read_byte(uint32_t addr);

/* Empty checks -------------------------------------------------------------*/
uint8_t FLASH_IsPageEmpty(uint32_t PageAddr);
uint8_t FLASH_IsSectorEmpty(uint32_t SectorAddr);

#endif /* __BSP_FLASH_H */
