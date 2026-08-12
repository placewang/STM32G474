#include "bsp_flash.h"

/*----------------------------- 辅助函数 -----------------------------*/
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)          return FLASH_SECTOR_0;
    else if (addr < ADDR_FLASH_SECTOR_2)     return FLASH_SECTOR_1;
    else if (addr < ADDR_FLASH_SECTOR_3)     return FLASH_SECTOR_2;
    else if (addr < ADDR_FLASH_SECTOR_4)     return FLASH_SECTOR_3;
    else if (addr < ADDR_FLASH_SECTOR_5)     return FLASH_SECTOR_4;
    else if (addr < ADDR_FLASH_SECTOR_6)     return FLASH_SECTOR_5;
    else if (addr < ADDR_FLASH_SECTOR_7)     return FLASH_SECTOR_6;
    else if (addr < (ADDR_FLASH_SECTOR_7 + 128*1024)) return FLASH_SECTOR_7;
    else                                     return 0xFF;  /* 无效地址 */
}

/*----------------------------- 扇区擦除 -----------------------------*/
HAL_StatusTypeDef bsp_IndividualSectorErasure(uint32_t SectorAddr)
{
    /* 地址有效性检查：必须是扇区起始地址 */
    if (SectorAddr != ADDR_FLASH_SECTOR_0 && SectorAddr != ADDR_FLASH_SECTOR_1 &&
        SectorAddr != ADDR_FLASH_SECTOR_2 && SectorAddr != ADDR_FLASH_SECTOR_3 &&
        SectorAddr != ADDR_FLASH_SECTOR_4 && SectorAddr != ADDR_FLASH_SECTOR_5 &&
        SectorAddr != ADDR_FLASH_SECTOR_6 && SectorAddr != ADDR_FLASH_SECTOR_7)
    {
        return HAL_ERROR;
    }

    uint8_t sector = STMFLASH_GetFlashSector(SectorAddr);
    if (sector == 0xFF) return HAL_ERROR;

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;

    eraseInit.TypeErase     = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector        = sector;
    eraseInit.NbSectors     = 1;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef bsp_FlashEraseSectors(uint32_t StartSectorAddr, uint8_t NumSectors)
{
    if (NumSectors == 0) return HAL_OK;

    uint8_t startSector = STMFLASH_GetFlashSector(StartSectorAddr);
    if (startSector == 0xFF) return HAL_ERROR;
    if (startSector + NumSectors > 8) return HAL_ERROR;  /* 最多到扇区7 */

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;

    eraseInit.TypeErase     = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector        = startSector;
    eraseInit.NbSectors     = NumSectors;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    HAL_FLASH_Lock();

    return status;
}

/*----------------------------- 编程与读取 -----------------------------*/
HAL_StatusTypeDef bsp_STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumWords)
{
    /* 地址对齐检查 */
    if ((WriteAddr % 4) != 0) return HAL_ERROR;
    if (WriteAddr < STM32_FLASH_BASE) return HAL_ERROR;

    HAL_StatusTypeDef status = HAL_OK;
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < NumWords; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr + i*4, pBuffer[i]);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();
    return status;
}

void bsp_STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords)
{
    for (uint32_t i = 0; i < NumWords; i++)
    {
        pBuffer[i] = *(volatile uint32_t *)ReadAddr;
        ReadAddr += 4;
    }
}

uint8_t FLASH_read_byte(uint32_t addr)
{
    return *(volatile uint8_t *)addr;
}

/*----------------------------- 辅助检查 -----------------------------*/
uint8_t FLASH_IsSectorEmpty(uint32_t SectorAddr)
{
    /* 获取扇区大小 (通过地址判断) */
    uint32_t sectorSize = 0;
    if (SectorAddr == ADDR_FLASH_SECTOR_0) sectorSize = 16*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_1) sectorSize = 16*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_2) sectorSize = 16*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_3) sectorSize = 16*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_4) sectorSize = 64*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_5) sectorSize = 128*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_6) sectorSize = 128*1024;
    else if (SectorAddr == ADDR_FLASH_SECTOR_7) sectorSize = 128*1024;
    else return 0;

    uint32_t endAddr = SectorAddr + sectorSize;
    for (uint32_t addr = SectorAddr; addr < endAddr; addr++)
    {
        if (FLASH_read_byte(addr) != 0xFF) return 0;
    }
    return 1;
}

