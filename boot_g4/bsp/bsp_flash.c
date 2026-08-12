#include "bsp_flash.h"


uint32_t STMFLASH_GetFlashSize(void)
{
#if defined(FLASH_SIZE)
    return FLASH_SIZE;
#elif defined(FLASHSIZE_BASE)
    return ((uint32_t)(*(__IO uint16_t *)FLASHSIZE_BASE)) * 1024U;
#else
    return 512U * 1024U;
#endif
}

uint32_t STMFLASH_GetFlashEndAddr(void)
{
    return STM32_FLASH_BASE + STMFLASH_GetFlashSize();
}

static uint8_t STMFLASH_IsFlashAddr(uint32_t addr)
{
    return (addr >= STM32_FLASH_BASE) &&
           (addr < STMFLASH_GetFlashEndAddr());
}

static uint8_t STMFLASH_IsDualBank(void)
{
#if defined(FLASH_BANK_2) && defined(FLASH_OPTR_DBANK)
    return ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U) ? 1U : 0U;
#elif defined(FLASH_BANK_2) && defined(FLASH_BANK_SIZE)
    return 1U;
#else
    return 0U;
#endif
}

static uint32_t STMFLASH_GetErasePageSize(void)
{
#if defined(FLASH_OPTR_DBANK) && defined(FLASH_PAGE_SIZE_128_BITS)
    if (STMFLASH_IsDualBank() == 0U)
    {
        return FLASH_PAGE_SIZE_128_BITS;
    }
#endif

    return STM32_FLASH_PAGE_SIZE;
}

uint32_t STMFLASH_GetFlashPageSize(void)
{
    return STMFLASH_GetErasePageSize();
}

uint32_t STMFLASH_GetFlashPageAddress(uint32_t page)
{
    uint32_t pageSize = STMFLASH_GetErasePageSize();
    uint32_t flashSize = STMFLASH_GetFlashSize();
    uint32_t offset = page * pageSize;

    if (((offset / pageSize) != page) || (offset >= flashSize))
    {
        return STM32_FLASH_INVALID_ADDRESS;
    }

    return STM32_FLASH_BASE + offset;
}

uint32_t STMFLASH_GetFlashSectorAddress(uint32_t sector)
{
    return STMFLASH_GetFlashPageAddress(sector);
}

uint8_t STMFLASH_IsValidRange(uint32_t addr, uint32_t bytes)
{
    uint32_t flashEnd = STMFLASH_GetFlashEndAddr();

    if (bytes == 0U)
    {
        return ((addr >= STM32_FLASH_BASE) && (addr <= flashEnd)) ? 1U : 0U;
    }

    return ((addr >= STM32_FLASH_BASE) &&
            (addr < flashEnd) &&
            (bytes <= (flashEnd - addr))) ? 1U : 0U;
}

static uint8_t STMFLASH_IsPageStartAddr(uint32_t addr)
{
    uint32_t pageSize = STMFLASH_GetErasePageSize();

    return STMFLASH_IsFlashAddr(addr) &&
           (((addr - STM32_FLASH_BASE) % pageSize) == 0U);
}

static uint8_t STMFLASH_IsBankSwapped(void)
{
#if defined(FLASH_BANK_2) && defined(FLASH_BANK_SIZE) && defined(SYSCFG_MEMRMP_FB_MODE)
    return (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) != 0U) ? 1U : 0U;
#else
    return 0U;
#endif
}

static uint32_t STMFLASH_GetFlashBank(uint32_t addr)
{
#if defined(FLASH_BANK_2) && defined(FLASH_BANK_SIZE)
    if (STMFLASH_IsDualBank() != 0U)
    {
        if (STMFLASH_IsBankSwapped() != 0U)
        {
            return (addr < (STM32_FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_2 : FLASH_BANK_1;
        }

        return (addr < (STM32_FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
    }
#endif

    return FLASH_BANK_1;
}

static uint32_t STMFLASH_GetFlashBankBase(uint32_t bank)
{
#if defined(FLASH_BANK_2) && defined(FLASH_BANK_SIZE)
    if (STMFLASH_IsDualBank() != 0U)
    {
        if (STMFLASH_IsBankSwapped() != 0U)
        {
            return (bank == FLASH_BANK_1) ? (STM32_FLASH_BASE + FLASH_BANK_SIZE) : STM32_FLASH_BASE;
        }

        return (bank == FLASH_BANK_2) ? (STM32_FLASH_BASE + FLASH_BANK_SIZE) : STM32_FLASH_BASE;
    }
#else
    (void)bank;
#endif

    return STM32_FLASH_BASE;
}

/*----------------------------- 辅助函数 -----------------------------*/
uint32_t STMFLASH_GetFlashPage(uint32_t addr)
{
    if (!STMFLASH_IsFlashAddr(addr))
    {
        return STM32_FLASH_INVALID_PAGE;
    }

    uint32_t bank = STMFLASH_GetFlashBank(addr);
    return (addr - STMFLASH_GetFlashBankBase(bank)) / STMFLASH_GetErasePageSize();
}

uint32_t STMFLASH_GetFlashSector(uint32_t addr)
{
    return STMFLASH_GetFlashPage(addr);
}

/*----------------------------- 页擦除 -----------------------------*/
HAL_StatusTypeDef bsp_IndividualPageErasure(uint32_t PageAddr)
{
    if (!STMFLASH_IsPageStartAddr(PageAddr))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0U;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks     = STMFLASH_GetFlashBank(PageAddr);
    eraseInit.Page      = STMFLASH_GetFlashPage(PageAddr);
    eraseInit.NbPages   = 1U;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef bsp_IndividualSectorErasure(uint32_t SectorAddr)
{
    return bsp_IndividualPageErasure(SectorAddr);
}

HAL_StatusTypeDef bsp_FlashErasePages(uint32_t StartPageAddr, uint32_t NumPages)
{
    if (NumPages == 0U)
    {
        return HAL_OK;
    }

    if (!STMFLASH_IsPageStartAddr(StartPageAddr))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0U;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0U; i < NumPages; i++)
    {
        uint32_t pageSize = STMFLASH_GetErasePageSize();
        uint32_t offset = i * pageSize;
        uint32_t pageAddr = StartPageAddr + offset;
        if (((offset / pageSize) != i) || (pageAddr < StartPageAddr))
        {
            status = HAL_ERROR;
            break;
        }

        if (!STMFLASH_IsPageStartAddr(pageAddr))
        {
            status = HAL_ERROR;
            break;
        }

        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.Banks     = STMFLASH_GetFlashBank(pageAddr);
        eraseInit.Page      = STMFLASH_GetFlashPage(pageAddr);
        eraseInit.NbPages   = 1U;

        status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef bsp_FlashEraseSectors(uint32_t StartSectorAddr, uint8_t NumSectors)
{
    return bsp_FlashErasePages(StartSectorAddr, (uint32_t)NumSectors);
}

/*----------------------------- 编程与读取 -----------------------------*/
HAL_StatusTypeDef bsp_STMFLASH_Write_64Bit(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumWords)
{
	static uint64_t temp_data=0;
    if (((WriteAddr % 8U) != 0U) || !STMFLASH_IsFlashAddr(WriteAddr))
    {
        return HAL_ERROR;
    }

    if ((pBuffer == NULL) ||NumWords == 0U)
    {
        return HAL_ERROR;
    }

    uint32_t writeBytes = NumWords * (uint32_t)sizeof(uint32_t);
    if (((writeBytes / (uint32_t)sizeof(uint32_t)) != NumWords) ||
        !STMFLASH_IsValidRange(WriteAddr, writeBytes))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;

    HAL_FLASH_Unlock();

    for (uint32_t i = 0U; i < NumWords; i ++)
    {

		temp_data =(((uint64_t)0xFFFFFFFF<<32)|pBuffer[i]);
       
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   WriteAddr + (i * sizeof(uint32_t)),
                                   temp_data);
        if (status != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef bsp_STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumWords)
{
    // 参数检查：地址必须 8 字节对齐
    if ((WriteAddr % 8U) != 0U) {
        return HAL_ERROR;
    }
    
    if ((pBuffer == NULL) || (NumWords == 0U)) {
        return HAL_ERROR;
    }
    
    // 检查地址范围
    if (!STMFLASH_IsValidRange(WriteAddr, NumWords * 4U)) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_OK;
    HAL_FLASH_Unlock();
    
    // 每次处理 8 字节（2 个 uint32_t）
    uint32_t write_count = 0;
    while (write_count < NumWords) {
        uint64_t temp_data = 0;
        uint32_t current_addr = WriteAddr + write_count * 4U;
        
        // 读取当前地址的 8 字节原始数据
        temp_data = *((volatile uint64_t *)current_addr);
        
        // 替换低 4 字节（第一个 uint32_t）
        temp_data &= ~((uint64_t)0xFFFFFFFF);
        temp_data |= (uint64_t)pBuffer[write_count];
        
        // 如果还有下一个 4 字节数据，替换高 4 字节
        if ((write_count + 1) < NumWords) {
            temp_data &= ~((uint64_t)0xFFFFFFFF << 32);
            temp_data |= ((uint64_t)pBuffer[write_count + 1] << 32);
        }
        
        // 写入 8 字节
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   current_addr, temp_data);
        if (status != HAL_OK) {
            break;
        }
        
        write_count += 2;  // 每次处理 2 个 uint32_t
    }
    
    HAL_FLASH_Lock();
    return status;
}


HAL_StatusTypeDef bsp_STMFLASH_ReadChecked(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords)
{
    if (NumWords == 0U)
    {
        return HAL_ERROR;
    }

    if (pBuffer == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t readBytes = NumWords * (uint32_t)sizeof(uint32_t);
    if (((readBytes / (uint32_t)sizeof(uint32_t)) != NumWords) ||
        !STMFLASH_IsValidRange(ReadAddr, readBytes))
    {
        return HAL_ERROR;
    }

    for (uint32_t i = 0U; i < NumWords; i++)
    {
        pBuffer[i] = *(volatile uint32_t *)ReadAddr;
        ReadAddr += sizeof(uint32_t);
    }

    return HAL_OK;
}

void bsp_STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords)
{
    (void)bsp_STMFLASH_ReadChecked(ReadAddr, pBuffer, NumWords);
}

uint8_t FLASH_read_byte(uint32_t addr)
{
    if (!STMFLASH_IsValidRange(addr, 1U))
    {
        return 0xFFU;
    }

    return *(volatile uint8_t *)addr;
}

/*----------------------------- 辅助检查 -----------------------------*/
uint8_t FLASH_IsPageEmpty(uint32_t PageAddr)
{
    if (!STMFLASH_IsPageStartAddr(PageAddr))
    {
        return 0U;
    }

    uint32_t endAddr = PageAddr + STMFLASH_GetErasePageSize();
    for (uint32_t addr = PageAddr; addr < endAddr; addr++)
    {
        if (FLASH_read_byte(addr) != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t FLASH_IsSectorEmpty(uint32_t SectorAddr)
{
    return FLASH_IsPageEmpty(SectorAddr);
}
