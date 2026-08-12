#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "stm32f4xx_hal.h"

/*----------------------------- 基础宏定义 -----------------------------*/
#define STM32_FLASH_BASE         0x08000000UL   /* Flash 起始地址 */
#define FLASH_WAIT_TIMEOUT       50000          /* Flash 操作超时时间 (us) */

/*----------------------------- F446 扇区定义 (512KB) -----------------------------*/
#define ADDR_FLASH_SECTOR_0      0x08000000UL   /* 16 KB */
#define ADDR_FLASH_SECTOR_1      0x08004000UL   /* 16 KB */
#define ADDR_FLASH_SECTOR_2      0x08008000UL   /* 16 KB */
#define ADDR_FLASH_SECTOR_3      0x0800C000UL   /* 16 KB */
#define ADDR_FLASH_SECTOR_4      0x08010000UL   /* 64 KB */
#define ADDR_FLASH_SECTOR_5      0x08020000UL   /* 128 KB */
#define ADDR_FLASH_SECTOR_6      0x08040000UL   /* 128 KB */
#define ADDR_FLASH_SECTOR_7      0x08060000UL   /* 128 KB */

/*----------------------------- 扇区大小 -----------------------------*/
#define FLASH_SECTOR_SIZE_KB_0   16
#define FLASH_SECTOR_SIZE_KB_1   16
#define FLASH_SECTOR_SIZE_KB_2   16
#define FLASH_SECTOR_SIZE_KB_3   16
#define FLASH_SECTOR_SIZE_KB_4   64
#define FLASH_SECTOR_SIZE_KB_5   128
#define FLASH_SECTOR_SIZE_KB_6   128
#define FLASH_SECTOR_SIZE_KB_7   128

/*----------------------------- 函数声明 -----------------------------*/

/**
 * @brief 获取地址所在的 Flash 扇区编号 (0~7)
 * @param addr: Flash 地址 (必须为扇区起始地址或内部地址)
 * @retval 扇区编号 (0~7)，若地址无效则返回 0xFF
 */
uint8_t STMFLASH_GetFlashSector(uint32_t addr);

/**
 * @brief 擦除单个扇区 (基于扇区起始地址)
 * @param SectorAddr: 扇区起始地址 (必须是上面 ADDR_FLASH_SECTOR_xx 之一)
 * @retval HAL_OK: 成功, 其他: HAL 错误码
 */
HAL_StatusTypeDef bsp_IndividualSectorErasure(uint32_t SectorAddr);

/**
 * @brief 擦除从起始扇区开始的连续多个扇区
 * @param StartSectorAddr: 起始扇区地址
 * @param NumSectors: 要擦除的扇区数量
 * @retval HAL_OK: 成功, 其他: 错误码
 */
HAL_StatusTypeDef bsp_FlashEraseSectors(uint32_t StartSectorAddr, uint8_t NumSectors);

/**
 * @brief 从指定地址写入多个字 (32位)，调用前需确保目标区域已擦除
 * @param WriteAddr: 写入起始地址 (必须 4 字节对齐)
 * @param pBuffer: 要写入的数据缓冲区 (字对齐)
 * @param NumWords: 要写入的字数
 * @retval HAL_OK: 成功, 其他: 错误码
 */
HAL_StatusTypeDef bsp_STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t NumWords);

/**
 * @brief 从指定地址读取多个字 (32位)
 * @param ReadAddr: 读取起始地址 (必须 4 字节对齐)
 * @param pBuffer: 读取数据存放缓冲区
 * @param NumWords: 要读取的字数
 */
 void bsp_STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumWords);

/**
 * @brief 读取一个字节 (8位)
 * @param addr: 字节地址
 * @retval 该地址的字节值
 */
uint8_t FLASH_read_byte(uint32_t addr);

/**
 * @brief 检查整个扇区是否已被擦除 (全为 0xFF)
 * @param SectorAddr: 扇区起始地址
 * @retval 1: 扇区为空, 0: 扇区非空
 */
uint8_t FLASH_IsSectorEmpty(uint32_t SectorAddr);

#endif /* __BSP_FLASH_H */



