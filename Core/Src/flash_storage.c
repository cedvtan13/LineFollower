/*
 * flash_storage.c  —  Persist PID config in internal flash (Sector 7)
 *
 *  Created on: Mar 1, 2026
 *      Author: cvt
 */

#include "flash_storage.h"
#include "stm32f4xx_hal.h"

/* ----------------------------------------------------------------
   Flash layout
   ---------------------------------------------------------------- */
#define STORAGE_SECTOR       FLASH_SECTOR_7
#define STORAGE_ADDR         0x08060000UL      /* start of Sector 7      */
#define STORAGE_MAGIC        0xDEAD1234UL      /* change if struct layout changes */
#define STORAGE_VOLTAGE      FLASH_VOLTAGE_RANGE_3   /* 2.7–3.6 V supply */

/* Raw data as it sits in flash (word-aligned) */
typedef struct {
    uint32_t magic;
    float    Kp;
    float    Ki;
    float    Kd;
    uint32_t baseSpeed;    /* stored as uint32 for word-program alignment */
} FlashData;

/* ----------------------------------------------------------------
   Public API
   ---------------------------------------------------------------- */

void FlashStorage_Load(PIDConfig *cfg)
{
    const volatile FlashData *stored = (const volatile FlashData *)STORAGE_ADDR;

    if (stored->magic != STORAGE_MAGIC) {
        /* No valid save data — keep whatever defaults the caller set */
        return;
    }

    cfg->Kp        = stored->Kp;
    cfg->Ki        = stored->Ki;
    cfg->Kd        = stored->Kd;
    cfg->baseSpeed = (uint8_t)stored->baseSpeed;
}

uint8_t FlashStorage_Save(const PIDConfig *cfg)
{
    FlashData data = {
        .magic     = STORAGE_MAGIC,
        .Kp        = cfg->Kp,
        .Ki        = cfg->Ki,
        .Kd        = cfg->Kd,
        .baseSpeed = (uint32_t)cfg->baseSpeed,
    };

    HAL_FLASH_Unlock();

    /* Erase the whole sector first (flash can only be written from 1→0, not 0→1) */
    FLASH_EraseInitTypeDef eraseInit = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = STORAGE_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = STORAGE_VOLTAGE,
    };
    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0;
    }

    /* Write word by word */
    const uint32_t *src  = (const uint32_t *)&data;
    uint32_t        addr = STORAGE_ADDR;
    for (uint8_t i = 0; i < sizeof(FlashData) / sizeof(uint32_t); i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return 1;
}
