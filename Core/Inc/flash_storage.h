/*
 * flash_storage.h  —  Persist PID config in internal flash (Sector 7)
 *
 * STM32F411CE flash map:
 *   Sector 0–3  16 KB each  (0x08000000 – 0x0800FFFF) — firmware
 *   Sector 4    64 KB       (0x08010000 – 0x0801FFFF) — firmware
 *   Sector 5   128 KB       (0x08020000 – 0x0803FFFF) — firmware
 *   Sector 6   128 KB       (0x08040000 – 0x0805FFFF) — firmware
 *   Sector 7   128 KB       (0x08060000 – 0x0807FFFF) ← used for storage
 *
 * Only ~20 bytes are written; the rest of the sector stays erased (0xFF).
 * A magic number validates the stored data so defaults are used on first boot.
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include "menu.h"
#include <stdint.h>

/*
 * FlashStorage_Load
 *   Reads PID values from flash into *cfg.
 *   If no valid data is found (first boot or after full-erase),
 *   *cfg is left unchanged so the caller's defaults remain active.
 */
void    FlashStorage_Load(PIDConfig *cfg);

/*
 * FlashStorage_Save
 *   Erases Sector 7 and writes *cfg.
 *   Returns 1 on success, 0 on HAL error.
 *   Interrupts are NOT disabled; avoid calling while motors are running
 *   (the ~ms erase stall is fine in menu navigation).
 */
uint8_t FlashStorage_Save(const PIDConfig *cfg);

#endif /* INC_FLASH_STORAGE_H_ */
