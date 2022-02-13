//
// Created by zhengweiqian on 13/2/2022.
//
#include "hardware/regs/addressmap.h"
#include "hardware/flash.h"

#define FLASH_TARGET_OFFSET 1024 * 1536;

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void rp2040_flash_init()
{
    flash_range_erase(flash_target_contents, 1024 * 1024 * 1);
}