#pragma once

#include <stdint.h>

void gba_cart_init(void);
void gba_cart_test(void);
void gba_cart_rom_read(uint32_t word_addr, uint16_t *data, size_t len);
