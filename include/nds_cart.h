#pragma once

#include <stdint.h>

void nds_cart_init(void);

void nds_cart_reset(void);
void nds_cart_rom_chip_id(uint8_t *data); // returned bytes; 4
void nds_cart_rom_read(size_t byte_addr, uint8_t *data, size_t len);
void nds_cart_read_header(uint8_t *data, size_t len);

extern unsigned char nds_cart_key[];
extern unsigned int nds_cart_key_len;
