#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void gba_cart_init(void);

void gba_cart_test(void);

size_t gba_cart_rom_size();
size_t gba_cart_save_size();

void gba_cart_rom_read(uint32_t word_addr, uint16_t *data, size_t len);
uint16_t gba_cart_rom_read_word(uint32_t word_addr);
void gba_cart_rom_write(uint32_t word_addr, const uint16_t *data, size_t len);
void gba_cart_rom_write_word(uint32_t word_addr, uint16_t data);

void gba_cart_sram_read(uint16_t byte_addr, uint8_t *data, size_t len);
uint8_t gba_cart_sram_read_byte(uint16_t byte_addr);
void gba_cart_sram_write(uint16_t byte_addr, const uint8_t *data, size_t len);
void gba_cart_sram_write_byte(uint16_t byte_addr, uint8_t data);

bool gba_cart_flash_erase(void);
bool gba_cart_flash_write(uint16_t byte_addr, uint8_t *data, size_t len);
void gba_cart_flash_switch_bank(uint8_t bank);

void gba_cart_eeprom_512_read_data(uint16_t block_addr, uint8_t *data);
bool gba_cart_eeprom_512_write_data(uint16_t block_addr, const uint8_t *data);
void gba_cart_eeprom_8k_read_data(uint16_t block_addr, uint8_t *data);
bool gba_cart_eeprom_8k_write_data(uint16_t block_addr, const uint8_t *data);
