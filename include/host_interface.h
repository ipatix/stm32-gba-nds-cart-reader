#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define XFER_MAX_PAYLOAD_SIZE 4096

#define HOST_REQ_MAGIC 0x8057

#define HOST_REQ_ROM_SEEK 0x40
#define HOST_REQ_ROM_READ 0x41
#define HOST_REQ_ROM_WRITE 0x42

#define HOST_REQ_SRAM_SEEK 0x50
#define HOST_REQ_SRAM_READ 0x51
#define HOST_REQ_SRAM_WRITE 0x52

#define HOST_REQ_FLASH_ERASE 0x60
#define HOST_REQ_FLASH_SEEK 0x61
#define HOST_REQ_FLASH_READ 0x62
#define HOST_REQ_FLASH_WRITE 0x63
#define HOST_REQ_FLASH_BANK 0x64

#define HOST_REQ_EEPROM512_SEEK 0x70
#define HOST_REQ_EEPROM512_READ 0x71
#define HOST_REQ_EEPROM512_WRITE 0x72

#define HOST_REQ_EEPROM8K_SEEK 0x80
#define HOST_REQ_EEPROM8K_READ 0x81
#define HOST_REQ_EEPROM8K_WRITE 0x82

#define HOST_REQ_UART_SYNC 0xA0

#define HOST_REQ_NDS_ROM_SEEK 0xB0
#define HOST_REQ_NDS_ROM_READ 0xB1
#define HOST_REQ_NDS_ROM_CHIPID 0xB2


//////////////////////////////////////////////

#define DEV_REPL_MAGIC 0xDE44

#define DEV_REPL_ROM_SEEK 0x140
#define DEV_REPL_ROM_READ 0x141
#define DEV_REPL_ROM_WRITE 0x142

#define DEV_REPL_SRAM_SEEK 0x150
#define DEV_REPL_SRAM_READ 0x151
#define DEV_REPL_SRAM_WRITE 0x152

#define DEV_REPL_FLASH_ERASE 0x160
#define DEV_REPL_FLASH_SEEK 0x161
#define DEV_REPL_FLASH_READ 0x162
#define DEV_REPL_FLASH_WRITE 0x163
#define DEV_REPL_FLASH_BANK 0x164

#define DEV_REPL_EEPROM512_SEEK 0x170
#define DEV_REPL_EEPROM512_READ 0x171
#define DEV_REPL_EEPROM512_WRITE 0x172

#define DEV_REPL_EEPROM8K_SEEK 0x180
#define DEV_REPL_EEPROM8K_READ 0x181
#define DEV_REPL_EEPROM8K_WRITE 0x182

#define DEV_REPL_UART_SYNC 0x1A0

#define DEV_REPL_NDS_ROM_SEEK 0x1B0
#define DEV_REPL_NDS_ROM_READ 0x1B1
#define DEV_REPL_NDS_ROM_CHIPID 0x1B2

#define DEV_REPL_ERR 0x1F0

//////////////////////////////////////////////

struct host_request {
    uint16_t magic;
    uint16_t type;
    uint16_t id;
    uint16_t len;
    uint8_t data[];
};

struct device_reply {
    uint16_t magic;
    uint16_t type;
    uint16_t id;
    uint16_t len;
    uint8_t data[];
};

void hostif_run(void);
void hostif_data_receive(const uint8_t *data, uint16_t size);
