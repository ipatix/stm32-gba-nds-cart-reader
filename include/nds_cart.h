#pragma once

#include <stdint.h>

void nds_cart_init(void);

void nds_cart_reset(void);
void nds_cart_rom_chip_id(uint8_t data[4]); // returned bytes; 4
void nds_cart_rom_read(size_t byte_addr, uint8_t *data, size_t len);
void nds_cart_read_header(uint8_t *data);
void nds_cart_read_secure_area_chunk(uint8_t *data, size_t chunk);

extern unsigned char nds_cart_key[];
extern unsigned char dsi_cart_key[];

struct nds_header {
    /* 0x0 */
    char game_title[12];
    /* 0xC */
    union {
        char game_code[4];
        uint32_t game_code_uint;
    };
    /* 0x10 */
    char maker_code[2];
    /* 0x12 */
    uint8_t uint_code;
    /* 0x13 */
    uint8_t encryption_seed_select;
    /* 0x14 */
    uint8_t device_capacity;
    /* 0x15 */
    uint8_t padding1[7];
    /* 0x1C */
    uint8_t dsi_flags;
    /* 0x1D */
    union {
        uint8_t nds_region;
        uint8_t dsi_permit_jump;
    };
    /* 0x1E */
    uint8_t rom_version;
    /* 0x1F */
    uint8_t autostart;
    /* 0x20 */
    uint32_t arm9_rom_offset;
    /* 0x24 */
    uint32_t arm9_entry_address;
    /* 0x28 */
    uint32_t arm9_ram_address;
    /* 0x2C */
    uint32_t arm9_size;
    /* 0x30 */
    uint32_t arm7_rom_offset;
    /* 0x34 */
    uint32_t arm7_entry_address;
    /* 0x38 */
    uint32_t arm7_ram_address;
    /* 0x3C */
    uint32_t arm7_size;
    /* 0x40 */
    uint32_t file_name_table_offset;
    /* 0x44 */
    uint32_t file_name_table_size;
    /* 0x48 */
    uint32_t file_allocation_table_offset;
    /* 0x4C */
    uint32_t file_allocation_table_size;
    /* 0x50 */
    uint32_t file_arm9_overlay_offset;
    /* 0x54 */
    uint32_t file_arm9_overlay_size;
    /* 0x58 */
    uint32_t file_arm7_overlay_offset;
    /* 0x5C */
    uint32_t file_arm7_overlay_size;
    /* 0x60 */
    uint32_t gamecart_bus_timing_normal;
    /* 0x64 */
    uint32_t gamecart_bus_timing_key1;
    /* 0x68 */
    uint32_t icon_title_offset;
    /* 0x6C */
    uint16_t secure_area_crc16;
    /* 0x6E */
    uint16_t secure_area_delay;
    /* 0x70 */
    uint32_t arm9_auto_load_list_hoom_ram_address;
    /* 0x74 */
    uint32_t arm7_auto_load_list_hoom_ram_address;
    /* 0x78 */
    union {
        uint8_t secure_area_disable[8];
        uint32_t secure_area_disable_uint[2];
    };
    /* 0x80 */
    uint32_t rom_size_used;
    /* 0x84 */
    uint32_t rom_header_size;
    /* 0x88 */
    uint32_t dsi_arm9_parameters_table_offset;
    /* 0x8C */
    uint32_t dsi_arm7_parameters_table_offset;
    /* 0x90 */
    uint16_t dsi_ntr_rom_region_end;
    /* 0x92 */
    uint16_t dsi_twl_region_start;
    /* 0x94 */
    uint8_t padding2[12 + 16];
    /* 0xB0 */
    char unlaunch_fastboot[16];
    /* 0xC0 */
    uint8_t nintendo_logo[0x9C];
    /* 0x15C */
    uint16_t nintento_logo_crc16;
    /* 0x15E */
    uint16_t header_checksum;
    /* 0x160 */
    uint32_t debug_rom_offset;
    /* 0x164 */
    uint32_t debug_size;
    /* 0x168 */
    uint32_t debug_ram_address;
    /* 0x16C */
    uint8_t padding3[0x53];
    /* 0x1BF */
    uint8_t dsi_hmac_flags;
    /* 0x1C0 */
    uint8_t padding4[0x17C];
    /* 0x33C */
    uint8_t dsi_hmac_icon_title[14];
    /* 0x34A */
    uint8_t padding5[0x2E];
    /* 0x378 */
    uint8_t dsi_hmac_header_arm9_arm7_areas[14];
    /* 0x386 */
    uint8_t padding6[6];
    /* 0x38C */
    uint8_t dsi_hmac_overlay_arm9_nitrofat[14];
    /* 0x39A */
    uint8_t padding7[0xBE6];
    /* 0xF80 */
    uint8_t dsi_rsa_signature[0x80];
};

_Static_assert(sizeof(struct nds_header) == 0x1000u, "wrongly sized nds_header struct");

#define NDS_PROTOCOL_MSK 0x80000000u
