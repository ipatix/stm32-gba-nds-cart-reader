#include <stm32f1xx_hal.h>

#include "gba_cart.h"
#include "util.h"

/*
 * Wiring:
 * PORTA[0..7]  <-> GBA_A[16..23]
 * PORTA[8]     <-> GBA_NWR
 * PORTA[13]    <-> GBA_NRD
 * PORTA[14]    <-> GBA_NCS
 * PORTA[15]    <-> GBA_NCS2
 *
 * PORTB[0..15] <-> GBA_AD[0..15]
 * 
 * PORTC[12]    <-> LVL_SHIFT_A
 * PORTD[2]     <-> LVL_SHIFT_AD
 */

#define PIN_NWR 8u
#define PIN_NWR_MSK (1u << PIN_NWR)
#define PIN_NRD 13u
#define PIN_NRD_MSK (1u << PIN_NRD)
#define PIN_NCS 14u
#define PIN_NCS_MSK (1u << PIN_NCS)
#define PIN_NCS2 15u
#define PIN_NCS2_MSK (1u << PIN_NCS2)

static int gba_cart_delay = 2;
#define LVL_SHIFT_DELAY() do { WAIT(0); } while (0)
#define GBA_CART_DELAY() do { WAIT(gba_cart_delay); } while (0)

//#define LVL_SHIFT_DELAY() HAL_Delay(2)
//#define GBA_CART_DELAY() HAL_Delay(2)

#define EEPROM_WORD_ADDR 0xFFFF80

__attribute__((always_inline)) static inline void adbus_level_dir_input(void)
{
    GPIOD->BSRR = 1u << 2u << 16u;
}

__attribute__((always_inline)) static inline void adbus_level_dir_output(void)
{
    GPIOD->BSRR = 1u << 2u;
}

__attribute__((always_inline)) static inline void uabus_level_dir_input(void)
{
    GPIOC->BSRR = 1u << 12u << 16u;
}

__attribute__((always_inline)) static inline void uabus_level_dir_output(void)
{
    GPIOC->BSRR = 1u << 12u;
}

__attribute__((always_inline)) static inline void adbus_dir_input(void)
{
    GPIOB->CRL = 0x44444444;
    GPIOB->CRH = 0x44444444;
    adbus_level_dir_input();
    LVL_SHIFT_DELAY();
}

__attribute__((always_inline)) static inline void adbus_dir_output(uint16_t init_value)
{
    adbus_level_dir_output();
    LVL_SHIFT_DELAY();
    GPIOB->ODR = init_value;
    GPIOB->CRL = 0x11111111;
    GPIOB->CRH = 0x11111111;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void uabus_dir_input(void)
{
    GPIOA->CRL = 0x44444444;
    uabus_level_dir_input();
    LVL_SHIFT_DELAY();
}

__attribute__((always_inline)) static inline void uabus_dir_output(uint8_t init_value)
{
    uabus_level_dir_output();
    LVL_SHIFT_DELAY();
    uint8_t ninit_value = (uint8_t)~init_value;
    GPIOA->BSRR = (uint32_t)(init_value | (ninit_value << 16));
    GPIOA->CRL = 0x11111111;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void ncs_low(void)
{
    GPIOA->BSRR = PIN_NCS_MSK << 16;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void ncs_high(void)
{
    GPIOA->BSRR = PIN_NCS_MSK;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void ncs2_low(void)
{
    GPIOA->BSRR = PIN_NCS2_MSK << 16;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void ncs2_high(void)
{
    GPIOA->BSRR = PIN_NCS2_MSK;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void nrd_low(void)
{
    GPIOA->BSRR = PIN_NRD_MSK << 16;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void nrd_high(void)
{
    GPIOA->BSRR = PIN_NRD_MSK;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void nwr_low(void)
{
    GPIOA->BSRR = PIN_NWR_MSK << 16;
    GBA_CART_DELAY();
}

__attribute__((always_inline)) static inline void nwr_high(void)
{
    GPIOA->BSRR = PIN_NWR_MSK;
    GBA_CART_DELAY();
}

/*
 * Init's all required GPIO's to access the ROM cartridge and may init specific variables
 */
void gba_cart_init(void)
{
    // port B all pins Tri-Z
    adbus_dir_input();

    // port D pin 2
    // set output level LOW
    adbus_level_dir_input();
    // set output push pull and speed 10 mhz
    MODIFY_REG(GPIOD->CRL, 0xF00u, 0x100u);

    // port C pin 12
    // set output level LOW
    uabus_level_dir_input();
    // set output push pull and speed 10 mhz
    MODIFY_REG(GPIOC->CRH, 0xF0000u, 0x10000u);

    // port A pin 8, 13, 14, 15
    // set output level HIGH
    ncs_high();
    ncs2_high();
    nrd_high();
    nwr_high();
    // set output push pull and speed 10 mhz
    MODIFY_REG(GPIOA->CRH, 0xFFF0000Fu, 0x11100001u);
    // set pin 0..7 Tri-Z
    uabus_dir_input();
}

void gba_cart_test(void)
{
    // nothing here, add code if you want
}

size_t gba_cart_rom_size()
{
    for (uint32_t i = 19; i < 24; i++) {
        // each iteration checks if all address bits >= i result mirrors
        uint32_t base_addr = 1u << i;
        uint32_t errors = 0;

        for (uint32_t j = 0; j < 1024; j++) {
            uint16_t v;
            gba_cart_rom_read(base_addr + j, &v, 1);
            if (v != j)
                errors++;
        }
        
        if (errors < 4)
            return base_addr << 1;
        /*
         * non-valid address ranges return High-Z's
         * this will cause the written address to be read again
         *
         * since a non driven bus won't be that deterministic we have
         * to tolerate a few errors
         */
    }
    return 0x2000000;
}

/*
 * returns true for 8 KiB EEPROM, false for 512 B EEPROM
 *
 * The code below is implemented by a reverse engineering
 * Rudolph's GBA Backup Tool
 * I don't understand how it works, but it seems to work
 *
 * TODO NEEDS TESTING
 */
static bool gba_cart_get_eeprom_type(void) {
    static const int NBLK = 96;
    uint8_t buffer[NBLK * 8];

    //uart_printf("determine eeprom type...\n");

    for (int i = 0; i < NBLK; i++) {
        gba_cart_eeprom_8k_read_data((uint16_t)i, &buffer[i * 8]);
    }

    //uart_printf("read 96 blocks\n");

    uint8_t ref = buffer[0];

    for (int i = 8; i < (NBLK * 8); i += 8) {
        if (buffer[i] != ref)
            return true;
    }

    //uart_printf("first 8k check failed\n");

    for (int i = 1; i < (64 * 8); i += 1) {
        if (buffer[i] != ref)
            return false;
    }

    //uart_printf("first 512 check failed\n");

    // in this case writing to eeprom won't hurt since there isn't gonna be any data on there

    for (int i = 0; i < 8; i++) {
        buffer[i] = (uint8_t)(i * 0x11);
    }

    gba_cart_eeprom_8k_write_data(0, buffer);
    gba_cart_eeprom_8k_read_data(0, &buffer[0]);
    gba_cart_eeprom_8k_read_data(1, &buffer[1]);

    // I have no clue why this would write back to the eeprom like that
    for (int i = 8; i < 16; i++) {
        if (buffer[i] != ref) {
            for (int j = 0; j < 64; j++) {
                gba_cart_eeprom_512_write_data((uint16_t)j, &buffer[j * 8 + 8]);
            }
            //uart_printf("succeeded final 512 test\n");
            return false;
        }
    }

    gba_cart_eeprom_8k_write_data(0, &buffer[8]);
    //uart_printf("succeeded final 8k test\n");
    return true;
}

size_t gba_cart_save_size()
{
    static const uint32_t BLOCK_SIZE = 512;
    uint32_t buffer[BLOCK_SIZE];
    for (uint32_t i = 0; i < 0x2000000; i += (BLOCK_SIZE * 4)) {
        gba_cart_rom_read(i / 2, (uint16_t *)buffer, BLOCK_SIZE * 2);
        for (uint32_t j = 0; j < BLOCK_SIZE; j++) {
            if (buffer[j] == *(uint32_t *)"EEPR") {
                uint32_t cmp;
                if (j + 1 == BLOCK_SIZE)
                    gba_cart_rom_read((i / 2) + (j * 2), (uint16_t *)&cmp, 2);
                else
                    cmp = buffer[j + 1];
                if ((cmp & 0xFFFFFF) == ((*(uint32_t *)"OM_") & 0xFFFFFF)) {
                    // found "EEPROM_" string
                    if (gba_cart_get_eeprom_type())
                        return 0x2000;
                    else
                        return 0x200;
                }
            } else if (buffer[j] == *(uint32_t *)"FLAS") {
                uint32_t cmp;
                if (j + 1 == BLOCK_SIZE)
                    gba_cart_rom_read((i / 2) + (j * 2), (uint16_t *)&cmp, 2);
                else
                    cmp = buffer[j + 1];
                if (cmp == *(uint32_t *)"H1M_") {
                    // found "FLASH1M_" string
                    return 0x20000;
                } else if ((cmp & 0xFFFF) == ((*(uint32_t *)"H_") & 0xFFFF)) {
                    // found "FLASH_" string
                    return 0x10000;
                }
            } else if (buffer[j] == *(uint32_t *)"SRAM") {
                uint32_t cmp;
                if (j + 1 == BLOCK_SIZE)
                    gba_cart_rom_read((i / 2) + (j * 2), (uint16_t *)&cmp, 2);
                else
                    cmp = buffer[j + 1];
                if ((cmp & 0xFF) == ((*(uint32_t *)"_") & 0xFF)) {
                    // found "SRAM_" string
                    return 0x8000;
                }
            }
        }
    }
    return 0u;
}

void gba_cart_rom_read(uint32_t word_addr, uint16_t *data, size_t len)
{
    // PORTA[0..7] and PORTB[0..15] are in Tri-Z initially
    // set them to the address bits
    adbus_dir_output((uint16_t)(word_addr & 0xFFFF));
    uint8_t word_addr_msb = (uint8_t)((word_addr >> 16) & 0xFF);
    uabus_dir_output(word_addr_msb);

    // latch address
    ncs_low();

    // remove address from data bus
    adbus_dir_input();

    while (len-- > 0) {
        nrd_low();
        // read data
        *data++ = (uint16_t)GPIOB->IDR;
        word_addr += 1;

        nrd_high();

        if (word_addr >> 16 != word_addr_msb) {
            word_addr_msb = (uint8_t)((word_addr >> 16) & 0xFF);
            ncs_high();
            uabus_dir_output(word_addr_msb);
            ncs_low();
        }
    }

    ncs_high();
    uabus_dir_input();
}

uint16_t gba_cart_rom_read_word(uint32_t word_addr)
{
    uint16_t retval;
    gba_cart_rom_read(word_addr, &retval, 1);
    return retval;
}

void gba_cart_rom_write(uint32_t word_addr, const uint16_t *data, size_t len)
{
    // PORTA[0..7] and PORTB[0..15] are in Tri-Z initially
    // set them to the address bits
    adbus_dir_output((uint16_t)(word_addr & 0xFFFF));
    uint8_t word_addr_msb = (uint8_t)((word_addr >> 16) & 0xFF);
    uabus_dir_output(word_addr_msb);

    // latch address
    ncs_low();

    while (len-- > 0) {
        adbus_dir_output(*data++);
        nwr_low();
        // TODO needs possibly more delay here?
        nwr_high();
        word_addr += 1;

        if (word_addr >> 16 != word_addr_msb) {
            word_addr_msb = (uint8_t)((word_addr >> 16) & 0xFF);
            ncs_high();
            uabus_dir_output(word_addr_msb);
            ncs_low();
        }
    }

    ncs_high();
    adbus_dir_input();
    uabus_dir_input();
}

void gba_cart_rom_write_word(uint32_t word_addr, uint16_t data)
{
    gba_cart_rom_write(word_addr, &data, 1);
}

void gba_cart_sram_read(uint16_t byte_addr, uint8_t *data, size_t len)
{
    // TODO might need additional delay
    while (len-- > 0) {
        adbus_dir_output(byte_addr++);
        ncs2_low();
        nrd_low();
        *data++ = (uint8_t)GPIOA->IDR;
        nrd_high();
        ncs2_high();
    }

    adbus_dir_input();
}

uint8_t gba_cart_sram_read_byte(uint16_t byte_addr)
{
    uint8_t retval;
    gba_cart_sram_read(byte_addr, &retval, 1);
    return retval;
}

void gba_cart_sram_write(uint16_t byte_addr, const uint8_t *data, size_t len)
{
    // TODO might need additional delay
    while (len-- > 0) {
        adbus_dir_output(byte_addr++);
        ncs2_low();
        uabus_dir_output(*data++);
        nwr_low();
        nwr_high();
        ncs2_high();
    }

    adbus_dir_input();
    uabus_dir_input();
}

void gba_cart_sram_write_byte(uint16_t byte_addr, uint8_t data)
{
    gba_cart_sram_write(byte_addr, &data, 1);
}

bool gba_cart_flash_erase(void)
{
    gba_cart_sram_write_byte(0x5555, 0xAA);
    gba_cart_sram_write_byte(0x2AAA, 0x55);
    gba_cart_sram_write_byte(0x5555, 0x80);
    gba_cart_sram_write_byte(0x5555, 0xAA);
    gba_cart_sram_write_byte(0x2AAA, 0x55);
    gba_cart_sram_write_byte(0x5555, 0x10);

    uint32_t timeout_start = HAL_GetTick();
    while (gba_cart_sram_read_byte(0x0) != 0xFF) {
        if (HAL_GetTick() >= timeout_start + 2000)
            return false;
    }
    return true;
}

bool gba_cart_flash_write(uint16_t byte_addr, uint8_t *data, size_t len)
{
    //uart_printf("writing to addr=%s with len=%s\n", itox32(byte_addr), itox32(len));
    while (len-- > 0) {
        gba_cart_sram_write_byte(0x5555, 0xAA);
        gba_cart_sram_write_byte(0x2AAA, 0x55);
        gba_cart_sram_write_byte(0x5555, 0xA0);
        gba_cart_sram_write_byte(byte_addr, *data);

        uint32_t timeout_start = HAL_GetTick();
        while (gba_cart_sram_read_byte(byte_addr) != *data) {
            if (HAL_GetTick() >= timeout_start + 1000)
                return false;
        }
        byte_addr++;
        data++;
    }
    return true;
}

void gba_cart_flash_switch_bank(uint8_t bank)
{
    gba_cart_sram_write_byte(0x5555, 0xAA);
    gba_cart_sram_write_byte(0x2AAA, 0x55);
    gba_cart_sram_write_byte(0x5555, 0xB0);
    gba_cart_sram_write_byte(0x0, bank);
}

/*
 * The large flag signals whether to access a "small" 512 byte or a large 8k byte eeprom
 * reads an 8 byte chunk
 */
static void gba_cart_eeprom_read_data(uint16_t block_addr, uint8_t *data, bool large)
{
    int gba_cart_delay_save = gba_cart_delay;
    gba_cart_delay = 8;
    {
        int address_bits = large ? 14 : 6;
        int bitstream_len = 2 + address_bits + 1;
        uint16_t bitstream[bitstream_len];
        bitstream[0] = 1;
        bitstream[1] = 1;
        for (int i = address_bits - 1, j = 0; i >= 0; i--, j++) {
            bitstream[2 + j] = (uint16_t)((block_addr >> i) & 1);
        }
        bitstream[2 + address_bits] = 0;
        gba_cart_rom_write(EEPROM_WORD_ADDR, bitstream, (size_t)bitstream_len);
    }
    {
        size_t bitstream_len = 4 + 64;
        uint16_t bitstream[bitstream_len];
        gba_cart_rom_read(EEPROM_WORD_ADDR, bitstream, bitstream_len);

        for (size_t i = 0; i < 64; i += 8) {
            int val =
                ((bitstream[4 + i + 0] & 1) << 7) |
                ((bitstream[4 + i + 1] & 1) << 6) |
                ((bitstream[4 + i + 2] & 1) << 5) |
                ((bitstream[4 + i + 3] & 1) << 4) |
                ((bitstream[4 + i + 4] & 1) << 3) |
                ((bitstream[4 + i + 5] & 1) << 2) |
                ((bitstream[4 + i + 6] & 1) << 1) |
                ((bitstream[4 + i + 7] & 1) << 0);
            data[i >> 3] = (uint8_t)val;
        }
    }
    gba_cart_delay = gba_cart_delay_save;
}

static bool gba_cart_eeprom_write_data(uint16_t block_addr, const uint8_t *data, bool large)
{
    int gba_cart_delay_save = gba_cart_delay;
    gba_cart_delay = 8;

    int address_bits = large ? 14 : 6;
    int bitstream_len = 2 + address_bits + 64 + 1;
    uint16_t bitstream[bitstream_len];
    bitstream[0] = 1;
    bitstream[1] = 0;

    for (int i = address_bits - 1, j = 0; i >= 0; i--, j++) {
        bitstream[2 + j] = (uint16_t)((block_addr >> i) & 1);
    }

    for (int i = 0; i < 64; i += 8) {
        uint8_t d = data[i >> 3];
        bitstream[2 + address_bits + i + 0] = (d >> 7) & 1;
        bitstream[2 + address_bits + i + 1] = (d >> 6) & 1;
        bitstream[2 + address_bits + i + 2] = (d >> 5) & 1;
        bitstream[2 + address_bits + i + 3] = (d >> 4) & 1;
        bitstream[2 + address_bits + i + 4] = (d >> 3) & 1;
        bitstream[2 + address_bits + i + 5] = (d >> 2) & 1;
        bitstream[2 + address_bits + i + 6] = (d >> 1) & 1;
        bitstream[2 + address_bits + i + 7] = (d >> 0) & 1;
    }

    bitstream[2 + address_bits + 64] = 0;

    gba_cart_rom_write(EEPROM_WORD_ADDR, bitstream, (size_t)bitstream_len);

    uint32_t timeout_begin = HAL_GetTick();
    while ((gba_cart_rom_read_word(EEPROM_WORD_ADDR) & 1) != 1) {
        if (HAL_GetTick() >= timeout_begin + 10) {
            gba_cart_delay = gba_cart_delay_save;
            return false;
        }
    }
    gba_cart_delay = gba_cart_delay_save;
    return true;
}

void gba_cart_eeprom_512_read_data(uint16_t block_addr, uint8_t *data)
{
    gba_cart_eeprom_read_data(block_addr, data, false);
}

bool gba_cart_eeprom_512_write_data(uint16_t block_addr, const uint8_t *data)
{
    return gba_cart_eeprom_write_data(block_addr, data, false);
}

void gba_cart_eeprom_8k_read_data(uint16_t block_addr, uint8_t *data)
{
    gba_cart_eeprom_read_data(block_addr, data, true);
}

bool gba_cart_eeprom_8k_write_data(uint16_t block_addr, const uint8_t *data)
{
    return gba_cart_eeprom_write_data(block_addr, data, true);
}
