#include <stm32f1xx_hal.h>
#include <string.h>

#include "nds_cart.h"
#include "util.h"

/*
 * Wiring:
 * PORTC[0..7]  <-> NDS_DATA[0..7]
 * PORTC[8]     <-> NDS_CLK
 * PORTC[9]     <-> NDS_ROMCS
 * PORTC[10]    <-> NDS_RESET
 * PORTC[11]    <-> NDS_EEPROMCS
 *
 * On all transfers, data should be stable at rising edge of CLK
 */

#define PIN_CLK 8u
#define PIN_CLK_MSK (1u << PIN_CLK)
#define PIN_ROMCS 9u
#define PIN_ROMCS_MSK (1u << PIN_ROMCS)
#define PIN_RESET 10u
#define PIN_RESET_MSK (1u << PIN_RESET)
#define PIN_EEPROMCS 11u
#define PIN_EEPROMCS_MSK (1u << PIN_EEPROMCS)

#define NDS_CART_DELAY() do { WAIT(400); } while (0);

__attribute__((always_inline)) static inline void clk_low(void)
{
    GPIOC->BSRR = PIN_CLK_MSK << 16;
    NDS_CART_DELAY();
}

__attribute__((always_inline)) static inline void clk_high(void)
{
    GPIOC->BSRR = PIN_CLK_MSK;
    NDS_CART_DELAY();
}

__attribute__((always_inline)) static inline void romcs_low(void)
{
    GPIOC->BSRR = PIN_ROMCS_MSK << 16;
}

__attribute__((always_inline)) static inline void romcs_high(void)
{
    GPIOC->BSRR = PIN_ROMCS_MSK;
}

__attribute__((always_inline)) static inline void reset_low(void)
{
    GPIOC->BSRR = PIN_RESET_MSK << 16;
}

__attribute__((always_inline)) static inline void reset_high(void)
{
    GPIOC->BSRR = PIN_RESET_MSK;
}

__attribute__((always_inline)) static inline void eepromcs_low(void)
{
    GPIOC->BSRR = PIN_EEPROMCS_MSK << 16;
}

__attribute__((always_inline)) static inline void eepromcs_high(void)
{
    GPIOC->BSRR = PIN_EEPROMCS_MSK;
}

__attribute__((always_inline)) static inline void data_dir_input(void)
{
    GPIOC->CRL = 0x44444444;
}

__attribute__((always_inline)) static inline uint8_t data_input(void)
{
    return (uint8_t)GPIOC->IDR;
}

__attribute__((always_inline)) static inline void data_dir_output()
{
    GPIOC->CRL = 0x11111111;
}

__attribute__((always_inline)) static inline void data_output(uint8_t v)
{
    uint8_t nv = (uint8_t)~v;
    GPIOC->BSRR = (uint32_t)(v | (nv << 16));
}

__attribute__((always_inline)) static inline uint8_t data_in_cycle(void)
{
    clk_low();
    clk_high();
    uint8_t ret = data_input();
    return ret;
}

__attribute__((always_inline)) static inline void data_out_cycle(uint8_t d)
{
    clk_low();
    data_output(d);
    clk_high();
}

/*
 * Init's all required GPIO's to access the ROM cartridge and Flash/EEPROM
 */

static struct {
    uint32_t iii;
    uint32_t jjj;
    uint32_t kkkkk;
} key1_state;

void nds_cart_init(void)
{
    // set port C[0..7] to High-Z
    data_dir_input();
    // set port C[8..11] to Output
    clk_high();
    romcs_high();
    reset_high();
    eepromcs_high();
    MODIFY_REG(GPIOC->CRH, 0xFFFFu, 0x1111u);
}

void nds_cart_reset(void)
{
    romcs_low();
    reset_low();
    clk_low();
    clk_high();
    reset_high();
    romcs_high();
    clk_low();
    clk_high();
}

void nds_cart_rom_chip_id(uint8_t *data)
{
    romcs_low();
    data_dir_output();

    data_out_cycle(0x90);
    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);

    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);

    data_dir_input();
    *data++ = data_in_cycle();
    *data++ = data_in_cycle();
    *data++ = data_in_cycle();
    *data++ = data_in_cycle();
    romcs_high();
}

void nds_cart_rom_read(size_t byte_addr, uint8_t *data, size_t len)
{
    size_t block_len = len;
    if (byte_addr < 0x1000) {
        if (byte_addr + block_len > 0x1000) {
            block_len = 0x1000 - byte_addr;
        }
        nds_cart_read_header(data, block_len);
        data += block_len;
        byte_addr += block_len;
        len -= block_len;
    }

    if (len == 0)
        return;

    block_len = len;
    if (byte_addr < 0x4000) {
        if (byte_addr + block_len > 0x4000) {
            block_len = 0x4000 - byte_addr;
        }
        /* 
         * area not used, fill with zeroes
         */
        memset(data, 0, block_len);
        data += block_len;
        byte_addr += block_len;
        len -= block_len;
    }

    if (len == 0)
        return;

    block_len = len;
    // TODO read encrypted area
}

void nds_cart_read_header(uint8_t *data, size_t len)
{
    romcs_low();
    data_dir_output();

    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);

    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);
    data_out_cycle(0x00);

    data_dir_input();

    while (len-- > 0) {
        *data++ = data_in_cycle();
    }

    romcs_high();
    clk_low();
    clk_high();
}
