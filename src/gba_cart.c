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

#define LVL_SHIFT_DELAY() do { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); } while (0)
#define GBA_CART_DELAY() do { __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); } while (0)

//#define LVL_SHIFT_DELAY() HAL_Delay(1)
//#define GBA_CART_DELAY() HAL_Delay(1)

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
    adbus_dir_output(0xFFFF);
    HAL_Delay(2000);
    adbus_dir_output(0x0000);
    HAL_Delay(2000);
    adbus_dir_input();
    HAL_Delay(2000);
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
