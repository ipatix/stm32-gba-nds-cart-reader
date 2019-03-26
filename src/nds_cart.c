#include <stm32f1xx_hal.h>
#include <string.h>
#include <stdbool.h>

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
    // TODO, is this right?
    clk_high();
}

/*
 * Init's all required GPIO's to access the ROM cartridge and Flash/EEPROM
 */

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

static void nds_cart_exec_raw_command(const uint8_t cmd[8], uint8_t *data, size_t len)
{
    romcs_low();
    data_dir_output();

    data_out_cycle(cmd[0]);
    data_out_cycle(cmd[1]);
    data_out_cycle(cmd[2]);
    data_out_cycle(cmd[3]);
    data_out_cycle(cmd[4]);
    data_out_cycle(cmd[5]);
    data_out_cycle(cmd[6]);
    data_out_cycle(cmd[7]);

    data_dir_input();

    if (data != NULL) {
        for (size_t i = 0; i < len; i++) {
            *data++ = data_in_cycle();
        }
    } else {
        for (size_t i = 0; i < len; i++) {
            data_in_cycle();
        }
    }

    romcs_high();
    clk_low();
    clk_high();
}

static void encrypt_64bit(uint32_t *data);
static void decrypt_64bit(uint32_t *data);

/*
 * KEY1 cmd procedure:
 *  if bit31 = 0:
 *      cmd to bus with key1
 *      read dummy 0x910 bytes with key2
 *      read data with len with key2
 *  if bit31 = 1:
 *      cmd to bus with key1
 *      read dummy 0x910 bytes (without clk)
 *      secure area delay
 *      cmd to bus with key1
 *      read data with len with key2
 */

static void nds_cart_exec_key1_command(uint8_t *cmd, uint8_t *data, size_t len, int iteration, bool protocol_rev)
{
    union {
        uint8_t local_cmd[8];
        uint32_t local_wcmd[2];
    } u;
    for (int i = 0; i < 8; i++)
        u.local_cmd[i] = cmd[i];

    encrypt_64bit(u.local_wcmd);

    romcs_low();
    data_dir_output();

    if (protocol_rev) {
        // "new" revision
    } else {
        // "old" revision
        data_out_cycle(u.local_cmd[0]);
        data_out_cycle(u.local_cmd[1]);
        data_out_cycle(u.local_cmd[2]);
        data_out_cycle(u.local_cmd[3]);
        data_out_cycle(u.local_cmd[4]);
        data_out_cycle(u.local_cmd[5]);
        data_out_cycle(u.local_cmd[6]);
        data_out_cycle(u.local_cmd[7]);

        data_dir_input();

        for (int i = 0; i < 0x910; i++) {
            // TODO
        }
    }
}

static struct {
    uint32_t kkkkk;
} key1_state;

void nds_cart_rom_read(size_t byte_addr, uint8_t *data, size_t len)
{
    byte_addr &= ~0xFFFu;
    len &= ~0xFFFu;

    if (byte_addr == 0x0) {
        nds_cart_read_header(data, len);
        data += 0x1000;
        byte_addr += 0x1000;
        len -= 0x1000;
    }

    if (len == 0)
        return;

    size_t block_len = len;
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
    if (byte_addr < 0x8000) {
        if (byte_addr + block_len > 0x8000) {
            block_len = 0x8000 - byte_addr;
        }
        size_t block_i = (byte_addr - 0x4000) >> 12;
        byte_addr += block_len;
        len -= block_len;

        while (block_len > 0) {
            uart_printf("reading secure area to 0x%s, block %u\n", data, block_i);
            nds_cart_read_secure_area_chunk(data, block_i++);
            data += 0x1000;
            block_len -= 0x1000;
        }
    }

    if (len == 0)
        return;

    // TODO read game cart data
}

void nds_cart_read_header(uint8_t *data, size_t len)
{
    nds_cart_reset();

    static const uint8_t cmd[8] = {
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };

    nds_cart_exec_raw_command(cmd, data, len);
}

void nds_cart_rom_chip_id(uint8_t data[4])
{
    nds_cart_reset();

    static const uint8_t cmd[8] = {
        0x90, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };

    nds_cart_exec_raw_command(cmd, data, 4);
}

static void init_keycode(uint32_t idcode, uint32_t level, uint32_t modulo, bool dsi);

/*
 *
 */

void nds_cart_read_secure_area_chunk(uint8_t *data, size_t chunk)
{
    static struct nds_header hd;
    nds_cart_read_header((uint8_t *)&hd, sizeof(hd));

    union {
        uint32_t wid;
        uint8_t id[4];
    } u;
    nds_cart_rom_chip_id(u.id);
    
    // decrypt secure area

    uint32_t idcode = hd.game_code_uint;
    init_keycode(idcode, 1, 8, false);
    decrypt_64bit(hd.secure_area_disable_uint);
    init_keycode(idcode, 2, 8, false);

    uint32_t kkkkk = 0;

    static const uint8_t cmd[8] = {
        0x3C, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    union {
        uint8_t cmd[8];
        uint32_t cmd_uint[2];
    } cu;

    memcpy(cu.cmd, cmd, sizeof(cmd));
    encrypt_64bit(cu.cmd_uint);

    romcs_low();
    data_dir_output();

    for (int i = 0; i < 8; i++)
        data_out_cycle(cu.cmd[i]);
    // TODO
}

/*
 * KEY1 encryption stuff
 */

static struct {
    uint32_t P[16 + 2];
    uint32_t S[4][256];
} blowfish_buf;

static void encrypt_64bit(uint32_t *data)
{
    uint32_t y = data[0];
    uint32_t x = data[1];

    for (int i = 0; i <= 15; i++) {
        uint32_t z = blowfish_buf.P[i] ^ x;
        x = blowfish_buf.S[0][(z >> 24) & 0xFF];
        x = blowfish_buf.S[1][(z >> 16) & 0xFF] + x;
        x = blowfish_buf.S[2][(z >> 8) & 0xFF] ^ x;
        x = blowfish_buf.S[3][(z >> 0) & 0xFF] + x;
        x = y ^ x;
        y = z;
    }

    data[0] = x ^ blowfish_buf.P[16];
    data[1] = y ^ blowfish_buf.P[17];
}

static void decrypt_64bit(uint32_t *data)
{
    uint32_t y = data[0];
    uint32_t x = data[1];

    for (int i = 17; i >= 2; i--) {
        uint32_t z = blowfish_buf.P[i] ^ x;
        x = blowfish_buf.S[0][(z >> 24) & 0xFF];
        x = blowfish_buf.S[1][(z >> 16) & 0xFF] + x;
        x = blowfish_buf.S[2][(z >> 8) & 0xFF] ^ x;
        x = blowfish_buf.S[3][(z >> 0) & 0xFF] + x;
        x = y ^ x;
        y = z;
    }

    data[0] = x ^ blowfish_buf.P[1];
    data[1] = y ^ blowfish_buf.P[0];
}

static void apply_keycode(uint32_t *keycode, uint32_t modulo)
{
    uint32_t scratch[2];
    scratch[0] = scratch[1] = 0;

    encrypt_64bit(&keycode[4]);
    encrypt_64bit(&keycode[0]);

    for (uint32_t i = 0; i < 18; i++) {
        blowfish_buf.P[i] ^= __REV(keycode[i % modulo]);
    }

    for (uint32_t i = 0; i < 4; i++) {
        for (uint32_t j = 0; j < 256; j += 2) {
            encrypt_64bit(scratch);
            blowfish_buf.S[i][j+0] = scratch[1];
            blowfish_buf.S[i][j+1] = scratch[0];
        }
    }
}

static void init_keycode(uint32_t idcode, uint32_t level, uint32_t modulo, bool dsi)
{
    uint32_t keycode[3];

    if (dsi)
        memcpy(&blowfish_buf, dsi_cart_key, sizeof(blowfish_buf));
    else
        memcpy(&blowfish_buf, nds_cart_key, sizeof(blowfish_buf));

    keycode[0] = idcode;
    keycode[1] = idcode / 2;
    keycode[2] = idcode * 2;

    if (level >= 1)
        apply_keycode(keycode, modulo);
    if (level >= 2)
        apply_keycode(keycode, modulo);

    keycode[1] *= 2;
    keycode[2] /= 2;

    if (level >= 3)
        apply_keycode(keycode, modulo);
}
