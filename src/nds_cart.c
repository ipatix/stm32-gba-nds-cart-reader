#include <stm32f1xx_hal.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

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

#define NDS_CART_DELAY_1() do { WAIT(nds_cart_state.gap1 * 6); } while (0)
#define NDS_CART_DELAY_2() do { WAIT(nds_cart_state.gap2 * 6); } while (0)

#define NDS_CHUNK_SIZE 0x1000
#define NDS_SMALL_HEADER_SIZE 0x200
#define NDS_EXT_HEADER_SIZE 0x1000

__attribute__((always_inline)) static inline void wait_cycles(size_t cycles) {
#pragma GCC unroll 64
    for (size_t i = 0; i < cycles; i++) {
        __asm__ volatile("nop");
    }
}

__attribute__((always_inline)) static inline void clk_low(void)
{
    GPIOC->BSRR = PIN_CLK_MSK << 16;
}

__attribute__((always_inline)) static inline void clk_high(void)
{
    GPIOC->BSRR = PIN_CLK_MSK;
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

__attribute__((always_inline)) static inline uint8_t data_in_cycle_42(void)
{
    clk_low();
    wait_cycles(8);
    uint8_t ret = data_input();
    clk_high();
    wait_cycles(8);
    return ret;
}

__attribute__((always_inline)) static inline uint8_t data_in_cycle_67(void)
{
    clk_low();
    wait_cycles(5);
    uint8_t ret = data_input();
    clk_high();
    wait_cycles(5);
    return ret;
}

__attribute__((always_inline)) static inline void data_out_cycle_42(uint8_t d)
{
    clk_low();
    data_output(d);
    wait_cycles(8);
    clk_high();
    wait_cycles(8);
}

__attribute__((always_inline)) static inline void data_out_cycle_67(uint8_t d)
{
    clk_low();
    data_output(d);
    wait_cycles(5);
    clk_high();
    wait_cycles(5);
}

static void nds_cart_read_header(uint8_t *data, bool extended);

enum nds_cart_state_t {
    NDS_CART_UNINITIALIZED,
    NDS_CART_RAW,
    NDS_CART_KEY1,
    NDS_CART_KEY2,
};

static struct {
    uint8_t state;
    bool key1_gap_clk_enabled : 1;
    bool clk_67 : 1;
    bool nds : 1;
    bool dsi : 1;
    uint8_t gap2;
    uint16_t gap1;
    uint16_t data_block_size;
    uint32_t kkkkk;
} nds_cart_state;
/*
 * Init's all required GPIO's to access the ROM cartridge and Flash/EEPROM
 */

static void nds_cart_reset(void)
{
    romcs_low();
    WAIT(4);
    reset_low();
    WAIT(2);
    clk_low();
    WAIT(2);
    clk_high();
    WAIT(2);
    reset_high();
    WAIT(4);
    romcs_high();

    nds_cart_state.state = NDS_CART_UNINITIALIZED;
    nds_cart_state.key1_gap_clk_enabled = false;
    nds_cart_state.clk_67 = false;
    nds_cart_state.nds = true;
    nds_cart_state.dsi = false;
    // gaps below add up to GBATEK's 0x910 gap
    nds_cart_state.gap1 = 0x8F8;
    nds_cart_state.gap2 = 0x18;
    nds_cart_state.data_block_size = 0x200;
    // TODO reset encryption state vaues
}

static void nds_cart_begin_raw(void)
{
    // parse header to set configuration for correct ROM access
    myassert(nds_cart_state.state == NDS_CART_UNINITIALIZED, "cart must be reset before beginning raw read\r\n", 5);

    struct nds_header hd;
    nds_cart_read_header((uint8_t *)&hd, false);

    nds_cart_state.key1_gap_clk_enabled = hd.gamecart_bus_timing_key1.key1_clk_gap;
    nds_cart_state.gap1 = hd.gamecart_bus_timing_normal.key1_gap1_len;
    nds_cart_state.gap2 = hd.gamecart_bus_timing_normal.key1_gap2_len;
    nds_cart_state.clk_67 = hd.gamecart_bus_timing_normal.clk_rate;

    if (hd.unit_code == 0x00) {
        nds_cart_state.nds = true;
        nds_cart_state.dsi = false;
    } else if (hd.unit_code == 0x02) {
        nds_cart_state.nds = true;
        nds_cart_state.dsi = true;
    } else if (hd.unit_code == 0x03) {
        nds_cart_state.nds = false;
        nds_cart_state.dsi = true;
    } else {
        nds_cart_state.nds = false;
        nds_cart_state.dsi = false;
    }

    static const uint16_t len[8] = {
        0, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 4,
    };

    nds_cart_state.data_block_size = len[hd.gamecart_bus_timing_key1.data_blk_size];
    
    nds_cart_state.state = NDS_CART_RAW;
}

static void nds_cart_begin_key1(void)
{
    // init blowfish for secure area loading
    myassert(nds_cart_state.state == NDS_CART_RAW, "cart must be in raw mode before beginning key1\r\n");


}

static void nds_cart_begin_key2(void)
{
    // init xor cipher for main data transfer
    myassert(nds_cart_state.state == NDS_CART_KEY1, "cart must be in key1 mode before beginning key2\r\n");
}

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

    nds_cart_reset();
}

static void nds_cart_change_state(uint8_t state)
{
    if (state == NDS_CART_UNINITIALIZED) {
        nds_cart_reset();
    } else if (state == NDS_CART_RAW && nds_cart_state.state != NDS_CART_RAW) {
        nds_cart_reset();
        nds_cart_begin_raw();
    } else if (state == NDS_CART_KEY1 && nds_cart_state.state != NDS_CART_KEY1) {
        nds_cart_reset();
        nds_cart_begin_raw();
        nds_cart_begin_key1();
    } else if (state == NDS_CART_KEY2 && nds_cart_state.state != NDS_CART_KEY2) {
        nds_cart_reset();
        nds_cart_begin_raw();
        nds_cart_begin_key1();
        nds_cart_begin_key2();
    } else {
        myassert(false, "switching to invalid state: 0x%x\r\n", (uint32_t)state);
    }
}

static void nds_cart_exec_raw_command(const uint8_t cmd[8], uint8_t *data, size_t len)
{
    romcs_low();
    data_dir_output();

    if (nds_cart_state.clk_67) {
        for (size_t i = 0; i < 8; i++) {
            data_out_cycle_67(cmd[i]);
        }

        data_dir_input();

        NDS_CART_DELAY_1();

        if (data != NULL) {
            while (len > 0) {
                NDS_CART_DELAY_2();
                size_t block_size = len;
                if (block_size > nds_cart_state.data_block_size)
                    block_size = nds_cart_state.data_block_size;

                // mind the difference to the case below
                for (size_t i = 0; i < block_size; i++)
                    *data++ = data_in_cycle_67();

                len -= block_size;
            }
        } else {
            while (len > 0) {
                NDS_CART_DELAY_2();
                size_t block_size = len;
                if (block_size > nds_cart_state.data_block_size)
                    block_size = nds_cart_state.data_block_size;

                // mind the difference to the case above
                for (size_t i = 0; i < block_size; i++)
                    (void)data_in_cycle_67();

                len -= block_size;
            }
        }
    } else {
        for (size_t i = 0; i < 8; i++) {
            data_out_cycle_42(cmd[i]);
        }

        data_dir_input();

        NDS_CART_DELAY_1();

        if (data != NULL) {
            while (len > 0) {
                NDS_CART_DELAY_2();
                size_t block_size = len;
                if (block_size > nds_cart_state.data_block_size)
                    block_size = nds_cart_state.data_block_size;

                // mind the difference to the case below
                for (size_t i = 0; i < block_size; i++)
                    *data++ = data_in_cycle_42();

                len -= block_size;
            }
        } else {
            while (len > 0) {
                NDS_CART_DELAY_2();
                size_t block_size = len;
                if (block_size > nds_cart_state.data_block_size)
                    block_size = nds_cart_state.data_block_size;

                // mind the difference to the case above
                for (size_t i = 0; i < block_size; i++)
                    (void)data_in_cycle_42();

                len -= block_size;
            }
        }
    }

    data_dir_input();
    romcs_high();
    WAIT(2);
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

    if (nds_cart_state.clk_67) {
    } else {
    }

    if (protocol_rev) {
        // "new" revision
    } else {
        // "old" revision
        data_out_cycle_67(u.local_cmd[0]);
        data_out_cycle_67(u.local_cmd[1]);
        data_out_cycle_67(u.local_cmd[2]);
        data_out_cycle_67(u.local_cmd[3]);
        data_out_cycle_67(u.local_cmd[4]);
        data_out_cycle_67(u.local_cmd[5]);
        data_out_cycle_67(u.local_cmd[6]);
        data_out_cycle_67(u.local_cmd[7]);

        data_dir_input();

        for (int i = 0; i < 0x910; i++) {
            // TODO
        }
    }
}

static void nds_cart_rom_read_chunk(uint8_t *read_buffer, size_t chunk_addr)
{
    uart_printf("read_chunk=%x\r\n", chunk_addr);

    chunk_addr &= ~0xFFFu;

    if (chunk_addr == 0) {
        // read header
        // TODO conditional extended header read
        nds_cart_read_header(read_buffer, true);
    } else if (chunk_addr < 0x4000) {
        // read unused area (zeroes)
        memset(read_buffer, 0, NDS_CHUNK_SIZE);
    } else if (chunk_addr < 0x8000) {
        // read secure area
        uart_printf("Reading from secure area is not yet implemented");
        memset(read_buffer, 0x5C, NDS_CHUNK_SIZE);
    } else {
        // read data
        uart_printf("Reading from normal regions not yet implemented");
        memset(read_buffer, 0xBA, NDS_CHUNK_SIZE);
    }
}

void nds_cart_rom_read(size_t byte_addr, uint8_t *data, size_t len)
{
    uint8_t read_buffer[NDS_CHUNK_SIZE];

    while (len > 0) {
        size_t chunk_addr = byte_addr & ~0xFFFu;
        size_t chunk_index = byte_addr & 0xFFFu;

        size_t read_len = NDS_CHUNK_SIZE - chunk_index;
        if (read_len > len)
            read_len = len;

        nds_cart_rom_read_chunk(read_buffer, chunk_addr);
        memcpy(data, &read_buffer[chunk_index], read_len);
        byte_addr += read_len;
        len -= read_len;
        data += read_len;
    }
}

static void nds_cart_read_header(uint8_t *data, bool extended)
{
    static const uint8_t cmd[8] = {
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };

    if (extended)
        nds_cart_exec_raw_command(cmd, data, NDS_EXT_HEADER_SIZE);
    else
        nds_cart_exec_raw_command(cmd, data, NDS_SMALL_HEADER_SIZE);
}

void nds_cart_rom_chip_id(uint8_t data[4])
{
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
    // TODO rewrite
    struct nds_header hd;
    static_assert(sizeof(hd) == NDS_CHUNK_SIZE);
    nds_cart_read_header((uint8_t *)&hd, false);

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
        data_out_cycle_67(cu.cmd[i]);
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
