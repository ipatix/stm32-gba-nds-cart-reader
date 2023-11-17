#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "gba_cart.h"
#include "nds_cart.h"
#include "host_interface.h"
#include "util.h"

#define REQ_BUF_SIZE (XFER_MAX_PAYLOAD_SIZE + sizeof(struct host_request))
#define REPL_BUF_SIZE (XFER_MAX_PAYLOAD_SIZE + sizeof(struct device_reply))

// Transfer variables

static uint8_t request_buffer_a[REQ_BUF_SIZE];
static uint8_t request_buffer_b[REQ_BUF_SIZE];
static struct host_request *const request_a = (struct host_request *)request_buffer_a;
static struct host_request *const request_b = (struct host_request *)request_buffer_b;
static volatile bool request_a_available = false;
static volatile bool request_b_available = false;

static uint8_t reply_buffer[REPL_BUF_SIZE];
static struct device_reply *const reply = (struct device_reply *)reply_buffer;

// private variables

struct {
    size_t rom_word_addr;
    uint16_t sram_byte_addr;
    uint16_t eeprom_block_addr;
} gba;

struct {
    size_t rom_byte_addr;
} nds;

// private functions

static void hostif_handle_request(struct host_request *const rq, struct device_reply *const rp);
static void hostif_send_reply(void);
static void hostif_reply_err(struct device_reply *const rp, const char *msg, ...);

void hostif_run(void)
{
    while (1) {
        if (request_a_available) {
            hostif_handle_request(request_a, reply);
            request_a_available = false;
        } else if (request_b_available) {
            hostif_handle_request(request_b, reply);
            request_b_available = false;
        }
    }
}

static void hostif_handle_request(struct host_request *const rq, struct device_reply *const rp)
{
    rp->magic = DEV_REPL_MAGIC;
    rp->id = rq->id;
    switch (rq->type) {
    case HOST_REQ_ROM_SEEK:
        if (rq->len != 4) {
            hostif_reply_err(rp,
                    "ERROR: gba rom address seek: len != 4: %s", itox32(rq->len));
        } else {
            gba.rom_word_addr = (size_t)(rq->data[0] | (rq->data[1] << 8) | (rq->data[2] << 16) | (rq->data[3] << 24));
            gba.rom_word_addr >>= 1; // convert byte to word addr
            rp->type = DEV_REPL_ROM_SEEK;
            rp->len = 0;
        }
        break;
    case HOST_REQ_ROM_READ:
        if (rq->len != 2) {
            hostif_reply_err(rp,
                    "ERROR: gba rom read len: len != 2: %s", itox32(rq->len));
        } else {
            uint16_t len = (uint16_t)(rq->data[0] | (rq->data[1] << 8));
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            gba_cart_rom_read(gba.rom_word_addr, (uint16_t *)rp->data, len / 2);
            gba.rom_word_addr += (uint16_t)(len / 2);
            rp->type = DEV_REPL_ROM_READ;
            rp->len = len;
        }
        break;
    case HOST_REQ_ROM_WRITE:
        {
            uint16_t len = rq->len;
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            gba_cart_rom_write(gba.rom_word_addr, (uint16_t *)rq->data, len / 2);
            gba.rom_word_addr += (uint16_t)(len / 2);
            rp->type = DEV_REPL_ROM_WRITE;
            rp->len = 0;
        }
        break;
    case HOST_REQ_ROM_SIZE:
        {
            rp->type = DEV_REPL_ROM_SIZE;
            size_t s = gba_cart_rom_size();
            rp->data[0] = (uint8_t)s;
            rp->data[1] = (uint8_t)(s >> 8);
            rp->data[2] = (uint8_t)(s >> 16);
            rp->data[3] = (uint8_t)(s >> 24);
            rp->len = 4;
        }
        break;
    case HOST_REQ_BKP_TYPE:
        {
            rp->type = DEV_REPL_BKP_TYPE;
            size_t s = gba_cart_save_size();
            rp->data[0] = (uint8_t)s;
            rp->data[1] = (uint8_t)(s >> 8);
            rp->data[2] = (uint8_t)(s >> 16);
            rp->data[3] = (uint8_t)(s >> 24);
            rp->len = 4;
        }
        break;
    case HOST_REQ_SRAM_SEEK:
    case HOST_REQ_FLASH_SEEK:
        if (rq->len != 2) {
            hostif_reply_err(rp,
                    "ERROR: gba sram address seek: len != 2: %s", itox32(rq->len));
        } else {
            gba.sram_byte_addr = (uint16_t)(rq->data[0] | (rq->data[1] << 8));
            if (rq->type == HOST_REQ_SRAM_SEEK)
                rp->type = DEV_REPL_SRAM_SEEK;
            else
                rp->type = DEV_REPL_FLASH_SEEK;
            rp->len = 0;
        }
        break;
    case HOST_REQ_SRAM_READ:
    case HOST_REQ_FLASH_READ:
        if (rq->len != 2) {
            hostif_reply_err(rp,
                    "ERROR: gba sram read len: len != 2: %s", itox32(rq->len));
        } else {
            uint16_t len = (uint16_t)(rq->data[0] | (rq->data[1] << 8));
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            gba_cart_sram_read(gba.sram_byte_addr, rp->data, len);
            gba.sram_byte_addr = (uint16_t)(gba.sram_byte_addr + len);
            if (rq->type == HOST_REQ_SRAM_READ)
                rp->type = DEV_REPL_SRAM_READ;
            else
                rp->type = DEV_REPL_FLASH_READ;
            rp->len = len;
        }
        break;
    case HOST_REQ_SRAM_WRITE:
        {
            uint16_t len = rq->len;
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            gba_cart_sram_write(gba.sram_byte_addr, rq->data, len);
            gba.sram_byte_addr = (uint16_t)(gba.sram_byte_addr + len);
            rp->type = DEV_REPL_SRAM_WRITE;
            rp->len = 0;
        }
        break;
    case HOST_REQ_FLASH_ERASE:
        if (gba_cart_flash_erase()) {
            rp->type = DEV_REPL_FLASH_ERASE;
            rp->len = 0;
        } else {
            hostif_reply_err(rp, "ERROR: gba flash erase timed out");
        }
        break;
    case HOST_REQ_FLASH_WRITE:
        {
            uint16_t len = rq->len;
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            if (gba_cart_flash_write(gba.sram_byte_addr, rq->data, len)) {
                gba.sram_byte_addr = (uint16_t)(gba.sram_byte_addr + len);
                rp->type = DEV_REPL_FLASH_WRITE;
                rp->len = 0;
            } else {
                hostif_reply_err(rp, "ERROR: gba flash write timed out");
            }
        }
        break;
    case HOST_REQ_FLASH_BANK:
        if (rq->len != 1) {
            hostif_reply_err(rp,
                    "ERROR: gba flash bank: len != 2: %s", itox32(rq->len));
        } else {
            gba_cart_flash_switch_bank(rq->data[0]);
            rp->type = DEV_REPL_FLASH_BANK;
            rp->len = 0;
        }
        break;
    case HOST_REQ_EEPROM512_SEEK:
    case HOST_REQ_EEPROM8K_SEEK:
        if (rq->len != 2) {
            hostif_reply_err(rp,
                    "ERROR: gba eeprom address seek: len != 2: %s", itox32(rq->len));
        } else {
            gba.eeprom_block_addr = (uint16_t)(rq->data[0] | (rq->data[1] << 8));
            gba.eeprom_block_addr >>= 3;
            gba.eeprom_block_addr &= 0x3FF;
            if (rq->type == HOST_REQ_EEPROM512_SEEK)
                rp->type = DEV_REPL_EEPROM512_SEEK;
            else
                rp->type = DEV_REPL_EEPROM8K_SEEK;
            rp->len = 0;
        }
        break;
    case HOST_REQ_EEPROM512_READ:
        {
            gba_cart_eeprom_512_read_data(gba.eeprom_block_addr++, rp->data);
            rp->type = DEV_REPL_EEPROM512_READ;
            rp->len = 8;
        }
        break;
    case HOST_REQ_EEPROM8K_READ:
        {
            gba_cart_eeprom_8k_read_data(gba.eeprom_block_addr++, rp->data);
            rp->type = DEV_REPL_EEPROM8K_READ;
            rp->len = 8;
        }
        break;
    case HOST_REQ_EEPROM512_WRITE:
        if (rq->len != 8) {
            hostif_reply_err(rp,
                    "ERROR: gba eeprom512 write: len != 8: %s", itox32(rq->len));
        } else {
            if (gba_cart_eeprom_512_write_data(gba.eeprom_block_addr++, rq->data)) {
                rp->type = DEV_REPL_EEPROM512_WRITE;
                rp->len = 0;
            } else {
                hostif_reply_err(rp, "ERROR: gba eeprom512 write timed out");
            }
        }
        break;
    case HOST_REQ_EEPROM8K_WRITE:
        if (rq->len != 8) {
            hostif_reply_err(rp,
                    "ERROR: gba eeprom8k write: len != 8: %s", itox32(rq->len));
        } else {
            if (gba_cart_eeprom_8k_write_data(gba.eeprom_block_addr++, rq->data)) {
                rp->type = DEV_REPL_EEPROM8K_WRITE;
                rp->len = 0;
            } else {
                hostif_reply_err(rp, "ERROR: gba eeprom8k write timed out");
            }
        }
        break;
    case HOST_REQ_UART_SYNC:
        rp->type = DEV_REPL_UART_SYNC;
        rp->len = 0;
        break;
    case HOST_REQ_NDS_ROM_SEEK:
        if (rq->len != 4) {
            hostif_reply_err(rp,
                    "ERROR: nds rom seek len != 4: %s", itox32(rq->len));
        } else {
            nds.rom_byte_addr = (size_t)(rq->data[0] | (rq->data[1] << 8) | (rq->data[2] << 16) | (rq->data[3] << 24));
            rp->type = DEV_REPL_NDS_ROM_SEEK;
            rp->len = 0;
        }
        break;
    case HOST_REQ_NDS_ROM_READ:
        if (rq->len != 2) {
            hostif_reply_err(rp,
                    "ERROR: nds rom read len != 2: %s", itox32(rq->len));
        } else {
            uint16_t len = (uint16_t)(rq->data[0] | (rq->data[1] << 8));
            if (len > XFER_MAX_PAYLOAD_SIZE)
                len = XFER_MAX_PAYLOAD_SIZE;
            nds_cart_rom_read(nds.rom_byte_addr, rp->data, len);
            nds.rom_byte_addr += len;
            rp->type = DEV_REPL_NDS_ROM_READ;
            rp->len = len;
        }
        break;
    case HOST_REQ_NDS_ROM_CHIPID:
        nds_cart_cmd_chip_id(rp->data);
        rp->type = DEV_REPL_NDS_ROM_CHIPID;
        rp->len = 4;
        break;
    case HOST_REQ_NDS_ROM_INIT:
        if (nds_cart_rom_init()) {
            rp->type = DEV_REPL_NDS_ROM_INIT;
            rp->len = 0;
        } else {
            hostif_reply_err(rp,
                    "ERROR: nds rom could not be initialized, see UART for details");
        }
        break;
    default:
        hostif_reply_err(rp,
                "ERROR: invalid request ID: %s", itox32(rq->type));
    }
    hostif_send_reply();
}

static void hostif_send_reply(void)
{
    usb_send_data(reply, (uint16_t)(sizeof(struct device_reply) + reply->len));
    //uart_printf("start reply\n");
    //uart_printf("magic: %s\n", itox32(reply->magic));
    //uart_printf("type: %s\n", itox32(reply->type));
    //uart_printf("id: %s\n", itox32(reply->id));
    //uart_printf("len: %s\n", itox32(reply->len));
}

static void hostif_reply_err(struct device_reply *const rp, const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    rp->type = DEV_REPL_ERR;
    rp->len = (uint16_t)vsnprintf((char *)rp->data, XFER_MAX_PAYLOAD_SIZE, msg, args);
    va_end(args);
}

void hostif_data_receive(const uint8_t *data, uint16_t size)
{
    static uint8_t request_receive_buffer[REQ_BUF_SIZE];
    static struct host_request *const request_receive = (struct host_request *)request_receive_buffer;
    static uint16_t request_receive_pos = 0;
    static bool buffer_overflow = false;

    if (size == 0)
        return;

    if (buffer_overflow) {
        // if the buffer get's overrun to heavily, flush it to be able to resync
        request_receive_pos = 0;
        buffer_overflow = false;
    }

    if (request_receive_pos >= REQ_BUF_SIZE) {
        buffer_overflow = true;
        return;
    }

    if ((size_t)size + request_receive_pos > REQ_BUF_SIZE)
        size = (uint16_t)(REQ_BUF_SIZE - request_receive_pos);

    memcpy(request_receive_buffer + request_receive_pos, data, size);
    request_receive_pos = (uint16_t)(request_receive_pos + size);

    // okay received chunk, check if header is valid
    if (request_receive_pos < offsetof(struct device_reply, magic) + sizeof(uint16_t))
        return;

    //usb_printf("magic=%s", itox32(request_receive->magic));

    while (request_receive->magic != HOST_REQ_MAGIC) {
        // illegal magic
        // TODO this could be made possibly more efficient
        // this shouldn't only be relevant however during the intial sync phase
        memmove(&request_receive_buffer[0], &request_receive_buffer[1], (size_t)(request_receive_pos - 1));
        request_receive_pos--;
        if (request_receive_pos <= 1)
            return;
    }

    if (request_receive_pos < offsetof(struct device_reply, len) + sizeof(uint16_t))
        return;

    uint16_t packet_len = request_receive->len;
    if (request_receive_pos < packet_len + sizeof(struct host_request))
        return;

    // enough data, okay, transfer data on next occasion
    if (request_a_available == false) {
        // free slot to read data to
        size_t bytes_to_copy = sizeof(struct host_request) + packet_len;
        memcpy(request_a, request_receive_buffer, bytes_to_copy);
        memmove(request_receive_buffer,
                request_receive_buffer + bytes_to_copy,
                request_receive_pos - bytes_to_copy);
        request_receive_pos = (uint16_t)(request_receive_pos - bytes_to_copy);
        request_a_available = true;
        return;
    }
    if (request_b_available == false) {
        // free slot to read data to
        size_t bytes_to_copy = sizeof(struct host_request) + packet_len;
        memcpy(request_b, request_receive_buffer, bytes_to_copy);
        memmove(request_receive_buffer,
                request_receive_buffer + bytes_to_copy,
                request_receive_pos - bytes_to_copy);
        request_receive_pos = (uint16_t)(request_receive_pos - bytes_to_copy);
        request_b_available = true;
        return;
    }
}
