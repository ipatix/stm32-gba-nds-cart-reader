#include <stdarg.h>
#include <stdio.h>

#include "usbd_cdc_if.h"
#include "usart.h"
#include "util.h"

void usb_send_data(const void *data, uint16_t len)
{
    while (len > 0) {
        uint16_t block_len = len;
        if (block_len > sizeof(UserTxBufferFS))
            block_len = sizeof(UserTxBufferFS);

        extern USBD_HandleTypeDef hUsbDeviceFS;
        USBD_CDC_HandleTypeDef *hcdc = hUsbDeviceFS.pClassData;

        while (hcdc->TxState != 0);

        memcpy(UserTxBufferFS, data, block_len);
        int result = CDC_Transmit_FS(UserTxBufferFS, block_len);

        data = (const char *)data + block_len;
        len = (uint16_t)(len - block_len);

        (void)result;
    }
}

void usb_print_bytes(const void *data, uint16_t len)
{
    if (len > 64)
        len = 64;
    char out[256];
    out[0] = '\0';
    const char *input = data;
    while (len-- > 0) {
        strcat(out, itox8((uint8_t)*input++));
        strcat(out, ",");
    }
    usb_printf("%s\n", out);
}

void usb_printf(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);

    char out[256];
    int len = vsnprintf(out, sizeof(out), msg, args);
    usb_send_data(out, (uint16_t)len);

    va_end(args);
}

void uart_send_data(const void *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (void *)data, len, HAL_MAX_DELAY);
}

void uart_print_bytes(const void *data, uint16_t len)
{
    if (len > 64)
        len = 64;
    char out[256];
    out[0] = '\0';
    const char *input = data;
    while (len-- > 0) {
        strcat(out, itox8((uint8_t)*input++));
        strcat(out, ",");
    }
    uart_printf("%s\n", out);
}

void uart_printf(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);

    char out[256];
    int len = vsnprintf(out, sizeof(out), msg, args);
    uart_send_data(out, (uint16_t)len);

    va_end(args);
}

#define INT_TO_DISP_CHAR(i) (((i) <= 9) ? (char)('0' + (i)) : (char)('A' + (i) - 10))

#define NUM_ITOX8 16
const char *itox8(uint8_t x)
{
    static char strs[NUM_ITOX8][3];
    static uint8_t str_index = 0;
    char *str = strs[str_index];

    str[0] = INT_TO_DISP_CHAR(x >> 4);
    str[1] = INT_TO_DISP_CHAR(x & 0xF);
    str[2] = '\0';
    str_index = (uint8_t)((str_index + 1) % NUM_ITOX8);
    return str;
}

#define NUM_ITOX32 4
const char *itox32(uint32_t x)
{
    static char strs[NUM_ITOX32][9];
    static uint8_t str_index = 0;
    char *str = strs[str_index];

    for (uint32_t i = 0; i < 8; i++) {
        uint32_t nibble = x >> 28;
        str[i] = INT_TO_DISP_CHAR(nibble);
        x <<= 4;
    }
    str[8] = '\0';
    str_index = (uint8_t)((str_index + 1) % NUM_ITOX32);
    return str;
}

uint64_t bitrev_64(uint64_t x)
{
    uart_printf("bitrev: x=%llx\r\n", x);
    uint64_t result = 0;
    for (size_t i = 0; i < 64; i++) {
        result <<= 1;
        result |= x & 1;
        x >>= 1;
    }
    uint32_t sp;
    __asm__("mov %[sp], sp" : [sp] "=r"  (sp));
    uart_printf("bitrev: result=%llx\r\n", result);
    uart_printf("sp=0x%08x\r\n", sp);
    return result;
}

void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
    uart_printf("ASSERTION FAILED:\n  EXPR: %s\n  FILE: %s\n  FUNC: %s\n  LINE: %d", failedexpr, file, func, line);
    while (1) {
        __asm__ volatile("nop");
    }
}
