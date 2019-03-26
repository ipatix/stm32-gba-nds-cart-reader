#pragma once

#include <stdint.h>

__attribute__((always_inline)) static inline void NOP(void)
{
    __asm__("\tnop");
}

__attribute__((always_inline)) static inline void WAIT(int x)
{
    __asm__ volatile(
            "delay_loop_%=:\n\t"
            "subs %[cnt], #1\n\t"
            "bgt delay_loop_%=\n\t"
            : [cnt] "+r" (x) :: "cc"
           );
}

void usb_send_data(const void *data, uint16_t len);
void usb_print_bytes(const void *data, uint16_t len);
void usb_printf(const char *msg, ...);

void uart_send_data(const void *data, uint16_t len);
void uart_print_bytes(const void *data, uint16_t len);
void uart_printf(const char *msg, ...);

const char *itox8(uint8_t x);
const char *itox32(uint32_t x);

#define WALGN __attribute__((aligned(4)))
