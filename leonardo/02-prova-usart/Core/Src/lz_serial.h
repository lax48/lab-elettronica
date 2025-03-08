#pragma once

#include <stdint.h>
#include <stddef.h>
#include "lz_queue.h"

// Events (implemented in main.c)
extern void __lz_serial_error_tx_buffer_full(void);
extern void __lz_serial_error_rx_buffer_full(void);

// Low level (implemented in stm32h7xx_it.c)
void __lz_serial_transmit_char(uint8_t c); // implemented in main.c
uint8_t __lz_serial_receive_char(void);

// Buffers
extern lz_queue_t __lz_serial_tx_buffer;
extern lz_queue_t __lz_serial_rx_buffer;
void __lz_serial_end_tx_transmission();

// Print
void lz_serial_print(char* str);
void lz_serial_println(char* str);
void lz_serial_send_binary(void* data, size_t size);

// Read
uint16_t lz_serial_available();
size_t lz_serial_read(void* data, size_t size);
size_t lz_serial_read_avl(void* data, size_t size);
size_t lz_serial_readln(char* data, size_t size);

