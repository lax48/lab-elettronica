#pragma once

#include <stdint.h>
#include <stddef.h>

#include "lz_queue.h"

// Default queues
LZ_DECLARE_QUEUE(__lz_serial_tx_buffer, 8);
LZ_DECLARE_QUEUE(__lz_serial_rx_buffer, 256);

// Events (implemented in main.c)
void __lz_serial_error_tx_buffer_full(void);
void __lz_serial_error_rx_buffer_full(void);

// Print
void lz_serial_print(char* str);
void lz_serial_println(char* str);
void lz_serial_send_binary(void* data, size_t size);
void lz_serial_flush(void);

// Read
uint16_t lz_serial_available();
size_t lz_serial_read(void* data, size_t size);
size_t lz_serial_read_avl(void* data, size_t size);
size_t lz_serial_readln(char* data, size_t size);

