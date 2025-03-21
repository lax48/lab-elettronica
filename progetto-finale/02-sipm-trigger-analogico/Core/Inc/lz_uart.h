#pragma once

#include <stdint.h>

uint8_t __lz_serial_new_character_read(char c); // Implemented in main.c

void lz_uart_setup(void);
int lz_uart_interrupt(void);

void lz_uart_tx_disable_interrupt(void);
void lz_uart_tx_enable_interrupt(void);
