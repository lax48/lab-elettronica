/*
 * lz_uart.c
 *
 *  Created on: Oct 18, 2024
 *      Author: root
 */

#include "lz_uart.h"
#include "lz_serial.h"
#include "lz_queue.h"

#include "stm32h743xx.h"

void lz_uart_setup(void) {
	USART3->CR1 |= USART_CR1_TE | USART_CR1_TCIE;	// Enable TX
	USART3->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;	// Enable RX
	USART3->CR1 |= USART_CR1_UE;					// Enable USART
}

int lz_uart_interrupt(void) {
	uint8_t handled = 0;
	if (USART3->ISR & (USART_ISR_TXE_TXFNF)) {
		uint8_t c;
		if (LZ_QUEUE_POP(__lz_serial_tx_buffer, &c))
			USART3->TDR = c;
		else
			USART3->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;

		handled = 1;
	}

	if (USART3->ISR & USART_ISR_TC) {
		USART3->ICR |= USART_ICR_TCCF;
		handled = 1;
	}

	if (USART3->ISR & USART_ISR_RXNE_RXFNE) {
		uint8_t c = USART3->RDR & 0xFF;

		if (!__lz_serial_new_character_read(c) && !LZ_QUEUE_PUSH(__lz_serial_rx_buffer, c))
			__lz_serial_error_rx_buffer_full();

		handled = 1;
	}

	return handled;
}

void __lz_serial_start_tx_transmission(void) {
	USART3->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}
