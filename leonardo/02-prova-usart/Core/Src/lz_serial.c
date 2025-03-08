#include "lz_serial.h"
#include <assert.h>

#define SERIAL_NONE 0
#define SERIAL_CR 1
#define SERIAL_LF 2
#define SERIAL_CRLF 4

#ifndef SERIAL_LINE_TERMINATOR_STRATEGY
#	define SERIAL_LINE_TERMINATOR_STRATEGY SERIAL_LF
#endif

volatile static uint8_t is_tx_transmitting = 0;

LZ_DEFINE_QUEUE(__lz_serial_tx_buffer, 8);
LZ_DEFINE_QUEUE(__lz_serial_rx_buffer, 256);


// Buffers
void __lz_serial_end_tx_transmission()
{
	is_tx_transmitting = 0;
}

static void __lz_serial_write_char(uint8_t c)
{
	while (!lz_queue_push(&__lz_serial_tx_buffer, c))
	{
		// TX buffer is full.

		// Possible causes:
		//	- TX buffer is too small
		//  - USART baudrate too low
		//  - USART interrupt is not set up right

		__lz_serial_error_tx_buffer_full();
	}

	if (!is_tx_transmitting)
	{
		// USART TX interrupt wont run here (is_tx_transmitting
		// is set to zero when previous transmission is finished)

		uint8_t success = lz_queue_pop(&__lz_serial_tx_buffer, &c);
		assert(success && lz_queue_is_empty(&__lz_serial_tx_buffer));

		is_tx_transmitting = 1;
		__lz_serial_transmit_char(c);
	}
}

void lz_serial_print(char* str)
{
	while (*str)
		__lz_serial_write_char(*(str++));
}

void lz_serial_println(char* str)
{
	lz_serial_print(str);
#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CR || SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
	__lz_serial_write_char('\r');
#endif
#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_LF || SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
	__lz_serial_write_char('\n');
#endif
}

void lz_serial_send_binary(void* data, size_t size)
{
	for (size_t i = 0; i < size; i++)
		__lz_serial_write_char(((char*)data)[i]);
}

uint16_t lz_serial_available()
{
	return lz_queue_count(&__lz_serial_rx_buffer);
}

size_t lz_serial_read(void* data, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		while (!lz_queue_pop(&__lz_serial_rx_buffer, (uint8_t*)data + i))
		{}
	}

	return size;
}

size_t lz_serial_read_avl(void* data, size_t size)
{
	size_t i;
	for (i = 0; i < size; i++)
	{
		if (!lz_queue_pop(&__lz_serial_rx_buffer, (uint8_t*)data + i))
			break;
	}

	return i;
}

size_t lz_serial_readln(char* data, size_t size)
{
#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
	uint8_t seenCarriageReturn = 0;
#endif
	size_t i;
	for (i = 0; i < size - 1; i++)
	{
		while (!lz_queue_pop(&__lz_serial_rx_buffer, (uint8_t*)data + i))
		{}

#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_LF || SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
		if (data[i] == '\n')
#elif SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CR
		if (data[i] == '\r')
#else
		assert(0 && "lz_serial_readln does not support current SERIAL_LINE_TERMINATOR_STRATEGY");
#endif
		{
#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
			i -= seenCarriageReturn ? 1 : 0;
#endif
			break;
		}

#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
		seenCarriageReturn = data[i] == '\r';
#endif
	}

	data[i] = '\0';
	return i;
}
