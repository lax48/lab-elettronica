#include "lz_serial.h"

#include <assert.h>
#include "lz_queue.h"

#define SERIAL_NONE 0
#define SERIAL_CR 1
#define SERIAL_LF 2
#define SERIAL_CRLF 4

#ifndef SERIAL_LINE_TERMINATOR_STRATEGY
#	define SERIAL_LINE_TERMINATOR_STRATEGY SERIAL_LF
#endif

LZ_DEFINE_QUEUE(__lz_serial_tx_buffer);
LZ_DEFINE_QUEUE(__lz_serial_rx_buffer);

// Private serial implementation
void __lz_serial_start_tx_transmission(void);
void __lz_serial_transmit_char(uint8_t c);
uint8_t __lz_serial_receive_char(void);

static void __lz_serial_write_char(uint8_t c)
{
	while (!LZ_QUEUE_PUSH(__lz_serial_tx_buffer, c))
	{
		// TX buffer is full.

		// Possible causes:
		//	- TX buffer is too small
		//  - USART baudrate too low
		//  - USART interrupt is not set up right

		__lz_serial_error_tx_buffer_full();
	}

	__lz_serial_start_tx_transmission(); // this can be called even if a transmission has already been started.
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
		__lz_serial_write_char(((uint8_t*)data)[i]);
}

void lz_serial_flush(void)
{
	__lz_serial_start_tx_transmission();
	while (!LZ_QUEUE_IS_EMPTY(__lz_serial_tx_buffer));
}

uint16_t lz_serial_available()
{
	return LZ_QUEUE_COUNT(__lz_serial_rx_buffer);
}

size_t lz_serial_read(void* data, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		while (!LZ_QUEUE_POP(__lz_serial_rx_buffer, (uint8_t*)data + i))
		{}
	}

	return size;
}

size_t lz_serial_read_avl(void* data, size_t size)
{
	size_t i;
	for (i = 0; i < size; i++)
	{
		if (!LZ_QUEUE_POP(__lz_serial_rx_buffer, (uint8_t*)data + i))
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

		while (!LZ_QUEUE_POP(__lz_serial_rx_buffer, (uint8_t*)data + i))
		//while (!lz_queue_pop(&__lz_serial_rx_buffer, (uint8_t*)data + i))
		{}

#if SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_LF || SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CRLF
		if (data[i] == '\n')
#elif SERIAL_LINE_TERMINATOR_STRATEGY == SERIAL_CR
		if (data[i] == '\r')
#else
		static_assert(0 && "lz_serial_readln does not support current SERIAL_LINE_TERMINATOR_STRATEGY");
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
