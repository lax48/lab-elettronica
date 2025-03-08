#pragma once

#include <stdint.h>
#include <assert.h>

typedef struct
{
	uint8_t* buffer;
	uint16_t size;
	volatile uint16_t start, end;
} lz_queue_t;

extern void __lz_queue_error(void);

uint8_t lz_queue_push(lz_queue_t* queue, uint8_t c);
uint8_t lz_queue_pop(lz_queue_t* queue, uint8_t* c);

uint16_t lz_queue_count(lz_queue_t* queue);
uint8_t lz_queue_is_empty(lz_queue_t* queue);
uint8_t lz_queue_is_full(lz_queue_t* queue);


#define LZ_DEFINE_QUEUE(queue_name, buffer_size) \
	static_assert((buffer_size) >= 2, "Queue must contain at least 2 elements"); \
	static_assert(((buffer_size) & ((buffer_size) - 1)) == 0, "Queue size must be a power of 2"); \
	uint8_t queue_name##_data[(buffer_size)];  \
    lz_queue_t queue_name = { queue_name##_data, (buffer_size), 0, 0 }
