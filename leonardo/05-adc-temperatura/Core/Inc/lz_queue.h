#pragma once

#include <stdint.h>
#include <assert.h>

/*
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

*/

#define LZ_DECLARE_QUEUE(queue_name, buffer_size)													\
	static_assert((buffer_size) >= 2, "Queue must contain at least 2 elements");					\
	static_assert(((buffer_size) & ((buffer_size) - 1)) == 0, "Queue size must be a power of 2");	\
	extern uint8_t queue_name##_data[(buffer_size)];												\
	extern volatile uint16_t queue_name##_start, queue_name##_end;

#define LZ_DEFINE_QUEUE(queue_name)																	\
	uint8_t queue_name##_data[];																	\
	volatile uint16_t queue_name##_start = 0, queue_name##_end = 0;

#define LZ_QUEUE_SIZE(queue) (sizeof(queue##_data) / sizeof(queue##_data[0]))

#define LZ_QUEUE_PUSH(queue, c)																		\
	(																								\
		(LZ_QUEUE_IS_FULL(queue)) ? 0 :																\
		(																							\
			(queue##_data[queue##_end] = (c)),														\
			(queue##_end = (queue##_end + 1) & (LZ_QUEUE_SIZE(queue) - 1)),							\
			1																						\
		)																							\
	)

#define LZ_QUEUE_POP(queue, cptr)																	\
	(																								\
		(LZ_QUEUE_IS_EMPTY(queue)) ? 0 :															\
		(																							\
			(*(cptr) = (queue##_data)[queue##_start]),												\
			(queue##_start = (queue##_start + 1) & (LZ_QUEUE_SIZE(queue) - 1)), 					\
			1 																						\
		)																							\
	)

#define LZ_QUEUE_COUNT(queue) ((queue##_end - queue##_start)& (LZ_QUEUE_SIZE(queue) - 1));
#define LZ_QUEUE_IS_EMPTY(queue) (queue##_start == queue##_end)
#define LZ_QUEUE_IS_FULL(queue) (((queue##_end + 1) & (LZ_QUEUE_SIZE(queue) - 1)) == queue##_start)
