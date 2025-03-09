#pragma once

#include <stdint.h>
#include <assert.h>

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
