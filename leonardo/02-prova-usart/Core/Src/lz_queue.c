#include "lz_queue.h"


uint8_t lz_queue_push(lz_queue_t* queue, uint8_t c)
{
	if (lz_queue_is_full(queue))
		return 0;

	queue->buffer[queue->end] = c;

	// Assume queue->size is a power of 2 (enforced by LZ_DEFINE_QUEUE)
	queue->end = (queue->end + 1) & (queue->size - 1);

	return 1;
}

uint8_t lz_queue_pop(lz_queue_t* queue, uint8_t* c)
{
	if (lz_queue_is_empty(queue))
		return 0;

	*c = queue->buffer[queue->start];

	// Assume queue->size is a power of 2 (enforced by LZ_DEFINE_QUEUE)
	queue->start = (queue->start + 1) & (queue->size - 1);

	return 1;
}

uint16_t lz_queue_count(lz_queue_t* queue)
{
	// Assume queue->size is a power of 2 (enforced by LZ_DEFINE_QUEUE)
	return (queue->end - queue->start) & (queue->size - 1);
}

uint8_t lz_queue_is_empty(lz_queue_t* queue)
{
	return queue->start == queue->end;
}

uint8_t lz_queue_is_full(lz_queue_t* queue)
{
	// Assume queue->size is a power of 2 (enforced by LZ_DEFINE_QUEUE)
	// Queue is full when queue->size - 1 elements are in use.
	// We can't define "full" when queue->size are in use because we
	// would have queue->start == queue->end, which is indistinguishable
	// from empty queue.

	return ((queue->end + 1) & (queue->size - 1)) == queue->start;
}
