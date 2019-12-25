#ifndef __QUEUE_H
#define __QUEUE_H

typedef struct {
	void* p_data;
	struct {
		uint32_t queue;
		uint32_t element;
	}size;
	uint32_t front;
	uint32_t rear;
}queue_t;

void queue_init(queue_t* q, void* p_data, uint32_t size_q, uint32_t size_e);
int32_t queue_push(queue_t* q, void* e);
int32_t queue_pop(queue_t* q, void* e);

#endif