//#include "ringbuffer.h"
//
//#include <stdlib.h>
//
//typedef struct ringbuffer {
//    uint16_t num_elements;
//    uint16_t element_size;
//    volatile uint16_t free_index;
//    volatile uint16_t free_count;
//    volatile uint16_t used_index;
//    volatile uint16_t used_count;
//    uint8_t element_data[];
//} ringbuffer;
//
//bool rb_init(uint8_t *mem, uint16_t num_elements, uint16_t element_size)
//{
//    ringbuffer *rb = (ringbuffer *)mem;
//    rb->num_elements = num_elements;
//    rb->element_size = element_size;
//}
//
//void *rb_get(ringbuffer *rb)
//{
//    uint16_t used_count = rb->used_count;
//    if (used_count == 0)
//        return NULL;
//    void *retval = &rb->element_data[rb->element_size * rb->used_count];
//    uint16_t used_index = rb->used_index;
//    if (++used_index >= num_elements)
//        used_index = 0;
//    rb->used_index = used_index;
//    rb->used_count = used_count - 1;
//    rb->free_count++;
//    return retval;
//}
//
//bool rb_put(ringbuffer *rb, void *element, uint16_t size)
//{
//    uint16_t free_count = rb->free_count;
//    if (free_count == 0)
//        return NULL;
//    
//}
