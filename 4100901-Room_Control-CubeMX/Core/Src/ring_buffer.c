#include "ring_buffer.h"
#include <stdbool.h>
#include <stdint.h>

void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity) 
{
    rb->buffer = buffer;
    rb->capacity = capacity;
    rb->head = 0;
    rb->tail = 0;
    rb->full = false;
}

bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    return (!rb-> full & (rb->head == rb->tail));
}

bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return rb->full;
}

bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    if (ring_buffer_is_full(rb)){
        return (rb->tail+1)% rb->capacity; // No hay espacio disponible
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->capacity;
    rb->full = (rb->head == rb->tail);
    return true;
}

bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    if (ring_buffer_is_empty(rb))
        return false; // No hay datos disponibles

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->full = false;
    return true;
}

uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    if (rb->full) 
        return rb->capacity;
    
    // Diferencia circular entre head y tail
    if (rb->head >= rb->tail)
        return (rb->head - rb->tail);
    else
        return (rb->capacity - (rb->tail - rb->head));
}

void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->full = false;
}
