/*
 * WiFi CNC Controller - Motion Segment Planner Ring Buffer
 *
 * Lock-free single-producer / single-consumer ring buffer using
 * volatile head/tail indices. The network task (Core 0) is the sole
 * producer; the motion control task (Core 1) is the sole consumer.
 *
 * For the clear operation (E-Stop), a spinlock is used since both
 * cores may be active.
 */

#include "planner.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_attr.h"

/* Compile-time check: buffer size must be power of 2 */
_Static_assert((CFG_PLANNER_BUFFER_SIZE & (CFG_PLANNER_BUFFER_SIZE - 1)) == 0,
               "CFG_PLANNER_BUFFER_SIZE must be a power of 2");

typedef struct {
    wcnc_motion_segment_t segments[CFG_PLANNER_BUFFER_SIZE];
    volatile uint16_t head;     /* Write index (producer: network task) */
    volatile uint16_t tail;     /* Read index (consumer: motion task) */
    portMUX_TYPE mux;           /* Spinlock for clear operation */
} planner_buffer_t;

static DRAM_ATTR planner_buffer_t pb;

#define MASK (CFG_PLANNER_BUFFER_SIZE - 1)

void planner_init(void)
{
    memset(&pb, 0, sizeof(pb));
    pb.head = 0;
    pb.tail = 0;
    portMUX_INITIALIZE(&pb.mux);
}

bool IRAM_ATTR planner_push_segment(const wcnc_motion_segment_t *seg)
{
    uint16_t next_head = (pb.head + 1) & MASK;

    /* Check if buffer is full */
    if (next_head == pb.tail) {
        return false;
    }

    memcpy(&pb.segments[pb.head], seg, sizeof(wcnc_motion_segment_t));

    /* Memory barrier: ensure segment data is written before head advances */
    __sync_synchronize();

    pb.head = next_head;
    return true;
}

bool IRAM_ATTR planner_pop_segment(wcnc_motion_segment_t *seg)
{
    /* Check if buffer is empty */
    if (pb.tail == pb.head) {
        return false;
    }

    memcpy(seg, &pb.segments[pb.tail], sizeof(wcnc_motion_segment_t));

    /* Memory barrier: ensure segment data is read before tail advances */
    __sync_synchronize();

    pb.tail = (pb.tail + 1) & MASK;
    return true;
}

uint16_t planner_available(void)
{
    uint16_t count = (pb.head - pb.tail) & MASK;
    return (CFG_PLANNER_BUFFER_SIZE - 1) - count;
}

uint16_t planner_count(void)
{
    return (pb.head - pb.tail) & MASK;
}

void planner_clear(void)
{
    portENTER_CRITICAL(&pb.mux);
    pb.head = 0;
    pb.tail = 0;
    portEXIT_CRITICAL(&pb.mux);
}

bool IRAM_ATTR planner_is_empty(void)
{
    return (pb.tail == pb.head);
}

bool IRAM_ATTR planner_is_full(void)
{
    return (((pb.head + 1) & MASK) == pb.tail);
}

uint16_t planner_capacity(void)
{
    return CFG_PLANNER_BUFFER_SIZE - 1;
}
