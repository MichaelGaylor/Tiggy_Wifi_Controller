/*
 * WiFi CNC Controller - Queue of segments in the speed-based form
 *
 * Segments described by entry speed, exit speed and acceleration: what
 * G-code mode produces, and what hosts older than protocol 1.2 send.  The
 * motion task takes them from here and turns each into timed slices for
 * the step engine (motion_core.c).
 *
 * Lock-free single-producer / single-consumer ring buffer using volatile
 * head/tail indices.  The producer is the G-code task (or the motion task
 * itself for an old host); the motion task is the only consumer.
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
    volatile uint16_t head;     /* Write index (producer) */
    volatile uint16_t tail;     /* Read index (consumer: motion task) */
    volatile bool clear_req;    /* planner_clear() was called: the consumer drops everything up to clear_to */
    volatile uint16_t clear_to;
} planner_buffer_t;

static DRAM_ATTR planner_buffer_t pb;

#define MASK (CFG_PLANNER_BUFFER_SIZE - 1)

void planner_init(void)
{
    memset(&pb, 0, sizeof(pb));
    pb.head = 0;
    pb.tail = 0;
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
    if (pb.clear_req) {
        pb.tail = pb.clear_to;
        pb.clear_req = false;
    }

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

uint16_t planner_count(void)
{
    uint16_t from = pb.clear_req ? pb.clear_to : pb.tail;
    return (pb.head - from) & MASK;
}

uint16_t planner_available(void)
{
    return (CFG_PLANNER_BUFFER_SIZE - 1) - planner_count();
}

/* May be called from any task.  Only the consumer moves the read index, so
 * the request is recorded here and carried out at the next pop; segments
 * pushed after this call are kept. */
void planner_clear(void)
{
    pb.clear_to = pb.head;
    pb.clear_req = true;
}

/* Consumer side: drop everything queued, now. */
void planner_drain(void)
{
    pb.clear_req = false;
    pb.tail = pb.head;
}

bool IRAM_ATTR planner_is_empty(void)
{
    return planner_count() == 0;
}


bool IRAM_ATTR planner_is_full(void)
{
    return (((pb.head + 1) & MASK) == pb.tail);
}

uint16_t planner_capacity(void)
{
    return CFG_PLANNER_BUFFER_SIZE - 1;
}
