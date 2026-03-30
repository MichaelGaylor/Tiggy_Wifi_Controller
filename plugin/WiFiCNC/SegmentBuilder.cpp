/*
 * WiFi CNC Controller - Segment Builder Implementation
 *
 * Aggregates Mach3 TrajPoints into motion segments for the ESP32.
 * Each TrajPoint represents one kernel period (1/KernelFreq seconds).
 */

#include "SegmentBuilder.h"
#include "BufferManager.h"
#include <cstring>
#include <cmath>
#include <cstdlib>

SegmentBuilder::SegmentBuilder()
    : m_probeMode(false)
    , m_pendingBufMgr(nullptr)
    , m_segmentId(0)
{
    memset(&m_accum, 0, sizeof(m_accum));
    /* Default acceleration per axis (overridden by SetAxisAccel) */
    for (int i = 0; i < MAX_AXES; i++) {
        m_axisAccel[i] = 5000;  /* 5000 steps/sec^2 default */
    }
}

void SegmentBuilder::SetAxisAccel(int axis, uint32_t accel_steps_per_sec2)
{
    if (axis >= 0 && axis < MAX_AXES) {
        m_axisAccel[axis] = accel_steps_per_sec2;
    }
}

bool SegmentBuilder::ShouldFlush(const TrajPoint *tp)
{
    /* Flush if accumulator is full */
    if (m_accum.point_count >= MAX_ACCUM_POINTS) return true;

    /* Flush if direction reverses on any axis */
    for (int i = 0; i < MAX_AXES; i++) {
        int8_t newDir = 0;
        if (tp->Points[i] > 0) newDir = 1;
        else if (tp->Points[i] < 0) newDir = -1;

        if (newDir != 0 && m_accum.prev_dir[i] != 0 &&
            newDir != m_accum.prev_dir[i]) {
            return true;
        }
    }

    return false;
}

void SegmentBuilder::AddTrajPoint(const TrajPoint *tp, double kernelFreq)
{
    /* Compute duration of one TrajPoint in microseconds */
    uint32_t point_duration_us = (uint32_t)(1000000.0 / kernelFreq);

    /* Check if we should flush before adding this point */
    if (m_accum.has_motion && ShouldFlush(tp)) {
        if (m_pendingBufMgr) {
            EmitSegment(m_pendingBufMgr);
        }
    }

    /* Accumulate step counts */
    for (int i = 0; i < MAX_AXES; i++) {
        m_accum.steps[i] += tp->Points[i];

        /* Track direction for reversal detection */
        if (tp->Points[i] > 0) m_accum.prev_dir[i] = 1;
        else if (tp->Points[i] < 0) m_accum.prev_dir[i] = -1;
    }

    m_accum.duration_us += point_duration_us;
    m_accum.point_count++;

    /* Check if any axis has non-zero steps */
    for (int i = 0; i < MAX_AXES; i++) {
        if (tp->Points[i] != 0) {
            m_accum.has_motion = true;
            break;
        }
    }
}

void SegmentBuilder::FlushSegments(BufferManager *bufMgr)
{
    m_pendingBufMgr = bufMgr;

    /* If we have accumulated data, emit it */
    if (m_accum.point_count > 0 && m_accum.has_motion) {
        EmitSegment(bufMgr);
    }
    /* Also flush if we have zero-motion accumulated (dwell-like) */
    else if (m_accum.point_count > 0 && m_accum.duration_us > 0) {
        EmitSegment(bufMgr);
    }
}

void SegmentBuilder::EmitSegment(BufferManager *bufMgr)
{
    if (m_accum.point_count == 0) return;

    wcnc_motion_segment_t seg;
    memset(&seg, 0, sizeof(seg));

    /* Copy step counts */
    for (int i = 0; i < MAX_AXES && i < WCNC_MAX_AXES; i++) {
        seg.steps[i] = m_accum.steps[i];
    }

    seg.duration_us = m_accum.duration_us;

    /* Compute speeds from step counts and duration */
    uint32_t max_steps = 0;
    for (int i = 0; i < MAX_AXES; i++) {
        uint32_t abs_steps = (uint32_t)(m_accum.steps[i] < 0 ? -(int64_t)m_accum.steps[i] : m_accum.steps[i]);
        if (abs_steps > max_steps) max_steps = abs_steps;
    }

    if (max_steps > 0 && m_accum.duration_us > 0) {
        float speed = (float)max_steps /
                      ((float)m_accum.duration_us / 1000000.0f);
        uint32_t speed_sqr = (uint32_t)(speed * speed * 1000.0f);

        seg.entry_speed_sqr = speed_sqr;
        seg.exit_speed_sqr = speed_sqr;

        /* Use dominant axis acceleration (axis with most steps) */
        int dominant = 0;
        uint32_t maxAbs = 0;
        for (int i = 0; i < MAX_AXES; i++) {
            uint32_t a = (uint32_t)(m_accum.steps[i] < 0 ? -(int64_t)m_accum.steps[i] : m_accum.steps[i]);
            if (a > maxAbs) { maxAbs = a; dominant = i; }
        }
        seg.acceleration = m_axisAccel[dominant] * 100;  /* Protocol: steps/sec^2 * 100 */
    }

    seg.segment_id = m_segmentId++;

    /* Apply flags */
    seg.flags = 0;
    if (m_probeMode) {
        seg.flags |= WCNC_SEG_FLAG_PROBE;
        m_probeMode = false; /* Auto-reset after one probe segment */
    }

    /* Queue segment */
    bufMgr->QueueSegment(&seg);

    /* Reset accumulator */
    memset(&m_accum, 0, sizeof(m_accum));
}

void SegmentBuilder::SetProbeMode(bool enable)
{
    m_probeMode = enable;
}

void SegmentBuilder::InsertDwell(double seconds)
{
    /* Emit any pending motion first */
    if (m_pendingBufMgr && m_accum.has_motion) {
        EmitSegment(m_pendingBufMgr);
    }

    /* Create a zero-motion segment with the dwell duration */
    wcnc_motion_segment_t seg;
    memset(&seg, 0, sizeof(seg));
    seg.duration_us = (uint32_t)(seconds * 1000000.0);
    seg.segment_id = m_segmentId++;
    seg.flags = WCNC_SEG_FLAG_EXACT_STOP;

    if (m_pendingBufMgr) {
        m_pendingBufMgr->QueueSegment(&seg);
    }
}
