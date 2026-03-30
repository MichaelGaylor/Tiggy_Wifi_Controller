/*
 * WiFi CNC Controller - Segment Builder
 *
 * Converts Mach3 TrajPoints (200us each) into aggregated motion segments
 * suitable for network transmission to the ESP32.
 *
 * Aggregation strategy:
 *   - Accumulate up to 50 TrajPoints into one segment (10ms)
 *   - Flush on direction reversal of any axis
 *   - Flush on reaching the accumulation limit
 */

#ifndef SEGMENTBUILDER_H
#define SEGMENTBUILDER_H

#include "MachIncludes/Engine.h"
#include "../../protocol/wifi_cnc_protocol.h"
#include <cstdint>

#ifndef MAX_AXES
#define MAX_AXES 6
#endif

class BufferManager;  /* Forward declaration */

class SegmentBuilder {
public:
    SegmentBuilder();

    /* Set per-axis acceleration (call after Engine is available) */
    void SetAxisAccel(int axis, uint32_t accel_steps_per_sec2);

    /* Add a single Mach3 TrajPoint */
    void AddTrajPoint(const TrajPoint *tp, double kernelFreq);

    /* Flush any accumulated data as segments to the buffer manager */
    void FlushSegments(BufferManager *bufMgr);

    /* Set probe mode (next motion segments will have probe flag) */
    void SetProbeMode(bool enable);

    /* Check if probe mode is active */
    bool IsProbeMode() const { return m_probeMode; }

    /* Insert a dwell segment (zero motion, timed pause) */
    void InsertDwell(double seconds);

private:
    struct Accumulator {
        int32_t  steps[MAX_AXES];       /* Accumulated steps */
        uint32_t duration_us;           /* Accumulated duration */
        uint32_t point_count;           /* TrajPoints consumed */
        bool     has_motion;            /* Any non-zero steps */
        int8_t   prev_dir[MAX_AXES];   /* Direction of previous point */
    };

    Accumulator m_accum;
    uint32_t m_axisAccel[MAX_AXES];  /* Per-axis accel in steps/sec^2 */
    bool m_probeMode;
    BufferManager *m_pendingBufMgr;     /* Set during FlushSegments */
    uint16_t m_segmentId;

    static const uint32_t MAX_ACCUM_POINTS = 50;

    void EmitSegment(BufferManager *bufMgr);
    bool ShouldFlush(const TrajPoint *tp);
};

#endif /* SEGMENTBUILDER_H */
