/*
 * WiFi CNC Controller - Buffer Manager Implementation
 *
 * Credit-based flow control:
 *   1. ESP32 reports buffer_available in each status packet
 *   2. We track "credits" (how many segments we can send)
 *   3. Each segment sent decrements credits
 *   4. Each status report refreshes credits
 *   5. When credits <= 0, segments queue locally
 */

#include "BufferManager.h"
#include "NetworkClient.h"
#include <cstring>

BufferManager::BufferManager(NetworkClient *net)
    : m_network(net)
    , m_queueHead(0)
    , m_queueTail(0)
    , m_remoteAvailable(0)    /* Zero until handshake provides capacity */
    , m_remoteTotal(0)
    , m_packetSegCount(0)
    , m_overflowCount(0)
{
    memset(m_queue, 0, sizeof(m_queue));
    memset(&m_currentPacket, 0, sizeof(m_currentPacket));
}

BufferManager::~BufferManager()
{
}

/* ===================================================================
 * Local Queue Operations
 * =================================================================== */

int BufferManager::QueueCount() const
{
    int count = m_queueHead - m_queueTail;
    if (count < 0) count += LOCAL_QUEUE_SIZE;
    return count;
}

bool BufferManager::QueuePush(const wcnc_motion_segment_t *seg)
{
    int nextHead = (m_queueHead + 1) % LOCAL_QUEUE_SIZE;
    if (nextHead == m_queueTail) return false; /* Queue full */

    memcpy(&m_queue[m_queueHead], seg, sizeof(wcnc_motion_segment_t));
    m_queueHead = nextHead;
    return true;
}

bool BufferManager::QueuePop(wcnc_motion_segment_t *seg)
{
    if (m_queueTail == m_queueHead) return false; /* Queue empty */

    memcpy(seg, &m_queue[m_queueTail], sizeof(wcnc_motion_segment_t));
    m_queueTail = (m_queueTail + 1) % LOCAL_QUEUE_SIZE;
    return true;
}

/* ===================================================================
 * Public Interface
 * =================================================================== */

void BufferManager::QueueSegment(const wcnc_motion_segment_t *seg)
{
    if (!QueuePush(seg)) {
        m_overflowCount++;
    }
}

void BufferManager::SetRemoteCapacity(uint16_t total)
{
    m_remoteTotal = total;
    m_remoteAvailable = total;  /* Assume all slots free after fresh connect */
}

void BufferManager::UpdateRemoteBufferLevel(uint16_t available, uint16_t total)
{
    m_remoteAvailable = available;
    m_remoteTotal = total;
}

void BufferManager::OnCycleStart()
{
    /* Nothing special needed; segments will flow automatically */
}

void BufferManager::Flush()
{
    /* Discard all queued segments */
    m_queueHead = 0;
    m_queueTail = 0;
    m_packetSegCount = 0;
}

/* ===================================================================
 * Packet Batching and Send
 * =================================================================== */

void BufferManager::FlushPacket()
{
    if (m_packetSegCount == 0) return;

    m_currentPacket.segment_count = m_packetSegCount;
    memset(m_currentPacket.reserved, 0, sizeof(m_currentPacket.reserved));

    /* Finalize header */
    uint16_t payloadLen = (uint16_t)(4 + m_packetSegCount *
                                     sizeof(wcnc_motion_segment_t));
    wcnc_finalize_packet(&m_currentPacket, WCNC_PKT_MOTION_SEGMENT,
                          payloadLen, 0, 0); /* Sequence/timestamp set by NetworkClient */

    m_network->SendMotionPacket(&m_currentPacket);

    m_packetSegCount = 0;
    memset(&m_currentPacket, 0, sizeof(m_currentPacket));
}

void BufferManager::ProcessSendQueue()
{
    if (!m_network || !m_network->IsConnected()) return;

    /* Send segments while we have credits and queued data */
    while (QueueCount() > 0 && m_remoteAvailable > 0) {
        wcnc_motion_segment_t seg;
        if (!QueuePop(&seg)) break;

        /* Add to current packet batch */
        memcpy(&m_currentPacket.segments[m_packetSegCount],
               &seg, sizeof(wcnc_motion_segment_t));
        m_packetSegCount++;
        m_remoteAvailable--;

        /* Flush packet when batch is full */
        if (m_packetSegCount >= WCNC_MAX_SEGMENTS_PER_PACKET) {
            FlushPacket();
        }
    }

    /* Flush any remaining partial packet */
    if (m_packetSegCount > 0) {
        FlushPacket();
    }
}
