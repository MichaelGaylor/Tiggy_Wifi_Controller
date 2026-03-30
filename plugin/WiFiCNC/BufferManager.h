/*
 * WiFi CNC Controller - Buffer Manager
 *
 * Manages flow control between the PC and ESP32.
 * Queues segments locally when the ESP32 buffer is full,
 * and batches segments into UDP packets for efficiency.
 */

#ifndef BUFFERMANAGER_H
#define BUFFERMANAGER_H

#include "../../protocol/wifi_cnc_protocol.h"
#include <cstdint>

class NetworkClient;

class BufferManager {
public:
    BufferManager(NetworkClient *net);
    ~BufferManager();

    /* Queue a segment for transmission */
    void QueueSegment(const wcnc_motion_segment_t *seg);

    /* Process the send queue (called each piUpdate cycle) */
    void ProcessSendQueue();

    /* Set remote buffer capacity from handshake (call once after connect) */
    void SetRemoteCapacity(uint16_t total);

    /* Update remote buffer state from status reports */
    void UpdateRemoteBufferLevel(uint16_t available, uint16_t total);

    /* Cycle start notification */
    void OnCycleStart();

    /* Flush all pending segments */
    void Flush();

    /* Diagnostics */
    uint32_t GetOverflowCount() const { return m_overflowCount; }

private:
    NetworkClient *m_network;

    /* Local send queue */
    static const int LOCAL_QUEUE_SIZE = 256;
    wcnc_motion_segment_t m_queue[LOCAL_QUEUE_SIZE];
    int m_queueHead;
    int m_queueTail;

    /* Remote buffer tracking (credit-based flow control) */
    uint16_t m_remoteAvailable;
    uint16_t m_remoteTotal;

    /* Packet batching */
    wcnc_motion_packet_t m_currentPacket;
    uint8_t m_packetSegCount;

    /* Overflow tracking */
    uint32_t m_overflowCount;

    int QueueCount() const;
    bool QueuePush(const wcnc_motion_segment_t *seg);
    bool QueuePop(wcnc_motion_segment_t *seg);
    void FlushPacket();
};

#endif /* BUFFERMANAGER_H */
