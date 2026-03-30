/*
 * WiFi CNC Controller - Network Client
 *
 * Manages TCP and UDP connections to the ESP32 controller.
 * TCP: configuration, handshake, keepalive
 * UDP: motion segments (send), status reports (receive)
 */

#ifndef NETWORKCLIENT_H
#define NETWORKCLIENT_H

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "../../protocol/wifi_cnc_protocol.h"
#include "PinMap.h"
#include <cstdint>

/* Log callback type - set via SetLogCallback to enable network diagnostics */
typedef void (*NetworkLogFunc)(const char *fmt, ...);

class NetworkClient {
public:
    NetworkClient();
    ~NetworkClient();

    void SetLogCallback(NetworkLogFunc func) { m_logFunc = func; }

    /* Auto-discover ESP32 on the network via UDP broadcast.
     * Returns true if found, writes IP to outIp buffer. */
    bool Discover(char *outIp, size_t outIpSize, int timeoutSec = 5);

    /* TCP connection to ESP32 */
    bool Connect(const char *ip, uint16_t tcpPort);
    void Disconnect();
    bool IsConnected() const;

    /* Handshake and keepalive */
    bool Handshake();
    const wcnc_handshake_resp_t* GetHandshakeResp() const { return &m_handshakeResp; }

    /* Start UDP channels (call after TCP handshake) */
    void StartUDP(const char *ip);

    /* Configuration commands (TCP) */
    bool SendConfig(uint16_t key, const void *value, uint16_t valueType);
    bool SendConfigSave();
    bool ReadConfig(uint16_t key, void *outValue, uint16_t *outValueType);
    bool ReadPinMap(PinMapEntry *entry);

    /* Motion commands (UDP) */
    bool SendMotionPacket(const wcnc_motion_packet_t *pkt);
    bool SendIOControlPacket(const wcnc_io_control_packet_t *pkt);
    bool SendEStop();
    bool SendFeedHold();
    bool SendFeedResume();
    bool SendReset();
    bool SendJogCommand(uint8_t axis, int8_t dir, uint32_t speed);
    bool SendJogStop(uint8_t axis);
    bool SendHomeCommand(uint8_t axis_mask);

    /* Status reception (non-blocking poll) */
    bool GetLatestStatus(wcnc_status_report_t *status);

    /* Probe/Home completion (non-blocking poll, consumed on read) */
    bool GetProbeResult(int32_t position_steps[WCNC_MAX_AXES], bool *success);
    bool GetHomeComplete(uint8_t *axis_mask, bool *success);

private:
    SOCKET m_tcpSocket;
    SOCKET m_udpSendSocket;
    SOCKET m_udpRecvSocket;
    sockaddr_in m_espAddr;
    bool m_connected;
    bool m_wsaInit;

    /* Status receive thread */
    HANDLE m_statusThread;
    static DWORD WINAPI StatusReceiveThread(LPVOID lpParam);
    volatile bool m_statusThreadRunning;

    CRITICAL_SECTION m_statusLock;
    wcnc_status_report_t m_latestStatus;
    bool m_hasNewStatus;

    /* Probe result */
    int32_t m_probePosition[WCNC_MAX_AXES];
    bool m_probeSuccess;
    bool m_hasProbeResult;

    /* Home completion */
    uint8_t m_homeAxisMask;
    bool m_homeSuccess;
    bool m_hasHomeComplete;

    uint32_t m_txSequence;
    NetworkLogFunc m_logFunc;

    /* Handshake response data */
    wcnc_handshake_resp_t m_handshakeResp;

    /* Internal helpers */
    void InitWinsock();
    void BuildHeader(wcnc_header_t *hdr, uint8_t type, uint16_t payloadLen);
    bool TcpSendPacket(const void *packet, size_t len);
    bool TcpRecvPacket(void *buffer, size_t bufSize, size_t *outLen);
    bool UdpSendPacket(const void *packet, size_t len);
    uint32_t GetTimestampUs();
};

#endif /* NETWORKCLIENT_H */
