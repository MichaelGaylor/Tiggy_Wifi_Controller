/*
 * WiFi CNC Controller - Network Client Implementation
 *
 * Manages all network communication with the ESP32 controller.
 * TCP uses length-prefixed framing (2-byte LE prefix + packet data).
 * UDP sends raw packets with protocol headers.
 */

#include "NetworkClient.h"
#include <cstring>
#include <cstdio>

/* ===================================================================
 * Constructor / Destructor
 * =================================================================== */

NetworkClient::NetworkClient()
    : m_tcpSocket(INVALID_SOCKET)
    , m_udpSendSocket(INVALID_SOCKET)
    , m_udpRecvSocket(INVALID_SOCKET)
    , m_connected(false)
    , m_wsaInit(false)
    , m_statusThread(nullptr)
    , m_statusThreadRunning(false)
    , m_hasNewStatus(false)
    , m_txSequence(0)
{
    memset(&m_espAddr, 0, sizeof(m_espAddr));
    memset(&m_latestStatus, 0, sizeof(m_latestStatus));
    memset(&m_handshakeResp, 0, sizeof(m_handshakeResp));
    memset(m_probePosition, 0, sizeof(m_probePosition));
    m_probeSuccess = false;
    m_hasProbeResult = false;
    m_homeAxisMask = 0;
    m_homeSuccess = false;
    m_hasHomeComplete = false;
    m_logFunc = nullptr;
    InitializeCriticalSection(&m_statusLock);
    InitWinsock();
}

NetworkClient::~NetworkClient()
{
    Disconnect();
    DeleteCriticalSection(&m_statusLock);
    if (m_wsaInit) WSACleanup();
}

void NetworkClient::InitWinsock()
{
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
        m_wsaInit = true;
    }
}

/* ===================================================================
 * Timestamp
 * =================================================================== */

uint32_t NetworkClient::GetTimestampUs()
{
    static LARGE_INTEGER freq = {0};
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint32_t)((now.QuadPart * 1000000) / freq.QuadPart);
}

/* ===================================================================
 * Header Construction
 * =================================================================== */

void NetworkClient::BuildHeader(wcnc_header_t *hdr, uint8_t type,
                                 uint16_t payloadLen)
{
    hdr->magic = WCNC_MAGIC;
    hdr->version = WCNC_PROTOCOL_VERSION;
    hdr->packet_type = type;
    hdr->payload_length = payloadLen;
    hdr->sequence = m_txSequence++;
    hdr->timestamp_us = GetTimestampUs();
    hdr->checksum = 0;
}

/* ===================================================================
 * Auto-Discovery via UDP Broadcast
 *
 * Sends a harmless JOG_STOP packet as broadcast on the motion port.
 * The ESP32 responds with a status report, revealing its IP address.
 * =================================================================== */

bool NetworkClient::Discover(char *outIp, size_t outIpSize, int timeoutSec)
{
    if (!m_wsaInit) return false;

    /* Build a harmless JOG_STOP broadcast packet */
    wcnc_jog_stop_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis = 0xFF;  /* All axes - no-op when not jogging */
    wcnc_finalize_packet(&pkt, WCNC_PKT_JOG_STOP,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());

    /* Create broadcast socket */
    SOCKET txSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (txSock == INVALID_SOCKET) return false;

    BOOL broadcast = TRUE;
    setsockopt(txSock, SOL_SOCKET, SO_BROADCAST,
               (const char*)&broadcast, sizeof(broadcast));

    sockaddr_in bcastAddr = {0};
    bcastAddr.sin_family = AF_INET;
    bcastAddr.sin_port = htons(WCNC_UDP_MOTION_PORT);
    bcastAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    /* Create receive socket for status reports */
    SOCKET rxSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rxSock == INVALID_SOCKET) {
        closesocket(txSock);
        return false;
    }

    BOOL reuseAddr = TRUE;
    setsockopt(rxSock, SOL_SOCKET, SO_REUSEADDR,
               (const char*)&reuseAddr, sizeof(reuseAddr));

    sockaddr_in bindAddr = {0};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_port = htons(WCNC_UDP_STATUS_PORT);
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(rxSock, (sockaddr*)&bindAddr, sizeof(bindAddr));

    /* 1-second receive timeout for polling */
    DWORD timeout = 1000;
    setsockopt(rxSock, SOL_SOCKET, SO_RCVTIMEO,
               (const char*)&timeout, sizeof(timeout));

    bool found = false;
    DWORD startTick = GetTickCount();

    while (!found && (GetTickCount() - startTick) < (DWORD)(timeoutSec * 1000)) {
        /* Send broadcast */
        sendto(txSock, (const char*)&pkt, sizeof(pkt), 0,
               (sockaddr*)&bcastAddr, sizeof(bcastAddr));

        /* Listen for responses */
        uint8_t buf[256];
        sockaddr_in fromAddr;
        int fromLen = sizeof(fromAddr);

        int n = recvfrom(rxSock, (char*)buf, sizeof(buf), 0,
                          (sockaddr*)&fromAddr, &fromLen);

        if (n >= (int)sizeof(wcnc_header_t)) {
            const wcnc_header_t *hdr = (const wcnc_header_t*)buf;
            if (hdr->magic == WCNC_MAGIC &&
                hdr->packet_type == WCNC_PKT_STATUS_REPORT) {
                /* Found the ESP32 */
                inet_ntop(AF_INET, &fromAddr.sin_addr,
                           outIp, (socklen_t)outIpSize);
                found = true;
            }
        }
    }

    closesocket(txSock);
    closesocket(rxSock);
    return found;
}

/* ===================================================================
 * TCP Connection
 * =================================================================== */

bool NetworkClient::Connect(const char *ip, uint16_t tcpPort)
{
    if (m_connected) Disconnect();

    m_tcpSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_tcpSocket == INVALID_SOCKET) return false;

    /* Set connection timeout */
    DWORD timeout = 3000;
    setsockopt(m_tcpSocket, SOL_SOCKET, SO_RCVTIMEO,
               (const char*)&timeout, sizeof(timeout));
    setsockopt(m_tcpSocket, SOL_SOCKET, SO_SNDTIMEO,
               (const char*)&timeout, sizeof(timeout));

    /* Resolve and connect */
    m_espAddr.sin_family = AF_INET;
    m_espAddr.sin_port = htons(tcpPort);
    inet_pton(AF_INET, ip, &m_espAddr.sin_addr);

    if (connect(m_tcpSocket, (sockaddr*)&m_espAddr, sizeof(m_espAddr)) != 0) {
        closesocket(m_tcpSocket);
        m_tcpSocket = INVALID_SOCKET;
        return false;
    }

    m_connected = true;
    return true;
}

void NetworkClient::Disconnect()
{
    m_connected = false;

    /* Stop status receive thread */
    if (m_statusThread) {
        m_statusThreadRunning = false;
        WaitForSingleObject(m_statusThread, 2000);
        CloseHandle(m_statusThread);
        m_statusThread = nullptr;
    }

    if (m_tcpSocket != INVALID_SOCKET) {
        closesocket(m_tcpSocket);
        m_tcpSocket = INVALID_SOCKET;
    }
    if (m_udpSendSocket != INVALID_SOCKET) {
        closesocket(m_udpSendSocket);
        m_udpSendSocket = INVALID_SOCKET;
    }
    if (m_udpRecvSocket != INVALID_SOCKET) {
        closesocket(m_udpRecvSocket);
        m_udpRecvSocket = INVALID_SOCKET;
    }
}

bool NetworkClient::IsConnected() const
{
    return m_connected;
}

/* ===================================================================
 * TCP Framing (length-prefixed)
 * =================================================================== */

bool NetworkClient::TcpSendPacket(const void *packet, size_t len)
{
    if (!m_connected || m_tcpSocket == INVALID_SOCKET) return false;

    /* Send 2-byte length prefix (little-endian) */
    uint8_t lenBuf[2] = {
        (uint8_t)(len & 0xFF),
        (uint8_t)((len >> 8) & 0xFF)
    };

    if (send(m_tcpSocket, (const char*)lenBuf, 2, 0) != 2) {
        m_connected = false;
        return false;
    }

    if (send(m_tcpSocket, (const char*)packet, (int)len, 0) != (int)len) {
        m_connected = false;
        return false;
    }

    return true;
}

bool NetworkClient::TcpRecvPacket(void *buffer, size_t bufSize, size_t *outLen)
{
    if (!m_connected || m_tcpSocket == INVALID_SOCKET) return false;

    /* Read 2-byte length prefix */
    uint8_t lenBuf[2];
    int received = 0;
    while (received < 2) {
        int n = recv(m_tcpSocket, (char*)lenBuf + received, 2 - received, 0);
        if (n == 0) { m_connected = false; return false; }  /* Clean close */
        if (n < 0) {
            int err = WSAGetLastError();
            if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
                return false;  /* Timeout — connection still alive */
            m_connected = false;
            return false;
        }
        received += n;
    }

    uint16_t pktLen = (uint16_t)lenBuf[0] | ((uint16_t)lenBuf[1] << 8);
    if (pktLen > bufSize) { m_connected = false; return false; }

    /* Read full packet */
    received = 0;
    while (received < (int)pktLen) {
        int n = recv(m_tcpSocket, (char*)buffer + received,
                     pktLen - received, 0);
        if (n == 0) { m_connected = false; return false; }
        if (n < 0) {
            int err = WSAGetLastError();
            if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK)
                return false;
            m_connected = false;
            return false;
        }
        received += n;
    }

    *outLen = pktLen;
    return true;
}

/* ===================================================================
 * UDP
 * =================================================================== */

void NetworkClient::StartUDP(const char *ip)
{
    /* Create send socket */
    m_udpSendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_logFunc) {
        m_logFunc("UDP: send socket = %s (dest=%s:%d)",
                  m_udpSendSocket != INVALID_SOCKET ? "OK" : "FAILED",
                  ip, WCNC_UDP_MOTION_PORT);
    }

    /* Create receive socket for status reports */
    m_udpRecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_udpRecvSocket != INVALID_SOCKET) {
        /* Allow port reuse (critical if previous instance didn't close cleanly) */
        BOOL reuseAddr = TRUE;
        setsockopt(m_udpRecvSocket, SOL_SOCKET, SO_REUSEADDR,
                   (const char*)&reuseAddr, sizeof(reuseAddr));

        sockaddr_in bindAddr = {0};
        bindAddr.sin_family = AF_INET;
        bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        bindAddr.sin_port = htons(WCNC_UDP_STATUS_PORT);
        int bindResult = bind(m_udpRecvSocket, (sockaddr*)&bindAddr, sizeof(bindAddr));

        if (m_logFunc) {
            if (bindResult == 0) {
                m_logFunc("UDP: recv socket bound to port %d OK", WCNC_UDP_STATUS_PORT);
            } else {
                m_logFunc("UDP: recv socket bind FAILED (err=%d) - status reports will not work!",
                          WSAGetLastError());
            }
        }

        /* Receive timeout for polling */
        DWORD timeout = 100;
        setsockopt(m_udpRecvSocket, SOL_SOCKET, SO_RCVTIMEO,
                   (const char*)&timeout, sizeof(timeout));
    } else if (m_logFunc) {
        m_logFunc("UDP: recv socket creation FAILED");
    }

    /* Send a test packet to verify UDP path (harmless JOG_STOP all axes) */
    {
        wcnc_jog_stop_packet_t pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.axis = 0xFF;
        wcnc_finalize_packet(&pkt, WCNC_PKT_JOG_STOP,
                              sizeof(pkt) - sizeof(wcnc_header_t),
                              m_txSequence++, GetTimestampUs());
        bool testSent = UdpSendPacket(&pkt, sizeof(pkt));
        if (m_logFunc) {
            m_logFunc("UDP: test packet (JOG_STOP) sent = %s", testSent ? "OK" : "FAILED");
        }
    }

    /* Start status receive thread */
    m_statusThreadRunning = true;
    m_statusThread = CreateThread(nullptr, 0, StatusReceiveThread,
                                   this, 0, nullptr);
}

bool NetworkClient::UdpSendPacket(const void *packet, size_t len)
{
    if (m_udpSendSocket == INVALID_SOCKET) return false;

    sockaddr_in dest = m_espAddr;
    dest.sin_port = htons(WCNC_UDP_MOTION_PORT);

    int sent = sendto(m_udpSendSocket, (const char*)packet, (int)len, 0,
                       (sockaddr*)&dest, sizeof(dest));
    return (sent == (int)len);
}

/* ===================================================================
 * Status Receive Thread
 * =================================================================== */

DWORD WINAPI NetworkClient::StatusReceiveThread(LPVOID lpParam)
{
    NetworkClient *self = static_cast<NetworkClient*>(lpParam);
    uint8_t buffer[256];

    while (self->m_statusThreadRunning) {
        sockaddr_in fromAddr;
        int fromLen = sizeof(fromAddr);

        int len = recvfrom(self->m_udpRecvSocket, (char*)buffer,
                            sizeof(buffer), 0,
                            (sockaddr*)&fromAddr, &fromLen);

        if (len < (int)sizeof(wcnc_header_t)) continue;

        /* Validate packet */
        const wcnc_header_t *hdr = (const wcnc_header_t*)buffer;
        if (hdr->magic != WCNC_MAGIC) continue;
        if (!wcnc_validate_packet(buffer, len)) continue;

        EnterCriticalSection(&self->m_statusLock);

        switch (hdr->packet_type) {
        case WCNC_PKT_STATUS_REPORT:
            if (len >= (int)sizeof(wcnc_status_packet_t)) {
                const wcnc_status_packet_t *pkt = (const wcnc_status_packet_t*)buffer;
                memcpy(&self->m_latestStatus, &pkt->report, sizeof(wcnc_status_report_t));
                self->m_hasNewStatus = true;
            }
            break;

        case WCNC_PKT_PROBE_RESULT:
            if (len >= (int)sizeof(wcnc_probe_result_packet_t)) {
                const wcnc_probe_result_packet_t *pkt =
                    (const wcnc_probe_result_packet_t*)buffer;
                memcpy(self->m_probePosition, pkt->position_steps,
                       sizeof(self->m_probePosition));
                self->m_probeSuccess = (pkt->success != 0);
                self->m_hasProbeResult = true;
            }
            break;

        case WCNC_PKT_HOME_COMPLETE:
            if (len >= (int)sizeof(wcnc_home_complete_packet_t)) {
                const wcnc_home_complete_packet_t *pkt =
                    (const wcnc_home_complete_packet_t*)buffer;
                self->m_homeAxisMask = pkt->axis_mask;
                self->m_homeSuccess = (pkt->success != 0);
                self->m_hasHomeComplete = true;
            }
            break;

        default:
            break;
        }

        LeaveCriticalSection(&self->m_statusLock);
    }

    return 0;
}

bool NetworkClient::GetLatestStatus(wcnc_status_report_t *status)
{
    EnterCriticalSection(&m_statusLock);
    bool hasNew = m_hasNewStatus;
    if (m_hasNewStatus) {
        m_hasNewStatus = false;
    }
    /* Always copy the latest status — multiple callers (plugin timer +
     * config dialog) need to read the same data without racing. */
    memcpy(status, &m_latestStatus, sizeof(wcnc_status_report_t));
    LeaveCriticalSection(&m_statusLock);
    return hasNew;
}

bool NetworkClient::GetProbeResult(int32_t position_steps[WCNC_MAX_AXES], bool *success)
{
    EnterCriticalSection(&m_statusLock);
    bool hasResult = m_hasProbeResult;
    if (hasResult) {
        memcpy(position_steps, m_probePosition, sizeof(m_probePosition));
        *success = m_probeSuccess;
        m_hasProbeResult = false;
    }
    LeaveCriticalSection(&m_statusLock);
    return hasResult;
}

bool NetworkClient::GetHomeComplete(uint8_t *axis_mask, bool *success)
{
    EnterCriticalSection(&m_statusLock);
    bool hasResult = m_hasHomeComplete;
    if (hasResult) {
        *axis_mask = m_homeAxisMask;
        *success = m_homeSuccess;
        m_hasHomeComplete = false;
    }
    LeaveCriticalSection(&m_statusLock);
    return hasResult;
}

/* ===================================================================
 * Handshake
 * =================================================================== */

bool NetworkClient::Handshake()
{
    wcnc_handshake_req_t req;
    memset(&req, 0, sizeof(req));
    req.host_version = WCNC_MAKE_VERSION(1, 0, 0);
    strcpy_s(req.host_name, "Mach3-Tiggy");

    wcnc_finalize_packet(&req, WCNC_PKT_HANDSHAKE_REQ,
                          sizeof(req) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());

    if (!TcpSendPacket(&req, sizeof(req))) return false;

    /* Receive response */
    uint8_t respBuf[256];
    size_t respLen;
    if (!TcpRecvPacket(respBuf, sizeof(respBuf), &respLen)) return false;

    if (respLen < sizeof(wcnc_handshake_resp_t)) return false;

    const wcnc_header_t *hdr = (const wcnc_header_t*)respBuf;
    if (hdr->packet_type != WCNC_PKT_HANDSHAKE_RESP) return false;

    memcpy(&m_handshakeResp, respBuf, sizeof(m_handshakeResp));
    return true;
}

/* ===================================================================
 * Configuration Commands (TCP)
 * =================================================================== */

bool NetworkClient::SendConfig(uint16_t key, const void *value,
                                uint16_t valueType)
{
    wcnc_config_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.key = key;
    pkt.value_type = valueType;

    switch (valueType) {
    case WCNC_VAL_FLOAT:
        memcpy(pkt.value, value, sizeof(float));
        break;
    case WCNC_VAL_UINT32:
        memcpy(pkt.value, value, sizeof(uint32_t));
        break;
    case WCNC_VAL_UINT16:
        memcpy(pkt.value, value, sizeof(uint16_t));
        break;
    case WCNC_VAL_UINT8:
        memcpy(pkt.value, value, sizeof(uint8_t));
        break;
    case WCNC_VAL_STRING:
        strncpy_s((char*)pkt.value, sizeof(pkt.value),
                  (const char*)value, _TRUNCATE);
        break;
    }

    wcnc_finalize_packet(&pkt, WCNC_PKT_CONFIG_SET,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());

    return TcpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendConfigSave()
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    wcnc_finalize_packet(&pkt, WCNC_PKT_CONFIG_SAVE, 0,
                          m_txSequence++, GetTimestampUs());
    return TcpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::ReadConfig(uint16_t key, void *outValue,
                                uint16_t *outValueType)
{
    /* Build CFG_GET request (only key matters, value is ignored) */
    wcnc_config_packet_t req;
    memset(&req, 0, sizeof(req));
    req.key = key;
    wcnc_finalize_packet(&req, WCNC_PKT_CONFIG_GET,
                          sizeof(req) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());

    if (!TcpSendPacket(&req, sizeof(req))) return false;

    /* Read responses, skipping stale ACKs from prior CFG_SET commands.
     * SyncConfigToESP32 sends many CFG_SETs without reading the ACKs,
     * so they accumulate in the TCP buffer ahead of our CONFIG_RESP. */
    for (int attempt = 0; attempt < 64; attempt++) {
        uint8_t respBuf[256];
        size_t respLen;
        if (!TcpRecvPacket(respBuf, sizeof(respBuf), &respLen)) return false;
        if (respLen < sizeof(wcnc_header_t)) continue;

        const wcnc_header_t *hdr = (const wcnc_header_t *)respBuf;
        if (hdr->packet_type != WCNC_PKT_CONFIG_RESP) continue; /* skip ACK etc. */
        if (respLen < sizeof(wcnc_config_packet_t)) return false;

        const wcnc_config_packet_t *resp = (const wcnc_config_packet_t *)respBuf;
        if (resp->key != key) continue; /* wrong key, skip */

        *outValueType = resp->value_type;

        switch (resp->value_type) {
        case WCNC_VAL_UINT8:
            memcpy(outValue, resp->value, sizeof(uint8_t));
            break;
        case WCNC_VAL_UINT16:
            memcpy(outValue, resp->value, sizeof(uint16_t));
            break;
        case WCNC_VAL_UINT32:
        case WCNC_VAL_FLOAT:
            memcpy(outValue, resp->value, sizeof(uint32_t));
            break;
        case WCNC_VAL_STRING:
            memcpy(outValue, resp->value, WCNC_CONFIG_VALUE_LEN);
            break;
        default:
            memcpy(outValue, resp->value, sizeof(uint32_t));
            break;
        }

        return true;
    }

    return false; /* Too many non-matching packets */
}

bool NetworkClient::ReadPinMap(PinMapEntry *entry)
{
    PinMap_Init(entry);

    if (!m_connected || m_tcpSocket == INVALID_SOCKET) return false;

    /* Disable Nagle to ensure each send goes out immediately.
     * Without this, the 2-byte length prefix + 84-byte body from
     * TcpSendPacket get delayed by Nagle, causing the ESP32 to
     * process requests out of order or with long gaps. */
    BOOL nodelay = TRUE;
    setsockopt(m_tcpSocket, IPPROTO_TCP, TCP_NODELAY,
               (const char*)&nodelay, sizeof(nodelay));

    /* ── Phase 1: Flush stale TCP data (ACKs from prior CFG_SET calls) ── */
    DWORD flushTimeout = 10;
    DWORD readTimeout  = 500;   /* 500ms per pin — generous for WiFi */
    DWORD normalTimeout = 3000;

    setsockopt(m_tcpSocket, SOL_SOCKET, SO_RCVTIMEO,
               (const char*)&flushTimeout, sizeof(flushTimeout));
    for (int i = 0; i < 128; i++) {
        uint8_t junk[256];
        size_t junkLen;
        if (!TcpRecvPacket(junk, sizeof(junk), &junkLen)) break;
    }

    /* ── Phase 2: Read each pin sequentially (send request → wait for response) ── */
    setsockopt(m_tcpSocket, SOL_SOCKET, SO_RCVTIMEO,
               (const char*)&readTimeout, sizeof(readTimeout));

    /* Helper: read a uint8 pin value, return -1 on failure or 0xFF (NC) */
    auto readPin = [this](uint16_t key) -> int8_t {
        uint8_t buf[WCNC_CONFIG_VALUE_LEN] = {0};
        uint16_t vtype = 0;
        if (!ReadConfig(key, buf, &vtype)) return -1;
        return (buf[0] == 0xFF) ? -1 : (int8_t)buf[0];
    };

    int received = 0;
    auto read = [&](uint16_t key) -> int8_t {
        int8_t v = readPin(key);
        if (v >= 0) received++;
        return v;
    };

    /* Step pins */
    entry->stepPin[0] = read(WCNC_CFG_PIN_STEP_X);
    entry->stepPin[1] = read(WCNC_CFG_PIN_STEP_Y);
    entry->stepPin[2] = read(WCNC_CFG_PIN_STEP_Z);
    entry->stepPin[3] = read(WCNC_CFG_PIN_STEP_A);
    entry->stepPin[4] = read(WCNC_CFG_PIN_STEP_B);
    entry->stepPin[5] = read(WCNC_CFG_PIN_STEP_C);

    /* Direction pins */
    entry->dirPin[0] = read(WCNC_CFG_PIN_DIR_X);
    entry->dirPin[1] = read(WCNC_CFG_PIN_DIR_Y);
    entry->dirPin[2] = read(WCNC_CFG_PIN_DIR_Z);
    entry->dirPin[3] = read(WCNC_CFG_PIN_DIR_A);
    entry->dirPin[4] = read(WCNC_CFG_PIN_DIR_B);
    entry->dirPin[5] = read(WCNC_CFG_PIN_DIR_C);

    /* Limit pins (shared with home) */
    for (int i = 0; i < PINMAP_MAX_AXES; i++) {
        static const uint16_t limitKeys[] = {
            WCNC_CFG_PIN_LIMIT_X, WCNC_CFG_PIN_LIMIT_Y, WCNC_CFG_PIN_LIMIT_Z,
            WCNC_CFG_PIN_LIMIT_A, WCNC_CFG_PIN_LIMIT_B, WCNC_CFG_PIN_LIMIT_C
        };
        entry->limitPin[i] = read(limitKeys[i]);
        entry->homePin[i]  = entry->limitPin[i];
    }

    /* Single-pin entries */
    entry->enablePin     = read(WCNC_CFG_PIN_ENABLE);
    entry->probePin      = read(WCNC_CFG_PIN_PROBE);
    entry->estopPin      = read(WCNC_CFG_PIN_ESTOP);
    entry->spindlePin    = read(WCNC_CFG_PIN_SPINDLE);
    entry->chargePumpPin = read(WCNC_CFG_PIN_CHARGE_PUMP);
    entry->ledPin        = read(WCNC_CFG_PIN_LED);

    /* Misc outputs & inputs */
    entry->miscOutPin[0] = read(WCNC_CFG_PIN_MISC_OUT0);
    entry->miscOutPin[1] = read(WCNC_CFG_PIN_MISC_OUT1);
    entry->miscInPin[0]  = read(WCNC_CFG_PIN_MISC_IN0);
    entry->miscInPin[1]  = read(WCNC_CFG_PIN_MISC_IN1);
    entry->miscInPin[2]  = read(WCNC_CFG_PIN_MISC_IN2);
    entry->miscInPin[3]  = read(WCNC_CFG_PIN_MISC_IN3);

    /* ── Restore socket state ── */
    setsockopt(m_tcpSocket, SOL_SOCKET, SO_RCVTIMEO,
               (const char*)&normalTimeout, sizeof(normalTimeout));
    nodelay = FALSE;
    setsockopt(m_tcpSocket, IPPROTO_TCP, TCP_NODELAY,
               (const char*)&nodelay, sizeof(nodelay));

    if (m_logFunc)
        m_logFunc("ReadPinMap: received %d/30 pin values", received);

    /* Board name from handshake response */
    strncpy_s(entry->name, sizeof(entry->name),
              m_handshakeResp.device_name, _TRUNCATE);
    strncpy_s(entry->description, sizeof(entry->description),
              "Live pin assignments from controller", _TRUNCATE);

    return (received > 0);
}

/* ===================================================================
 * Motion Commands (UDP)
 * =================================================================== */

bool NetworkClient::SendMotionPacket(const wcnc_motion_packet_t *pkt)
{
    return UdpSendPacket(pkt,
        sizeof(wcnc_header_t) + 4 +
        pkt->segment_count * sizeof(wcnc_motion_segment_t));
}

bool NetworkClient::SendIOControlPacket(const wcnc_io_control_packet_t *pkt)
{
    return UdpSendPacket(pkt, sizeof(wcnc_io_control_packet_t));
}

bool NetworkClient::SendEStop()
{
    /* Send 3x for redundancy */
    wcnc_estop_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    for (int i = 0; i < 3; i++) {
        wcnc_finalize_packet(&pkt, WCNC_PKT_ESTOP, 0,
                              m_txSequence++, GetTimestampUs());
        UdpSendPacket(&pkt, sizeof(pkt));
    }
    return true;
}

bool NetworkClient::SendFeedHold()
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    wcnc_finalize_packet(&pkt, WCNC_PKT_FEED_HOLD, 0,
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendFeedResume()
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    wcnc_finalize_packet(&pkt, WCNC_PKT_FEED_RESUME, 0,
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendReset()
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    wcnc_finalize_packet(&pkt, WCNC_PKT_RESET, 0,
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendJogCommand(uint8_t axis, int8_t dir, uint32_t speed)
{
    wcnc_jog_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis = axis;
    pkt.direction = dir;
    pkt.speed = speed;
    wcnc_finalize_packet(&pkt, WCNC_PKT_JOG_COMMAND,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendJogStop(uint8_t axis)
{
    wcnc_jog_stop_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis = axis;
    wcnc_finalize_packet(&pkt, WCNC_PKT_JOG_STOP,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}

bool NetworkClient::SendHomeCommand(uint8_t axis_mask)
{
    wcnc_home_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis_mask = axis_mask;
    wcnc_finalize_packet(&pkt, WCNC_PKT_HOME_COMMAND,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          m_txSequence++, GetTimestampUs());
    return UdpSendPacket(&pkt, sizeof(pkt));
}
