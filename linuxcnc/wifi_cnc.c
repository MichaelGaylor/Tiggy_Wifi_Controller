/*
 * WiFi CNC Controller - LinuxCNC HAL Userspace Component
 *
 * Communicates with the ESP32 motion controller over WiFi using
 * the same protocol as the Mach3 plugin.
 *
 * Compile:   halcompile --install --userspace wifi_cnc.c
 * Load:      loadusr -W wifi_cnc --ip=192.168.4.1 --joints=3 --config=wifi_cnc.conf
 *
 * Based on the Remora project pattern and gs2_vfd reference.
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
#include <getopt.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <pthread.h>

#include "../protocol/wifi_cnc_protocol.h"

/* ===================================================================
 * Constants
 * =================================================================== */

#define COMP_NAME       "wifi-cnc"
#define MAX_JOINTS      WCNC_MAX_AXES
#define DEFAULT_IP      "192.168.4.1"
#define UPDATE_RATE_HZ  100
#define UPDATE_PERIOD_US (1000000 / UPDATE_RATE_HZ)

/* ===================================================================
 * Configuration (loaded from wifi_cnc.conf)
 * =================================================================== */

typedef struct {
    /* Timing */
    uint16_t step_pulse_us;
    uint16_t dir_setup_us;
    uint16_t step_idle_delay_ms;

    /* Inversion bitmasks (bit per axis) */
    uint8_t invert_step;
    uint8_t invert_dir;
    uint8_t invert_limit;
    uint8_t invert_home;
    uint8_t invert_estop;
    uint8_t invert_probe;

    /* Homing */
    uint8_t  homing_dir_mask;
    uint32_t homing_seek_rate;
    uint32_t homing_feed_rate;
    uint32_t homing_pulloff;

    /* Spindle */
    uint16_t spindle_pwm_freq;
    uint32_t spindle_max_rpm;
    int      laser_mode;

    /* Charge pump */
    uint16_t charge_pump_freq;

    /* Misc */
    int shared_limits;

    /* Axis cloning */
    int clone_master[MAX_JOINTS];    /* -1 = none */
    int clone_reversed[MAX_JOINTS];  /* 0 or 1 */
} wifi_cnc_config_t;

static wifi_cnc_config_t g_config = {
    .step_pulse_us = 5,
    .dir_setup_us = 5,
    .step_idle_delay_ms = 0,
    .invert_step = 0, .invert_dir = 0,
    .invert_limit = 0, .invert_home = 0,
    .invert_estop = 0, .invert_probe = 0,
    .homing_dir_mask = 0,
    .homing_seek_rate = 500, .homing_feed_rate = 50, .homing_pulloff = 200,
    .spindle_pwm_freq = 1000, .spindle_max_rpm = 24000, .laser_mode = 0,
    .charge_pump_freq = 0,
    .shared_limits = 1,
    .clone_master = {-1, -1, -1, -1, -1, -1},
    .clone_reversed = {0, 0, 0, 0, 0, 0},
};

static char config_path[256] = "";

/* ===================================================================
 * HAL Pin Data Structure
 * =================================================================== */

typedef struct {
    /* Per-joint pins */
    hal_float_t *pos_cmd[MAX_JOINTS];       /* IN:  commanded position */
    hal_float_t *pos_fb[MAX_JOINTS];        /* OUT: position feedback */
    hal_s32_t   *counts[MAX_JOINTS];        /* OUT: raw step counts */
    hal_float_t *freq_cmd[MAX_JOINTS];      /* OUT: current step frequency */
    hal_bit_t   *joint_enable[MAX_JOINTS];  /* IN:  amplifier enable */
    hal_bit_t   *neg_lim[MAX_JOINTS];       /* OUT: negative limit switch */
    hal_bit_t   *pos_lim[MAX_JOINTS];       /* OUT: positive limit switch */
    hal_bit_t   *home_sw[MAX_JOINTS];       /* OUT: home switch */

    /* Per-joint parameters */
    hal_float_t scale[MAX_JOINTS];          /* steps per user unit */
    hal_float_t maxvel[MAX_JOINTS];         /* max velocity (units/sec) */
    hal_float_t maxaccel[MAX_JOINTS];       /* max acceleration (units/sec^2) */

    /* Global pins */
    hal_bit_t   *enable;                    /* IN:  global enable */
    hal_bit_t   *estop_in;                  /* IN:  e-stop from LinuxCNC */
    hal_bit_t   *connected;                 /* OUT: ESP32 connection status */
    hal_bit_t   *estop_out;                 /* OUT: e-stop from ESP32 */
    hal_bit_t   *probe;                     /* OUT: probe input state */
    hal_float_t *feed_rate;                 /* OUT: current feed rate */
    hal_bit_t   *charge_pump;               /* IN:  charge pump enable */
    hal_bit_t   *probe_active;              /* IN:  probing in progress */

    /* Misc I/O pins */
    hal_bit_t   *misc_out[5];              /* IN:  misc output control */
    hal_bit_t   *misc_in[5];               /* OUT: misc input state */

    /* Spindle / Coolant */
    hal_bit_t   *spindle_on;               /* IN:  spindle enable */
    hal_bit_t   *spindle_cw;               /* IN:  spindle direction (1=CW) */
    hal_float_t *spindle_speed;            /* IN:  spindle speed command (RPM) */
    hal_bit_t   *coolant_flood;            /* IN:  flood coolant */
    hal_bit_t   *coolant_mist;             /* IN:  mist coolant */

    /* Spindle encoder feedback (v1.1) */
    hal_float_t *spindle_revs;             /* OUT: cumulative revolutions (for threading) */
    hal_float_t *spindle_speed_fb;         /* OUT: speed in RPS */
    hal_bit_t   *spindle_index_enable;     /* I/O: index enable (reset on index pulse) */
    hal_float_t *spindle_rpm_fb;           /* OUT: RPM feedback */

    /* I/O expansion module (v1.1) */
    hal_bit_t   *io_mod_in[16];            /* OUT: I/O module input channels */
    hal_bit_t   *io_mod_out[16];           /* IN:  I/O module output channels */
    hal_bit_t   *io_mod_connected;         /* OUT: I/O module connection status */

    /* Parameters */
    hal_bit_t   shared_limits;             /* RW: 1=same bit for neg/pos lim */
} wifi_cnc_data_t;

/* ===================================================================
 * Global State
 * =================================================================== */

static int comp_id;
static int done = 0;
static int num_joints = 3;
static char esp_ip[64] = DEFAULT_IP;
static int ip_specified = 0;

static int tcp_sock = -1;
static int udp_send_sock = -1;
static int udp_recv_sock = -1;
static struct sockaddr_in esp_addr;
static uint32_t tx_sequence = 0;

static volatile int esp_connected = 0;
static wcnc_status_report_t latest_status;
static pthread_mutex_t status_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t status_thread;
static volatile int status_thread_running = 0;

static double prev_pos_cmd[MAX_JOINTS] = {0};
static double prev_vel[MAX_JOINTS] = {0};
static int32_t accumulated_steps[MAX_JOINTS] = {0};

/* Position offset tracking */
static int32_t g_pos_offset[MAX_JOINTS] = {0};
static int pos_initialized = 0;

/* Charge pump state tracking */
static int prev_charge_pump = -1;

/* Reconnection backoff */
static int reconnect_attempts = 0;
static uint32_t next_reconnect_ms = 0;

/* I/O expansion module (second ESP32) */
static char io_mod_ip[64] = "";
static int io_mod_enabled = 0;
static int io_mod_tcp_sock = -1;
static int io_mod_udp_send_sock = -1;
static int io_mod_udp_recv_sock = -1;
static struct sockaddr_in io_mod_addr;
static volatile int io_mod_connected = 0;
static wcnc_status_report_t io_mod_status;
static pthread_mutex_t io_mod_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t io_mod_status_thread;
static volatile int io_mod_thread_running = 0;
static int io_mod_reconnect_attempts = 0;
static uint32_t io_mod_next_reconnect_ms = 0;
static uint32_t io_mod_tx_sequence = 0;

/* ===================================================================
 * Config File Parser
 * =================================================================== */

static void parse_config_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        rtapi_print_msg(RTAPI_MSG_WARN,
            "%s: Config file not found: %s (using defaults)\n",
            COMP_NAME, path);
        return;
    }

    char line[256];
    char axis_names[] = "XYZABC";

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == ';' || *p == '[' || *p == '\n' || *p == '\0')
            continue;

        char key[64] = {0}, val[64] = {0};
        if (sscanf(p, "%63[^= ] = %63s", key, val) != 2)
            continue;

        /* Connection */
        if (strcmp(key, "esp_ip") == 0 && strcmp(val, "auto") != 0) {
            strncpy(esp_ip, val, sizeof(esp_ip) - 1);
            ip_specified = 1;
        }
        else if (strcmp(key, "num_joints") == 0) {
            int n = atoi(val);
            if (n >= 1 && n <= MAX_JOINTS) num_joints = n;
        }
        /* Timing */
        else if (strcmp(key, "step_pulse_us") == 0)
            g_config.step_pulse_us = (uint16_t)atoi(val);
        else if (strcmp(key, "dir_setup_us") == 0)
            g_config.dir_setup_us = (uint16_t)atoi(val);
        else if (strcmp(key, "step_idle_delay_ms") == 0)
            g_config.step_idle_delay_ms = (uint16_t)atoi(val);
        /* Inversion */
        else if (strcmp(key, "invert_step") == 0)
            g_config.invert_step = (uint8_t)atoi(val);
        else if (strcmp(key, "invert_dir") == 0)
            g_config.invert_dir = (uint8_t)atoi(val);
        else if (strcmp(key, "invert_limit") == 0)
            g_config.invert_limit = (uint8_t)atoi(val);
        else if (strcmp(key, "invert_home") == 0)
            g_config.invert_home = (uint8_t)atoi(val);
        else if (strcmp(key, "invert_estop") == 0)
            g_config.invert_estop = (uint8_t)atoi(val);
        else if (strcmp(key, "invert_probe") == 0)
            g_config.invert_probe = (uint8_t)atoi(val);
        /* Homing */
        else if (strcmp(key, "homing_dir_mask") == 0)
            g_config.homing_dir_mask = (uint8_t)atoi(val);
        else if (strcmp(key, "homing_seek_rate") == 0)
            g_config.homing_seek_rate = (uint32_t)atoi(val);
        else if (strcmp(key, "homing_feed_rate") == 0)
            g_config.homing_feed_rate = (uint32_t)atoi(val);
        else if (strcmp(key, "homing_pulloff") == 0)
            g_config.homing_pulloff = (uint32_t)atoi(val);
        /* Spindle */
        else if (strcmp(key, "spindle_pwm_freq") == 0)
            g_config.spindle_pwm_freq = (uint16_t)atoi(val);
        else if (strcmp(key, "spindle_max_rpm") == 0)
            g_config.spindle_max_rpm = (uint32_t)atoi(val);
        else if (strcmp(key, "laser_mode") == 0)
            g_config.laser_mode = atoi(val);
        /* Charge pump */
        else if (strcmp(key, "charge_pump_freq") == 0)
            g_config.charge_pump_freq = (uint16_t)atoi(val);
        /* Misc */
        else if (strcmp(key, "shared_limits") == 0)
            g_config.shared_limits = atoi(val);
        /* Axis cloning: clone_A_of = Y, clone_A_reversed = 1 */
        else if (strncmp(key, "clone_", 6) == 0 && strlen(key) >= 10) {
            char axis_ch = key[6];
            int axis_idx = -1;
            for (int i = 0; i < 6; i++) {
                if (axis_names[i] == axis_ch) { axis_idx = i; break; }
            }
            if (axis_idx >= 0 && strstr(key, "_of") != NULL) {
                /* Value is axis name or -1/none */
                if (strcmp(val, "none") == 0 || strcmp(val, "-1") == 0) {
                    g_config.clone_master[axis_idx] = -1;
                } else {
                    for (int i = 0; i < 6; i++) {
                        if (axis_names[i] == val[0]) {
                            g_config.clone_master[axis_idx] = i;
                            break;
                        }
                    }
                }
            }
            else if (axis_idx >= 0 && strstr(key, "_reversed") != NULL) {
                g_config.clone_reversed[axis_idx] = atoi(val);
            }
        }
        /* I/O expansion module */
        else if (strcmp(key, "io_module_ip") == 0 && strcmp(val, "none") != 0) {
            strncpy(io_mod_ip, val, sizeof(io_mod_ip) - 1);
            io_mod_enabled = 1;
        }
        else if (strcmp(key, "io_module_enabled") == 0)
            io_mod_enabled = atoi(val);
    }

    fclose(f);
    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: Loaded config from %s\n", COMP_NAME, path);
}

/* ===================================================================
 * Signal Handler
 * =================================================================== */

static void quit(int sig)
{
    (void)sig;
    done = 1;
}

/* ===================================================================
 * Network Helpers
 * =================================================================== */

static uint32_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
}

static uint32_t get_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void build_header(wcnc_header_t *hdr, uint8_t type, uint16_t payload_len)
{
    hdr->magic = WCNC_MAGIC;
    hdr->version = WCNC_PROTOCOL_VERSION;
    hdr->packet_type = type;
    hdr->payload_length = payload_len;
    hdr->sequence = ++tx_sequence;
    hdr->timestamp_us = get_timestamp_us();
    hdr->checksum = 0;
}

static int tcp_connect(const char *ip, uint16_t port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &addr.sin_addr);

    /* Set connect timeout */
    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

static int udp_create_send(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    memset(&esp_addr, 0, sizeof(esp_addr));
    esp_addr.sin_family = AF_INET;
    esp_addr.sin_port = htons(WCNC_UDP_MOTION_PORT);
    inet_pton(AF_INET, esp_ip, &esp_addr.sin_addr);

    return sock;
}

static int udp_create_recv(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(WCNC_UDP_STATUS_PORT);
    bind_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        close(sock);
        return -1;
    }

    /* Non-blocking */
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    return sock;
}

/* TCP with 2-byte LE length prefix (matches ESP32 tcp_server.c framing) */
static int tcp_send_packet(const void *pkt, size_t len)
{
    if (tcp_sock < 0) return -1;

    uint8_t len_buf[2] = {
        (uint8_t)(len & 0xFF),
        (uint8_t)((len >> 8) & 0xFF)
    };

    if (send(tcp_sock, len_buf, 2, MSG_NOSIGNAL) != 2)
        return -1;

    size_t total_sent = 0;
    while (total_sent < len) {
        ssize_t n = send(tcp_sock, (const uint8_t *)pkt + total_sent,
                         len - total_sent, MSG_NOSIGNAL);
        if (n <= 0) return -1;
        total_sent += (size_t)n;
    }
    return 0;
}

static int tcp_recv_packet(void *buf, size_t bufsize, size_t *out_len)
{
    if (tcp_sock < 0) return -1;

    /* Read 2-byte LE length prefix */
    uint8_t len_buf[2];
    int received = 0;
    while (received < 2) {
        ssize_t n = recv(tcp_sock, len_buf + received, 2 - received, 0);
        if (n <= 0) return -1;
        received += (int)n;
    }

    uint16_t pkt_len = (uint16_t)len_buf[0] | ((uint16_t)len_buf[1] << 8);
    if (pkt_len == 0 || pkt_len > bufsize) return -1;

    /* Read full packet body */
    received = 0;
    while (received < (int)pkt_len) {
        ssize_t n = recv(tcp_sock, (uint8_t *)buf + received,
                         pkt_len - received, 0);
        if (n <= 0) return -1;
        received += (int)n;
    }

    if (out_len) *out_len = (size_t)pkt_len;
    return 0;
}

static int udp_send_packet(const void *pkt, size_t len)
{
    if (udp_send_sock < 0) return -1;
    ssize_t sent = sendto(udp_send_sock, pkt, len, 0,
                           (struct sockaddr *)&esp_addr, sizeof(esp_addr));
    return (sent == (ssize_t)len) ? 0 : -1;
}

/* ===================================================================
 * Auto-Discovery via UDP Broadcast
 * =================================================================== */

static int discover_esp32(char *out_ip, size_t out_ip_size, int timeout_sec)
{
    wcnc_jog_stop_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.axis = 0xFF;
    build_header(&pkt.header, WCNC_PKT_JOG_STOP,
                 sizeof(pkt) - sizeof(wcnc_header_t));
    wcnc_finalize_packet(&pkt, WCNC_PKT_JOG_STOP,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          pkt.header.sequence, pkt.header.timestamp_us);

    int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock < 0) return -1;

    int bcast = 1;
    setsockopt(tx_sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    struct sockaddr_in bcast_addr;
    memset(&bcast_addr, 0, sizeof(bcast_addr));
    bcast_addr.sin_family = AF_INET;
    bcast_addr.sin_port = htons(WCNC_UDP_MOTION_PORT);
    bcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock < 0) { close(tx_sock); return -1; }

    int reuse = 1;
    setsockopt(rx_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(WCNC_UDP_STATUS_PORT);
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind(rx_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(rx_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int found = 0;
    time_t start = time(NULL);

    while (!found && (time(NULL) - start) < timeout_sec) {
        sendto(tx_sock, &pkt, sizeof(pkt), 0,
               (struct sockaddr *)&bcast_addr, sizeof(bcast_addr));

        uint8_t buf[256];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        ssize_t n = recvfrom(rx_sock, buf, sizeof(buf), 0,
                              (struct sockaddr *)&from_addr, &from_len);

        if (n >= (ssize_t)sizeof(wcnc_header_t)) {
            const wcnc_header_t *hdr = (const wcnc_header_t *)buf;
            if (hdr->magic == WCNC_MAGIC &&
                hdr->packet_type == WCNC_PKT_STATUS_REPORT) {
                inet_ntop(AF_INET, &from_addr.sin_addr,
                           out_ip, (socklen_t)out_ip_size);
                found = 1;
            }
        }
    }

    close(tx_sock);
    close(rx_sock);
    return found ? 0 : -1;
}

/* ===================================================================
 * Protocol: Handshake
 * =================================================================== */

static int do_handshake(void)
{
    wcnc_handshake_req_t req;
    memset(&req, 0, sizeof(req));
    build_header(&req.header, WCNC_PKT_HANDSHAKE_REQ,
                 sizeof(req) - sizeof(wcnc_header_t));
    req.host_version = WCNC_FIRMWARE_VERSION;
    snprintf(req.host_name, WCNC_DEVICE_NAME_LEN, "LinuxCNC WiFi CNC");
    wcnc_finalize_packet(&req, WCNC_PKT_HANDSHAKE_REQ,
                          sizeof(req) - sizeof(wcnc_header_t),
                          req.header.sequence, req.header.timestamp_us);

    if (tcp_send_packet(&req, sizeof(req)) < 0) return -1;

    wcnc_handshake_resp_t resp;
    size_t resp_len;
    if (tcp_recv_packet(&resp, sizeof(resp), &resp_len) < 0) return -1;

    if (resp.header.packet_type != WCNC_PKT_HANDSHAKE_RESP) return -1;

    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: Connected to %s (firmware v%d.%d.%d, %d axes, buffer=%d)\n",
        COMP_NAME, resp.device_name,
        (resp.firmware_version >> 24) & 0xFF,
        (resp.firmware_version >> 16) & 0xFF,
        resp.firmware_version & 0xFFFF,
        resp.num_axes, resp.buffer_capacity);

    return 0;
}

/* ===================================================================
 * Protocol: Send Config
 * =================================================================== */

static int send_config(uint16_t key, const void *value, uint16_t value_type)
{
    wcnc_config_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_CONFIG_SET,
                 sizeof(pkt) - sizeof(wcnc_header_t));
    pkt.key = key;
    pkt.value_type = value_type;

    size_t val_size = 0;
    switch (value_type) {
        case WCNC_VAL_UINT8:  val_size = 1; break;
        case WCNC_VAL_UINT16: val_size = 2; break;
        case WCNC_VAL_UINT32: val_size = 4; break;
        case WCNC_VAL_INT32:  val_size = 4; break;
        case WCNC_VAL_FLOAT:  val_size = 4; break;
        case WCNC_VAL_STRING:
            val_size = strlen((const char *)value) + 1;
            if (val_size > WCNC_CONFIG_VALUE_LEN)
                val_size = WCNC_CONFIG_VALUE_LEN;
            break;
        default: return -1;
    }
    memcpy(pkt.value, value, val_size);

    wcnc_finalize_packet(&pkt, WCNC_PKT_CONFIG_SET,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          pkt.header.sequence, pkt.header.timestamp_us);

    return tcp_send_packet(&pkt, sizeof(pkt));
}

static int send_config_save(void)
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_CONFIG_SAVE, 0);
    wcnc_finalize_packet(&pkt, WCNC_PKT_CONFIG_SAVE, 0,
                          pkt.header.sequence, pkt.header.timestamp_us);
    return tcp_send_packet(&pkt, sizeof(pkt));
}

/* ===================================================================
 * Protocol: Full Config Sync to ESP32
 * (Matches Mach3 plugin's SyncConfigToESP32)
 * =================================================================== */

static void sync_config(wifi_cnc_data_t *data)
{
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Syncing config to ESP32...\n", COMP_NAME);

    /* Timing */
    send_config(WCNC_CFG_STEP_PULSE_US, &g_config.step_pulse_us, WCNC_VAL_UINT16);
    usleep(5000);
    send_config(WCNC_CFG_DIR_SETUP_US, &g_config.dir_setup_us, WCNC_VAL_UINT16);
    usleep(5000);

    /* Inversion masks (combine limit+home like Mach3 plugin — they share GPIO) */
    send_config(WCNC_CFG_INVERT_STEP, &g_config.invert_step, WCNC_VAL_UINT8);
    usleep(5000);
    send_config(WCNC_CFG_INVERT_DIR, &g_config.invert_dir, WCNC_VAL_UINT8);
    usleep(5000);

    uint8_t combined_invert = g_config.invert_limit | g_config.invert_home;
    send_config(WCNC_CFG_INVERT_LIMIT, &combined_invert, WCNC_VAL_UINT8);
    usleep(5000);
    send_config(WCNC_CFG_INVERT_HOME, &combined_invert, WCNC_VAL_UINT8);
    usleep(5000);

    send_config(WCNC_CFG_INVERT_ESTOP, &g_config.invert_estop, WCNC_VAL_UINT8);
    usleep(5000);
    send_config(WCNC_CFG_INVERT_PROBE, &g_config.invert_probe, WCNC_VAL_UINT8);
    usleep(5000);

    /* Per-axis */
    for (int i = 0; i < num_joints; i++) {
        float steps_per_mm = (float)data->scale[i];
        send_config(WCNC_CFG_STEPS_PER_MM_X + i, &steps_per_mm, WCNC_VAL_FLOAT);

        uint32_t max_rate = (uint32_t)(data->maxvel[i] * fabs(data->scale[i]));
        send_config(WCNC_CFG_MAX_RATE_X + i, &max_rate, WCNC_VAL_UINT32);

        uint32_t accel = (uint32_t)(data->maxaccel[i] * fabs(data->scale[i]));
        send_config(WCNC_CFG_ACCEL_X + i, &accel, WCNC_VAL_UINT32);
        usleep(5000);
    }

    /* Homing */
    send_config(WCNC_CFG_HOMING_DIR_MASK, &g_config.homing_dir_mask, WCNC_VAL_UINT8);
    send_config(WCNC_CFG_HOMING_SEEK_RATE, &g_config.homing_seek_rate, WCNC_VAL_UINT32);
    send_config(WCNC_CFG_HOMING_FEED_RATE, &g_config.homing_feed_rate, WCNC_VAL_UINT32);
    send_config(WCNC_CFG_HOMING_PULLOFF, &g_config.homing_pulloff, WCNC_VAL_UINT32);
    usleep(5000);

    /* Spindle */
    send_config(WCNC_CFG_SPINDLE_PWM_FREQ, &g_config.spindle_pwm_freq, WCNC_VAL_UINT16);
    send_config(WCNC_CFG_SPINDLE_MAX_RPM, &g_config.spindle_max_rpm, WCNC_VAL_UINT32);
    usleep(5000);

    /* Charge pump (initial config — dynamic on/off handled in main loop) */
    send_config(WCNC_CFG_CHARGE_PUMP_FREQ, &g_config.charge_pump_freq, WCNC_VAL_UINT16);

    /* Step idle delay */
    send_config(WCNC_CFG_STEP_IDLE_DELAY_MS, &g_config.step_idle_delay_ms, WCNC_VAL_UINT16);
    usleep(5000);

    send_config_save();
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Config sync complete\n", COMP_NAME);
}

/* ===================================================================
 * Protocol: Send Control Commands
 * =================================================================== */

static void send_estop(void)
{
    wcnc_estop_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_ESTOP, 0);
    wcnc_finalize_packet(&pkt, WCNC_PKT_ESTOP, 0,
                          pkt.header.sequence, pkt.header.timestamp_us);
    /* Send 3x for redundancy (UDP is unreliable) */
    udp_send_packet(&pkt, sizeof(pkt));
    udp_send_packet(&pkt, sizeof(pkt));
    udp_send_packet(&pkt, sizeof(pkt));
}

static void send_reset(void)
{
    wcnc_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_RESET, 0);
    wcnc_finalize_packet(&pkt, WCNC_PKT_RESET, 0,
                          pkt.header.sequence, pkt.header.timestamp_us);
    udp_send_packet(&pkt, sizeof(pkt));
}

/* ===================================================================
 * Status Receive Thread
 * =================================================================== */

static void *status_receive_thread(void *arg)
{
    (void)arg;
    uint8_t buf[512];

    while (status_thread_running) {
        ssize_t n = recv(udp_recv_sock, buf, sizeof(buf), 0);
        if (n > 0 && (size_t)n >= sizeof(wcnc_header_t)) {
            wcnc_header_t *hdr = (wcnc_header_t *)buf;
            if (hdr->magic == WCNC_MAGIC &&
                hdr->packet_type == WCNC_PKT_STATUS_REPORT &&
                (size_t)n >= sizeof(wcnc_header_t) + 46) {
                pthread_mutex_lock(&status_lock);
                memset(&latest_status, 0, sizeof(latest_status));
                size_t copy_len = (size_t)n - sizeof(wcnc_header_t);
                if (copy_len > sizeof(wcnc_status_report_t))
                    copy_len = sizeof(wcnc_status_report_t);
                memcpy(&latest_status, buf + sizeof(wcnc_header_t), copy_len);
                pthread_mutex_unlock(&status_lock);
            }
        }
        usleep(1000);
    }
    return NULL;
}

/* ===================================================================
 * Connection Management
 * =================================================================== */

static int establish_connection(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: Connecting to ESP32 at %s...\n", COMP_NAME, esp_ip);

    tcp_sock = tcp_connect(esp_ip, WCNC_TCP_CONTROL_PORT);
    if (tcp_sock < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: TCP connect failed\n", COMP_NAME);
        return -1;
    }

    if (do_handshake() < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: Handshake failed\n", COMP_NAME);
        close(tcp_sock);
        tcp_sock = -1;
        return -1;
    }

    udp_send_sock = udp_create_send();
    udp_recv_sock = udp_create_recv();
    if (udp_send_sock < 0 || udp_recv_sock < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: UDP socket creation failed\n", COMP_NAME);
        close(tcp_sock);
        tcp_sock = -1;
        return -1;
    }

    /* Send UDP registration so firmware knows our address */
    {
        wcnc_jog_stop_packet_t reg;
        memset(&reg, 0, sizeof(reg));
        reg.axis = 0xFF;
        build_header(&reg.header, WCNC_PKT_JOG_STOP,
                     sizeof(reg) - sizeof(wcnc_header_t));
        wcnc_finalize_packet(&reg, WCNC_PKT_JOG_STOP,
                              sizeof(reg) - sizeof(wcnc_header_t),
                              reg.header.sequence, reg.header.timestamp_us);
        udp_send_packet(&reg, sizeof(reg));
    }

    /* Start status receive thread */
    status_thread_running = 1;
    pthread_create(&status_thread, NULL, status_receive_thread, NULL);

    esp_connected = 1;
    return 0;
}

static void close_connection(void)
{
    esp_connected = 0;
    status_thread_running = 0;
    if (status_thread) {
        pthread_join(status_thread, NULL);
        status_thread = 0;
    }
    if (tcp_sock >= 0) { close(tcp_sock); tcp_sock = -1; }
    if (udp_send_sock >= 0) { close(udp_send_sock); udp_send_sock = -1; }
    if (udp_recv_sock >= 0) { close(udp_recv_sock); udp_recv_sock = -1; }

    /* Reset position tracking so offsets recalculate on reconnect */
    pos_initialized = 0;
    prev_charge_pump = -1;
}

/* ===================================================================
 * I/O Module Connection
 * =================================================================== */

static void *io_mod_status_receive_thread(void *arg)
{
    (void)arg;
    uint8_t buf[512];

    while (io_mod_thread_running) {
        ssize_t n = recv(io_mod_udp_recv_sock, buf, sizeof(buf), 0);
        if (n > 0 && (size_t)n >= sizeof(wcnc_header_t)) {
            wcnc_header_t *hdr = (wcnc_header_t *)buf;
            if (hdr->magic == WCNC_MAGIC &&
                hdr->packet_type == WCNC_PKT_STATUS_REPORT &&
                (size_t)n >= sizeof(wcnc_header_t) + 46) {
                pthread_mutex_lock(&io_mod_lock);
                memset(&io_mod_status, 0, sizeof(io_mod_status));
                size_t copy_len = (size_t)n - sizeof(wcnc_header_t);
                if (copy_len > sizeof(wcnc_status_report_t))
                    copy_len = sizeof(wcnc_status_report_t);
                memcpy(&io_mod_status, buf + sizeof(wcnc_header_t), copy_len);
                pthread_mutex_unlock(&io_mod_lock);
            }
        }
        usleep(1000);
    }
    return NULL;
}

static int io_mod_establish_connection(void)
{
    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: I/O Module: connecting to %s...\n", COMP_NAME, io_mod_ip);

    io_mod_tcp_sock = tcp_connect(io_mod_ip, WCNC_TCP_CONTROL_PORT);
    if (io_mod_tcp_sock < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: I/O Module: TCP connect failed\n", COMP_NAME);
        return -1;
    }

    /* Handshake with I/O module (reuse build_header but use io_mod sockets) */
    {
        wcnc_handshake_req_t req;
        memset(&req, 0, sizeof(req));
        build_header(&req.header, WCNC_PKT_HANDSHAKE_REQ,
                     sizeof(req) - sizeof(wcnc_header_t));
        req.host_version = WCNC_FIRMWARE_VERSION;
        snprintf(req.host_name, WCNC_DEVICE_NAME_LEN, "LinuxCNC IO Module");
        wcnc_finalize_packet(&req, WCNC_PKT_HANDSHAKE_REQ,
                              sizeof(req) - sizeof(wcnc_header_t),
                              req.header.sequence, req.header.timestamp_us);

        /* TCP send via io_mod_tcp_sock */
        uint8_t lenBuf[2] = { sizeof(req) & 0xFF, (sizeof(req) >> 8) & 0xFF };
        if (send(io_mod_tcp_sock, lenBuf, 2, 0) != 2 ||
            send(io_mod_tcp_sock, &req, sizeof(req), 0) != sizeof(req)) {
            close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
            return -1;
        }

        /* TCP recv response */
        uint8_t rlenBuf[2];
        if (recv(io_mod_tcp_sock, rlenBuf, 2, MSG_WAITALL) != 2) {
            close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
            return -1;
        }
        uint16_t rlen = rlenBuf[0] | (rlenBuf[1] << 8);
        wcnc_handshake_resp_t resp;
        if (rlen > sizeof(resp) || recv(io_mod_tcp_sock, &resp, rlen, MSG_WAITALL) != rlen) {
            close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
            return -1;
        }
        if (resp.header.packet_type != WCNC_PKT_HANDSHAKE_RESP) {
            close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
            return -1;
        }
        if (!(resp.capabilities & WCNC_CAP_IO_MODULE)) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                "%s: I/O Module: device is not in I/O module mode\n", COMP_NAME);
            close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO,
            "%s: I/O Module: handshake OK (%s, %d channels)\n",
            COMP_NAME, resp.device_name, resp.io_channel_count);
    }

    /* Create I/O module UDP sockets (different port pair for status) */
    io_mod_udp_send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    io_mod_udp_recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (io_mod_udp_send_sock < 0 || io_mod_udp_recv_sock < 0) {
        close(io_mod_tcp_sock); io_mod_tcp_sock = -1;
        return -1;
    }

    /* Bind recv to a different local port (58430) to avoid conflict */
    {
        int reuse = 1;
        setsockopt(io_mod_udp_recv_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(WCNC_UDP_STATUS_PORT + 2);
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind(io_mod_udp_recv_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

        struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
        setsockopt(io_mod_udp_recv_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    /* Set I/O module address */
    memset(&io_mod_addr, 0, sizeof(io_mod_addr));
    io_mod_addr.sin_family = AF_INET;
    io_mod_addr.sin_port = htons(WCNC_UDP_MOTION_PORT);
    inet_pton(AF_INET, io_mod_ip, &io_mod_addr.sin_addr);

    /* Send UDP registration */
    {
        wcnc_jog_stop_packet_t reg;
        memset(&reg, 0, sizeof(reg));
        reg.axis = 0xFF;
        build_header(&reg.header, WCNC_PKT_JOG_STOP,
                     sizeof(reg) - sizeof(wcnc_header_t));
        wcnc_finalize_packet(&reg, WCNC_PKT_JOG_STOP,
                              sizeof(reg) - sizeof(wcnc_header_t),
                              reg.header.sequence, reg.header.timestamp_us);
        sendto(io_mod_udp_send_sock, &reg, sizeof(reg), 0,
               (struct sockaddr *)&io_mod_addr, sizeof(io_mod_addr));
    }

    /* Start I/O module status receive thread */
    io_mod_thread_running = 1;
    pthread_create(&io_mod_status_thread, NULL, io_mod_status_receive_thread, NULL);

    io_mod_connected = 1;
    return 0;
}

static void io_mod_close_connection(void)
{
    io_mod_connected = 0;
    io_mod_thread_running = 0;
    if (io_mod_status_thread) {
        pthread_join(io_mod_status_thread, NULL);
        io_mod_status_thread = 0;
    }
    if (io_mod_tcp_sock >= 0) { close(io_mod_tcp_sock); io_mod_tcp_sock = -1; }
    if (io_mod_udp_send_sock >= 0) { close(io_mod_udp_send_sock); io_mod_udp_send_sock = -1; }
    if (io_mod_udp_recv_sock >= 0) { close(io_mod_udp_recv_sock); io_mod_udp_recv_sock = -1; }
}

/* ===================================================================
 * Motion: Position-to-Step Conversion and Segment Sending
 * =================================================================== */

static void update_motion(wifi_cnc_data_t *data, double dt)
{
    if (!esp_connected) return;

    wcnc_motion_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_MOTION_SEGMENT,
                 sizeof(pkt) - sizeof(wcnc_header_t));
    pkt.segment_count = 1;

    wcnc_motion_segment_t *seg = &pkt.segments[0];
    uint32_t duration_us = (uint32_t)(dt * 1000000.0);
    seg->duration_us = duration_us;
    seg->flags = 0;

    /* Set probe flag if probing is active */
    if (*(data->probe_active))
        seg->flags |= WCNC_SEG_FLAG_PROBE;

    int has_motion = 0;

    for (int i = 0; i < num_joints; i++) {
        if (!*(data->joint_enable[i])) continue;

        double cmd = *(data->pos_cmd[i]);
        double scale = data->scale[i];
        double delta = cmd - prev_pos_cmd[i];

        /* Acceleration limiting */
        double vel = delta / dt;
        double max_dv = data->maxaccel[i] * dt;
        double dv = vel - prev_vel[i];
        if (dv > max_dv) vel = prev_vel[i] + max_dv;
        else if (dv < -max_dv) vel = prev_vel[i] - max_dv;

        /* Velocity limiting */
        if (vel > data->maxvel[i]) vel = data->maxvel[i];
        if (vel < -data->maxvel[i]) vel = -data->maxvel[i];

        /* Convert to steps */
        int32_t steps = (int32_t)(vel * dt * scale);

        seg->steps[i] = steps;
        accumulated_steps[i] += steps;
        prev_vel[i] = vel;
        prev_pos_cmd[i] = cmd;

        *(data->freq_cmd[i]) = vel * scale;
        *(data->counts[i]) = accumulated_steps[i];

        if (steps != 0) has_motion = 1;
    }

    /* Clone steps to slave axes */
    for (int i = 0; i < num_joints; i++) {
        int master = g_config.clone_master[i];
        if (master >= 0 && master < num_joints) {
            int32_t steps = seg->steps[master];
            if (g_config.clone_reversed[i]) steps = -steps;
            seg->steps[i] = steps;
            accumulated_steps[i] += steps;
            *(data->counts[i]) = accumulated_steps[i];
            if (steps != 0) has_motion = 1;
        }
    }

    if (has_motion) {
        wcnc_finalize_packet(&pkt, WCNC_PKT_MOTION_SEGMENT,
                              sizeof(pkt) - sizeof(wcnc_header_t),
                              pkt.header.sequence, pkt.header.timestamp_us);
        udp_send_packet(&pkt, sizeof(pkt));
    }
}

/* ===================================================================
 * I/O Control: Send Misc/Spindle/Coolant State to ESP32
 * =================================================================== */

static void update_io_control(wifi_cnc_data_t *data)
{
    if (!esp_connected) return;

    wcnc_io_control_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    build_header(&pkt.header, WCNC_PKT_IO_CONTROL,
                 sizeof(pkt) - sizeof(wcnc_header_t));

    /* Misc outputs */
    uint8_t misc_out = 0;
    for (int i = 0; i < 5; i++) {
        if (*(data->misc_out[i]))
            misc_out |= (1 << i);
    }
    pkt.misc_outputs = misc_out;

    /* Spindle state */
    if (*(data->spindle_on)) {
        pkt.spindle_state = *(data->spindle_cw) ? 1 : 2;
    }

    uint16_t rpm = (uint16_t)fabs(*(data->spindle_speed));

    /* Laser mode: scale spindle power with feed rate.
     * During rapids (no motion or very fast), power goes to 0.
     * During cutting, power scales with actual vs commanded speed. */
    if (g_config.laser_mode && *(data->spindle_on)) {
        /* Read current feed rate from ESP32 status */
        wcnc_status_report_t status;
        pthread_mutex_lock(&status_lock);
        memcpy(&status, &latest_status, sizeof(status));
        pthread_mutex_unlock(&status_lock);

        /* If feed rate is near zero (e.g. rapid or stopped), turn off laser */
        double actual_feed = fabs((double)status.feed_rate);
        if (actual_feed < 1.0) {
            rpm = 0;
        }
        /* Otherwise keep commanded RPM — LinuxCNC handles the S-word scaling */
    }

    pkt.spindle_rpm = rpm;

    /* Coolant */
    uint8_t coolant = 0;
    if (*(data->coolant_flood)) coolant |= 0x01;
    if (*(data->coolant_mist))  coolant |= 0x02;
    pkt.coolant_state = coolant;

    wcnc_finalize_packet(&pkt, WCNC_PKT_IO_CONTROL,
                          sizeof(pkt) - sizeof(wcnc_header_t),
                          pkt.header.sequence, pkt.header.timestamp_us);
    udp_send_packet(&pkt, sizeof(pkt));
}

/* ===================================================================
 * Charge Pump Control
 * =================================================================== */

static void update_charge_pump(wifi_cnc_data_t *data)
{
    if (!esp_connected) return;

    int cp = *(data->charge_pump) ? 1 : 0;
    if (cp != prev_charge_pump) {
        prev_charge_pump = cp;
        uint16_t freq = cp ? g_config.charge_pump_freq : 0;
        send_config(WCNC_CFG_CHARGE_PUMP_FREQ, &freq, WCNC_VAL_UINT16);
        rtapi_print_msg(RTAPI_MSG_INFO,
            "%s: Charge pump %s (freq=%u Hz)\n",
            COMP_NAME, cp ? "ON" : "OFF", freq);
    }
}

/* ===================================================================
 * Status Feedback: Update HAL Pins from ESP32 Status
 * =================================================================== */

static void update_feedback(wifi_cnc_data_t *data)
{
    wcnc_status_report_t status;
    pthread_mutex_lock(&status_lock);
    memcpy(&status, &latest_status, sizeof(status));
    pthread_mutex_unlock(&status_lock);

    for (int i = 0; i < num_joints; i++) {
        double scale = data->scale[i];
        if (scale == 0.0) scale = 1.0;

        /* Position offset: on first status, align ESP32 position with
         * LinuxCNC's current commanded position so DRO doesn't jump. */
        if (!pos_initialized) {
            g_pos_offset[i] = (int32_t)(*(data->pos_cmd[i]) * scale)
                              - status.position_steps[i];
        }

        int32_t corrected_steps = status.position_steps[i] + g_pos_offset[i];
        *(data->pos_fb[i]) = (double)corrected_steps / scale;
        *(data->counts[i]) = corrected_steps;

        /* Limit switches */
        int lim_bit = (status.limit_switches >> i) & 1;
        *(data->neg_lim[i]) = lim_bit;
        *(data->pos_lim[i]) = data->shared_limits ? lim_bit : 0;

        /* Home switches */
        *(data->home_sw[i]) = (status.home_switches >> i) & 1;
    }

    if (!pos_initialized) {
        pos_initialized = 1;
        rtapi_print_msg(RTAPI_MSG_INFO,
            "%s: Position offsets initialized\n", COMP_NAME);
    }

    /* Global status */
    *(data->connected) = esp_connected ? 1 : 0;
    *(data->estop_out) = status.estop_input ? 1 :
                         (status.state == WCNC_STATE_ESTOP ||
                          status.state == WCNC_STATE_ALARM) ? 1 : 0;
    *(data->probe) = status.probe_state ? 1 : 0;
    *(data->feed_rate) = (double)status.feed_rate;

    for (int i = 0; i < 5; i++) {
        *(data->misc_in[i]) = (status.misc_inputs >> i) & 1;
    }

    /* Spindle encoder feedback (v1.1) */
    *(data->spindle_rpm_fb) = (double)status.spindle_rpm;
    *(data->spindle_speed_fb) = (double)status.spindle_rpm / 60.0;  /* RPS */

    /* Cumulative revolutions for threading (from index count) */
    static uint32_t prev_index_count = 0;
    static double cumulative_revs = 0.0;
    if (status.spindle_index_count != prev_index_count) {
        cumulative_revs += (double)(status.spindle_index_count - prev_index_count);
        prev_index_count = status.spindle_index_count;

        /* index-enable: reset to 0 on each index pulse */
        if (*(data->spindle_index_enable)) {
            *(data->spindle_index_enable) = 0;
        }
    }
    /* Add fractional revolution from position (0-65535 = 0-1 rev) */
    *(data->spindle_revs) = cumulative_revs
                            + (double)status.spindle_position / 65536.0;

    /* I/O module feedback */
    *(data->io_mod_connected) = io_mod_connected ? 1 : 0;
    if (io_mod_connected) {
        wcnc_status_report_t io_status;
        pthread_mutex_lock(&io_mod_lock);
        memcpy(&io_status, &io_mod_status, sizeof(io_status));
        pthread_mutex_unlock(&io_mod_lock);

        for (int i = 0; i < 16; i++) {
            *(data->io_mod_in[i]) = (io_status.io_inputs >> i) & 1;
        }
    }
}

/* ===================================================================
 * Usage / Help
 * =================================================================== */

static void print_usage(void)
{
    printf("Usage: wifi_cnc [options]\n"
           "  --ip=ADDR       ESP32 IP address (default: auto-discover)\n"
           "                  Use 'auto' or omit for auto-discovery\n"
           "  --joints=N      Number of joints (default: 3, max: %d)\n"
           "  --config=PATH   Config file path (wifi_cnc.conf)\n"
           "  --help          Show this help\n",
           MAX_JOINTS);
}

/* ===================================================================
 * Main
 * =================================================================== */

int main(int argc, char **argv)
{
    /* Parse command line */
    static struct option long_opts[] = {
        { "ip",     required_argument, NULL, 'i' },
        { "joints", required_argument, NULL, 'j' },
        { "config", required_argument, NULL, 'c' },
        { "help",   no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "i:j:c:h", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'i':
            if (strcmp(optarg, "auto") == 0) {
                ip_specified = 0;
            } else {
                strncpy(esp_ip, optarg, sizeof(esp_ip) - 1);
                ip_specified = 1;
            }
            break;
        case 'j':
            num_joints = atoi(optarg);
            if (num_joints < 1) num_joints = 1;
            if (num_joints > MAX_JOINTS) num_joints = MAX_JOINTS;
            break;
        case 'c':
            strncpy(config_path, optarg, sizeof(config_path) - 1);
            break;
        case 'h':
            print_usage();
            return 0;
        default:
            print_usage();
            return 1;
        }
    }

    /* Load config file (may override ip and num_joints) */
    if (config_path[0]) {
        parse_config_file(config_path);
    }

    /* Signal handlers */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Initialize HAL */
    comp_id = hal_init(COMP_NAME);
    if (comp_id < 0) {
        fprintf(stderr, "%s: hal_init() failed\n", COMP_NAME);
        return 1;
    }

    /* Allocate HAL shared memory */
    wifi_cnc_data_t *data = hal_malloc(sizeof(wifi_cnc_data_t));
    if (!data) {
        fprintf(stderr, "%s: hal_malloc() failed\n", COMP_NAME);
        hal_exit(comp_id);
        return 1;
    }
    memset(data, 0, sizeof(*data));

    /* Create per-joint pins and parameters */
    int r;
    for (int i = 0; i < num_joints; i++) {
        r = hal_pin_float_newf(HAL_IN, &data->pos_cmd[i], comp_id,
                                "%s.joint.%d.pos-cmd", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_float_newf(HAL_OUT, &data->pos_fb[i], comp_id,
                                "%s.joint.%d.pos-fb", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_s32_newf(HAL_OUT, &data->counts[i], comp_id,
                              "%s.joint.%d.counts", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_float_newf(HAL_OUT, &data->freq_cmd[i], comp_id,
                                "%s.joint.%d.freq-cmd", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_IN, &data->joint_enable[i], comp_id,
                              "%s.joint.%d.enable", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_OUT, &data->neg_lim[i], comp_id,
                              "%s.joint.%d.neg-lim", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_OUT, &data->pos_lim[i], comp_id,
                              "%s.joint.%d.pos-lim", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_OUT, &data->home_sw[i], comp_id,
                              "%s.joint.%d.home-sw", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_param_float_newf(HAL_RW, &data->scale[i], comp_id,
                                  "%s.joint.%d.scale", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_param_float_newf(HAL_RW, &data->maxvel[i], comp_id,
                                  "%s.joint.%d.maxvel", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_param_float_newf(HAL_RW, &data->maxaccel[i], comp_id,
                                  "%s.joint.%d.maxaccel", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        data->scale[i] = 200.0;
        data->maxvel[i] = 50.0;
        data->maxaccel[i] = 500.0;
    }

    /* Global pins */
    r = hal_pin_bit_newf(HAL_IN, &data->enable, comp_id,
                          "%s.enable", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->estop_in, comp_id,
                          "%s.estop-in", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_OUT, &data->connected, comp_id,
                          "%s.connected", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_OUT, &data->estop_out, comp_id,
                          "%s.estop-out", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_OUT, &data->probe, comp_id,
                          "%s.probe", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_float_newf(HAL_OUT, &data->feed_rate, comp_id,
                            "%s.feed-rate", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->charge_pump, comp_id,
                          "%s.charge-pump", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->probe_active, comp_id,
                          "%s.probe-active", COMP_NAME);
    if (r < 0) goto pin_fail;

    /* Misc I/O pins */
    for (int i = 0; i < 5; i++) {
        r = hal_pin_bit_newf(HAL_IN, &data->misc_out[i], comp_id,
                              "%s.misc-out.%d", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_OUT, &data->misc_in[i], comp_id,
                              "%s.misc-in.%d", COMP_NAME, i);
        if (r < 0) goto pin_fail;
    }

    /* Spindle / Coolant pins */
    r = hal_pin_bit_newf(HAL_IN, &data->spindle_on, comp_id,
                          "%s.spindle-on", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->spindle_cw, comp_id,
                          "%s.spindle-cw", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_float_newf(HAL_IN, &data->spindle_speed, comp_id,
                            "%s.spindle-speed", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->coolant_flood, comp_id,
                          "%s.coolant-flood", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IN, &data->coolant_mist, comp_id,
                          "%s.coolant-mist", COMP_NAME);
    if (r < 0) goto pin_fail;

    /* Spindle encoder feedback pins */
    r = hal_pin_float_newf(HAL_OUT, &data->spindle_revs, comp_id,
                            "%s.spindle-revs", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_float_newf(HAL_OUT, &data->spindle_speed_fb, comp_id,
                            "%s.spindle-speed-fb", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_bit_newf(HAL_IO, &data->spindle_index_enable, comp_id,
                          "%s.spindle-index-enable", COMP_NAME);
    if (r < 0) goto pin_fail;

    r = hal_pin_float_newf(HAL_OUT, &data->spindle_rpm_fb, comp_id,
                            "%s.spindle-rpm-fb", COMP_NAME);
    if (r < 0) goto pin_fail;

    /* I/O module pins */
    for (int i = 0; i < 16; i++) {
        r = hal_pin_bit_newf(HAL_OUT, &data->io_mod_in[i], comp_id,
                              "%s.io-module.in-%02d", COMP_NAME, i);
        if (r < 0) goto pin_fail;

        r = hal_pin_bit_newf(HAL_IN, &data->io_mod_out[i], comp_id,
                              "%s.io-module.out-%02d", COMP_NAME, i);
        if (r < 0) goto pin_fail;
    }

    r = hal_pin_bit_newf(HAL_OUT, &data->io_mod_connected, comp_id,
                          "%s.io-module.connected", COMP_NAME);
    if (r < 0) goto pin_fail;

    /* shared_limits parameter */
    r = hal_param_bit_newf(HAL_RW, &data->shared_limits, comp_id,
                            "%s.shared-limits", COMP_NAME);
    if (r < 0) goto pin_fail;
    data->shared_limits = g_config.shared_limits ? 1 : 0;

    /* Signal ready */
    hal_ready(comp_id);

    /* Auto-discover ESP32 if no IP was specified */
    if (!ip_specified) {
        rtapi_print_msg(RTAPI_MSG_INFO,
            "%s: Auto-discovering ESP32 on network...\n", COMP_NAME);
        char discovered_ip[64] = {0};
        if (discover_esp32(discovered_ip, sizeof(discovered_ip), 5) == 0) {
            strncpy(esp_ip, discovered_ip, sizeof(esp_ip) - 1);
            rtapi_print_msg(RTAPI_MSG_INFO,
                "%s: Found ESP32 at %s\n", COMP_NAME, esp_ip);
        } else {
            rtapi_print_msg(RTAPI_MSG_WARN,
                "%s: Auto-discovery failed, falling back to %s\n",
                COMP_NAME, esp_ip);
        }
    }

    rtapi_print_msg(RTAPI_MSG_INFO,
        "%s: Ready (%d joints, target %s)\n", COMP_NAME, num_joints, esp_ip);

    /* Connect to ESP32 */
    if (establish_connection() == 0) {
        sync_config(data);
    }

    /* ===================================================================
     * Main Loop
     * =================================================================== */

    double dt = 1.0 / UPDATE_RATE_HZ;

    while (!done) {
        /* Handle E-Stop from LinuxCNC */
        if (*(data->estop_in) && esp_connected) {
            send_estop();
        }

        /* Reconnect with exponential backoff */
        if (!esp_connected && *(data->enable)) {
            uint32_t now_ms = get_timestamp_ms();
            if (now_ms >= next_reconnect_ms) {
                reconnect_attempts++;
                rtapi_print_msg(RTAPI_MSG_INFO,
                    "%s: Reconnect attempt #%d...\n",
                    COMP_NAME, reconnect_attempts);

                if (!ip_specified) {
                    char discovered_ip[64] = {0};
                    if (discover_esp32(discovered_ip, sizeof(discovered_ip), 2) == 0) {
                        strncpy(esp_ip, discovered_ip, sizeof(esp_ip) - 1);
                        inet_pton(AF_INET, esp_ip, &esp_addr.sin_addr);
                    }
                }

                if (establish_connection() == 0) {
                    sync_config(data);
                    send_reset();
                    reconnect_attempts = 0;
                } else {
                    /* Backoff: 1s, 2s, 4s, 8s, 16s, cap at 30s */
                    uint32_t delay_ms = 1000u << (reconnect_attempts < 5 ? reconnect_attempts : 4);
                    if (delay_ms > 30000) delay_ms = 30000;
                    next_reconnect_ms = now_ms + delay_ms;
                    rtapi_print_msg(RTAPI_MSG_INFO,
                        "%s: Reconnect failed, next attempt in %u ms\n",
                        COMP_NAME, delay_ms);
                }
            }
        }

        /* Send motion commands */
        if (*(data->enable) && esp_connected) {
            update_motion(data, dt);
        }

        /* Send I/O control (misc outputs, spindle, coolant) */
        if (esp_connected) {
            update_io_control(data);
        }

        /* Charge pump control */
        if (esp_connected) {
            update_charge_pump(data);
        }

        /* I/O module: reconnect with backoff */
        if (io_mod_enabled && !io_mod_connected) {
            uint32_t now_ms = get_timestamp_ms();
            if (now_ms >= io_mod_next_reconnect_ms) {
                io_mod_reconnect_attempts++;
                if (io_mod_establish_connection() == 0) {
                    io_mod_reconnect_attempts = 0;
                } else {
                    uint32_t delay_ms = 1000u << (io_mod_reconnect_attempts < 5 ?
                                                   io_mod_reconnect_attempts : 4);
                    if (delay_ms > 30000) delay_ms = 30000;
                    io_mod_next_reconnect_ms = now_ms + delay_ms;
                }
            }
        }

        /* I/O module: forward HAL output pins to I/O module */
        if (io_mod_connected) {
            static uint16_t prev_io_out = 0;
            uint16_t io_out = 0;
            for (int i = 0; i < 16; i++) {
                if (*(data->io_mod_out[i]))
                    io_out |= (1 << i);
            }
            if (io_out != prev_io_out) {
                prev_io_out = io_out;
                wcnc_io_control_packet_t pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.misc_outputs = (uint8_t)(io_out & 0xFF);
                build_header(&pkt.header, WCNC_PKT_IO_CONTROL,
                             sizeof(pkt) - sizeof(wcnc_header_t));
                wcnc_finalize_packet(&pkt, WCNC_PKT_IO_CONTROL,
                                      sizeof(pkt) - sizeof(wcnc_header_t),
                                      pkt.header.sequence, pkt.header.timestamp_us);
                sendto(io_mod_udp_send_sock, &pkt, sizeof(pkt), 0,
                       (struct sockaddr *)&io_mod_addr, sizeof(io_mod_addr));
            }
        }

        /* Read feedback */
        update_feedback(data);

        usleep(UPDATE_PERIOD_US);
    }

    /* Cleanup */
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Shutting down\n", COMP_NAME);
    if (io_mod_connected) {
        io_mod_close_connection();
    }
    if (esp_connected) {
        send_estop();
        close_connection();
    }
    hal_exit(comp_id);
    return 0;

pin_fail:
    fprintf(stderr, "%s: Failed to create HAL pin\n", COMP_NAME);
    hal_exit(comp_id);
    return 1;
}
