/*
 * WiFi CNC Controller - Shared Protocol Definition
 *
 * This header defines the complete communication protocol between the host
 * (Mach3 plugin or future LinuxCNC HAL) and the ESP32 motion controller.
 *
 * Written in C99 for portability across:
 *   - ESP-IDF (xtensa-gcc / riscv-gcc)
 *   - MSVC (Mach3 plugin, Win32)
 *   - GCC/Linux (future LinuxCNC HAL)
 *
 * All structures are packed to ensure identical layout across compilers.
 * All multi-byte fields are little-endian (native for both ESP32 and x86).
 */

#ifndef WIFI_CNC_PROTOCOL_H
#define WIFI_CNC_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ===================================================================
 * Protocol Constants
 * =================================================================== */

#define WCNC_MAGIC              0x574D4333  /* "WMC3" in little-endian */
#define WCNC_PROTOCOL_VERSION   1

#define WCNC_MAX_AXES           6
#define WCNC_MAX_SEGMENTS_PER_PACKET  8

/* Network ports */
#define WCNC_UDP_MOTION_PORT    58427   /* PC -> ESP32: motion segments */
#define WCNC_UDP_STATUS_PORT    58428   /* ESP32 -> PC: status reports */
#define WCNC_TCP_CONTROL_PORT   58429   /* Bidirectional: config, handshake */

/* Device name max length */
#define WCNC_DEVICE_NAME_LEN    32
#define WCNC_CONFIG_VALUE_LEN   64
#define WCNC_SSID_MAX_LEN       32
#define WCNC_PASSWORD_MAX_LEN   64

/* Version packing: major(8).minor(8).patch(16) */
#define WCNC_MAKE_VERSION(maj, min, pat) \
    (((uint32_t)(maj) << 24) | ((uint32_t)(min) << 16) | ((uint32_t)(pat)))

#define WCNC_FIRMWARE_VERSION   WCNC_MAKE_VERSION(1, 1, 0)

/* ===================================================================
 * Packet Types
 * =================================================================== */

typedef enum {
    /* PC -> ESP32 (UDP motion channel) */
    WCNC_PKT_MOTION_SEGMENT    = 0x01,
    WCNC_PKT_JOG_COMMAND       = 0x02,
    WCNC_PKT_JOG_STOP          = 0x03,
    WCNC_PKT_ESTOP             = 0x04,
    WCNC_PKT_FEED_HOLD         = 0x05,
    WCNC_PKT_FEED_RESUME       = 0x06,
    WCNC_PKT_RESET             = 0x07,
    WCNC_PKT_HOME_COMMAND      = 0x08,
    WCNC_PKT_IO_CONTROL        = 0x09,   /* Real-time output control (UDP) */

    /* ESP32 -> PC (UDP status channel) */
    WCNC_PKT_STATUS_REPORT     = 0x20,
    WCNC_PKT_ALARM             = 0x21,
    WCNC_PKT_HOME_COMPLETE     = 0x22,
    WCNC_PKT_PROBE_RESULT      = 0x23,
    WCNC_PKT_ACK               = 0x24,

    /* Bidirectional (TCP control channel) */
    WCNC_PKT_HANDSHAKE_REQ     = 0x40,
    WCNC_PKT_HANDSHAKE_RESP    = 0x41,
    WCNC_PKT_CONFIG_GET        = 0x42,
    WCNC_PKT_CONFIG_SET        = 0x43,
    WCNC_PKT_CONFIG_RESP       = 0x44,
    WCNC_PKT_CONFIG_SAVE       = 0x45,
    WCNC_PKT_FIRMWARE_INFO     = 0x46,
    WCNC_PKT_PING              = 0x50,
    WCNC_PKT_PONG              = 0x51,
} wcnc_packet_type_t;

/* ===================================================================
 * Machine States
 * =================================================================== */

typedef enum {
    WCNC_STATE_IDLE             = 0,
    WCNC_STATE_RUN              = 1,
    WCNC_STATE_HOLD             = 2,
    WCNC_STATE_JOG              = 3,
    WCNC_STATE_HOMING           = 4,
    WCNC_STATE_PROBING          = 5,
    WCNC_STATE_ALARM            = 6,
    WCNC_STATE_ESTOP            = 7,
    WCNC_STATE_DISCONNECTED     = 8,
} wcnc_machine_state_t;

/* ===================================================================
 * Alarm Codes
 * =================================================================== */

typedef enum {
    WCNC_ALARM_NONE             = 0,
    WCNC_ALARM_LIMIT_X          = 1,
    WCNC_ALARM_LIMIT_Y          = 2,
    WCNC_ALARM_LIMIT_Z          = 3,
    WCNC_ALARM_LIMIT_A          = 4,
    WCNC_ALARM_LIMIT_B          = 5,
    WCNC_ALARM_LIMIT_C          = 6,
    WCNC_ALARM_PROBE_FAIL       = 10,
    WCNC_ALARM_BUFFER_UNDERRUN  = 11,
    WCNC_ALARM_ESTOP_ACTIVE     = 20,
    WCNC_ALARM_WATCHDOG         = 30,
} wcnc_alarm_code_t;

/* ===================================================================
 * Motion Segment Flags
 * =================================================================== */

#define WCNC_SEG_FLAG_RAPID         0x01   /* G0 rapid move */
#define WCNC_SEG_FLAG_LAST          0x02   /* Last segment of a G-code block */
#define WCNC_SEG_FLAG_PROBE         0x04   /* Probe move: stop on contact */
#define WCNC_SEG_FLAG_EXACT_STOP    0x08   /* Decelerate to zero at end */

/* ===================================================================
 * Status Flags
 * =================================================================== */

#define WCNC_STATUS_WIFI_CONNECTED  0x01
#define WCNC_STATUS_HOST_CONNECTED  0x02
#define WCNC_STATUS_MOTION_ACTIVE   0x04
#define WCNC_STATUS_HOMING_ACTIVE   0x08

/* ===================================================================
 * Capability Flags (reported in handshake)
 * =================================================================== */

#define WCNC_CAP_CHARGE_PUMP       0x01
#define WCNC_CAP_MISC_OUTPUTS      0x02
#define WCNC_CAP_MISC_INPUTS       0x04
#define WCNC_CAP_SPINDLE_PWM       0x08
#define WCNC_CAP_SPINDLE_ENCODER   0x10   /* Has spindle encoder (PCNT-based) */
#define WCNC_CAP_IO_MODULE         0x20   /* Running in I/O expansion module mode */

/* ===================================================================
 * Configuration Keys
 * =================================================================== */

typedef enum {
    /* Per-axis steps/mm (float) - base + axis index */
    WCNC_CFG_STEPS_PER_MM_X    = 0x0001,
    WCNC_CFG_STEPS_PER_MM_Y    = 0x0002,
    WCNC_CFG_STEPS_PER_MM_Z    = 0x0003,
    WCNC_CFG_STEPS_PER_MM_A    = 0x0004,
    WCNC_CFG_STEPS_PER_MM_B    = 0x0005,
    WCNC_CFG_STEPS_PER_MM_C    = 0x0006,

    /* Per-axis max rate in steps/sec (uint32) */
    WCNC_CFG_MAX_RATE_X         = 0x0010,
    WCNC_CFG_MAX_RATE_Y         = 0x0011,
    WCNC_CFG_MAX_RATE_Z         = 0x0012,
    WCNC_CFG_MAX_RATE_A         = 0x0013,
    WCNC_CFG_MAX_RATE_B         = 0x0014,
    WCNC_CFG_MAX_RATE_C         = 0x0015,

    /* Per-axis acceleration in steps/sec^2 (uint32) */
    WCNC_CFG_ACCEL_X            = 0x0020,
    WCNC_CFG_ACCEL_Y            = 0x0021,
    WCNC_CFG_ACCEL_Z            = 0x0022,
    WCNC_CFG_ACCEL_A            = 0x0023,
    WCNC_CFG_ACCEL_B            = 0x0024,
    WCNC_CFG_ACCEL_C            = 0x0025,

    /* Timing parameters */
    WCNC_CFG_STEP_PULSE_US      = 0x0100,  /* uint16: step pulse width */
    WCNC_CFG_DIR_SETUP_US       = 0x0101,  /* uint16: dir setup time before step */
    WCNC_CFG_STEP_IDLE_DELAY_MS = 0x0102,  /* uint16: disable steppers after idle */
    WCNC_CFG_STATUS_INTERVAL_MS = 0x0103,  /* uint16: status report interval */

    /* WiFi (string) */
    WCNC_CFG_WIFI_SSID          = 0x0200,
    WCNC_CFG_WIFI_PASSWORD      = 0x0201,
    WCNC_CFG_IP_MODE            = 0x0210,  /* uint8: 0=DHCP, 1=static */
    WCNC_CFG_STATIC_IP          = 0x0211,  /* uint32: IPv4 address */
    WCNC_CFG_STATIC_GATEWAY     = 0x0212,  /* uint32: gateway address */
    WCNC_CFG_STATIC_NETMASK     = 0x0213,  /* uint32: subnet mask */

    /* Inversion bitmasks (uint8, bit per axis) */
    WCNC_CFG_INVERT_STEP        = 0x0300,
    WCNC_CFG_INVERT_DIR         = 0x0301,
    WCNC_CFG_INVERT_ENABLE      = 0x0302,
    WCNC_CFG_INVERT_LIMIT       = 0x0310,
    WCNC_CFG_INVERT_HOME        = 0x0311,  /* uint8: home switch inversion bitmask */
    WCNC_CFG_INVERT_ESTOP       = 0x0312,  /* uint8: 0=active low, 1=active high */
    WCNC_CFG_INVERT_PROBE       = 0x0313,  /* uint8: 0=active low, 1=active high */

    /* Homing */
    WCNC_CFG_HOMING_DIR_MASK    = 0x0320,  /* uint8: 1=positive direction */
    WCNC_CFG_HOMING_SEEK_RATE   = 0x0321,  /* uint32: steps/sec */
    WCNC_CFG_HOMING_FEED_RATE   = 0x0322,  /* uint32: steps/sec (slow) */
    WCNC_CFG_HOMING_PULLOFF     = 0x0323,  /* uint32: pulloff in steps */

    /* Output control */
    WCNC_CFG_CHARGE_PUMP_FREQ   = 0x0400,  /* uint16: Hz, 0=disabled */
    WCNC_CFG_SPINDLE_PWM_FREQ   = 0x0430,  /* uint16: Hz */
    WCNC_CFG_SPINDLE_MAX_RPM    = 0x0431,  /* uint32: max RPM at 100% duty */

    /* Pin assignments (uint8 = GPIO number, requires reboot to take effect) */
    WCNC_CFG_PIN_STEP_X         = 0x0500,
    WCNC_CFG_PIN_STEP_Y         = 0x0501,
    WCNC_CFG_PIN_STEP_Z         = 0x0502,
    WCNC_CFG_PIN_STEP_A         = 0x0503,
    WCNC_CFG_PIN_STEP_B         = 0x0504,
    WCNC_CFG_PIN_STEP_C         = 0x0505,
    WCNC_CFG_PIN_DIR_X          = 0x0506,
    WCNC_CFG_PIN_DIR_Y          = 0x0507,
    WCNC_CFG_PIN_DIR_Z          = 0x0508,
    WCNC_CFG_PIN_DIR_A          = 0x0509,
    WCNC_CFG_PIN_DIR_B          = 0x050A,
    WCNC_CFG_PIN_DIR_C          = 0x050B,
    WCNC_CFG_PIN_ENABLE         = 0x050C,
    WCNC_CFG_PIN_LIMIT_X        = 0x050D,
    WCNC_CFG_PIN_LIMIT_Y        = 0x050E,
    WCNC_CFG_PIN_LIMIT_Z        = 0x050F,
    WCNC_CFG_PIN_LIMIT_A        = 0x0510,
    WCNC_CFG_PIN_LIMIT_B        = 0x0511,
    WCNC_CFG_PIN_LIMIT_C        = 0x0512,
    WCNC_CFG_PIN_PROBE          = 0x0513,
    WCNC_CFG_PIN_ESTOP          = 0x0514,
    WCNC_CFG_PIN_SPINDLE        = 0x0515,
    WCNC_CFG_PIN_LED            = 0x0516,
    WCNC_CFG_PIN_CHARGE_PUMP    = 0x0517,
    WCNC_CFG_PIN_MISC_OUT0      = 0x0518,
    WCNC_CFG_PIN_MISC_OUT1      = 0x0519,

    /* Spindle encoder */
    WCNC_CFG_PIN_ENCODER_A      = 0x0520,  /* uint8: GPIO for quadrature channel A */
    WCNC_CFG_PIN_ENCODER_B      = 0x0521,  /* uint8: GPIO for quadrature channel B */
    WCNC_CFG_PIN_ENCODER_INDEX  = 0x0522,  /* uint8: GPIO for index pulse (0xFF=none) */
    WCNC_CFG_ENCODER_PPR        = 0x0523,  /* uint16: pulses per revolution */
    WCNC_CFG_ENCODER_MODE       = 0x0524,  /* uint8: 0=quadrature, 1=index-only */
    WCNC_CFG_ENCODER_FILTER_NS  = 0x0525,  /* uint16: PCNT glitch filter (ns) */

    /* Misc input pin assignments */
    WCNC_CFG_PIN_MISC_IN0       = 0x0530,
    WCNC_CFG_PIN_MISC_IN1       = 0x0531,
    WCNC_CFG_PIN_MISC_IN2       = 0x0532,
    WCNC_CFG_PIN_MISC_IN3       = 0x0533,

    /* Device mode */
    WCNC_CFG_DEVICE_MODE        = 0x0600,  /* uint8: 0=motion controller, 1=I/O module */

    /* I/O module configuration */
    WCNC_CFG_IO_PIN_COUNT       = 0x0610,  /* uint8: number of I/O channels */
    WCNC_CFG_IO_DIR_MASK        = 0x0611,  /* uint16: 1=output, 0=input per channel */
    WCNC_CFG_IO_PULLUP_MASK     = 0x0612,  /* uint16: 1=enable pullup per channel */
    WCNC_CFG_IO_INVERT_MASK     = 0x0613,  /* uint16: 1=invert per channel */
    /* I/O module pin GPIO assignments: 0x0620..0x062F (up to 16 channels) */
    WCNC_CFG_IO_PIN_BASE        = 0x0620,  /* uint8: GPIO for I/O channel 0 */
} wcnc_config_key_t;

/* Configuration value types */
typedef enum {
    WCNC_VAL_UINT8   = 0,
    WCNC_VAL_UINT16  = 1,
    WCNC_VAL_UINT32  = 2,
    WCNC_VAL_INT32   = 3,
    WCNC_VAL_FLOAT   = 4,
    WCNC_VAL_STRING  = 5,
} wcnc_value_type_t;

/* ===================================================================
 * Packed Structures - All protocol packets
 * =================================================================== */

#ifdef _MSC_VER
  #pragma pack(push, 1)
  #define WCNC_PACKED
#else
  #define WCNC_PACKED __attribute__((packed))
#endif

/* --- Packet Header (all packets start with this) --- */

typedef struct WCNC_PACKED {
    uint32_t magic;             /* WCNC_MAGIC */
    uint8_t  version;           /* WCNC_PROTOCOL_VERSION */
    uint8_t  packet_type;       /* wcnc_packet_type_t */
    uint16_t payload_length;    /* Bytes after header */
    uint32_t sequence;          /* Monotonically increasing per channel */
    uint32_t timestamp_us;      /* Microseconds since connection/boot */
    uint16_t checksum;          /* CRC-16/CCITT (header+payload, this field=0 during calc) */
} wcnc_header_t;

/* --- Motion Segment --- */

typedef struct WCNC_PACKED {
    int32_t  steps[WCNC_MAX_AXES];  /* Step count per axis (signed = direction) */
    uint32_t duration_us;            /* Segment duration in microseconds */
    uint32_t entry_speed_sqr;        /* Entry speed^2 * 1000 (steps/sec)^2 */
    uint32_t exit_speed_sqr;         /* Exit speed^2 * 1000 */
    uint32_t acceleration;           /* steps/sec^2 * 100 */
    uint16_t segment_id;             /* For tracking / debugging */
    uint8_t  flags;                  /* WCNC_SEG_FLAG_* */
    uint8_t  reserved;
} wcnc_motion_segment_t;

/* Motion packet: header + array of segments */
typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  segment_count;          /* 1..WCNC_MAX_SEGMENTS_PER_PACKET */
    uint8_t  reserved[3];
    wcnc_motion_segment_t segments[WCNC_MAX_SEGMENTS_PER_PACKET];
} wcnc_motion_packet_t;

/* --- Status Report --- */

typedef struct WCNC_PACKED {
    int32_t  position_steps[WCNC_MAX_AXES];
    uint16_t buffer_available;       /* Free slots in ring buffer */
    uint16_t buffer_total;           /* Total ring buffer capacity */
    uint8_t  state;                  /* wcnc_machine_state_t */
    uint8_t  alarm_code;             /* wcnc_alarm_code_t */
    uint8_t  limit_switches;         /* Bitmask: bit 0=X, bit 5=C */
    uint8_t  probe_state;            /* 0=open, 1=triggered */
    uint8_t  homing_state;           /* Bitmask of axes currently homing */
    uint8_t  home_switches;          /* Bitmask: bit 0=X home .. bit 5=C home (home inversion applied) */
    uint8_t  estop_input;            /* Physical E-Stop pin state: 0=clear, 1=active */
    uint8_t  flags;                  /* WCNC_STATUS_* flags */
    uint16_t current_segment_id;
    uint32_t uptime_ms;
    int32_t  feed_rate;              /* Current feed rate in steps/sec */
    /* --- Extended fields (v1.1, 4 bytes) --- */
    uint8_t  misc_outputs;           /* Bitmask of current misc output states */
    uint8_t  misc_inputs;            /* Bitmask of current misc input states */
    uint8_t  spindle_state;          /* 0=off, 1=CW, 2=CCW */
    uint8_t  coolant_state;          /* bit 0=flood, bit 1=mist */
    /* --- Spindle encoder (v1.1, 8 bytes) --- */
    uint16_t spindle_rpm;            /* Current RPM (0-65535) */
    uint16_t spindle_position;       /* Angular position: 0-65535 = 0-360 degrees */
    uint32_t spindle_index_count;    /* Cumulative index pulse count since boot */
    /* --- I/O module (v1.1, 4 bytes) --- */
    uint16_t io_inputs;              /* Digital input states (up to 16 channels) */
    uint16_t io_outputs;             /* Digital output states (up to 16 channels) */
} wcnc_status_report_t;

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    wcnc_status_report_t report;
} wcnc_status_packet_t;

/* --- Jog Command --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  axis;                   /* 0=X .. 5=C */
    int8_t   direction;              /* -1 or +1 */
    uint8_t  reserved[2];
    uint32_t speed;                  /* Steps per second */
} wcnc_jog_packet_t;

/* --- Jog Stop --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  axis;                   /* 0=X .. 5=C, or 0xFF = all axes */
    uint8_t  reserved[3];
} wcnc_jog_stop_packet_t;

/* --- Home Command --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  axis_mask;              /* Bitmask of axes to home */
    uint8_t  reserved[3];
} wcnc_home_packet_t;

/* --- IO Control (PC -> ESP32, UDP) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  misc_outputs;           /* Bitmask: bit 0..4 misc output states */
    uint8_t  spindle_state;          /* 0=off, 1=CW, 2=CCW */
    uint16_t spindle_rpm;            /* Target RPM (0..max_rpm) */
    uint8_t  coolant_state;          /* bit 0=flood, bit 1=mist */
    uint8_t  reserved[3];
} wcnc_io_control_packet_t;

/* --- Home Complete --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint8_t  axis_mask;              /* Bitmask of axes that completed homing */
    uint8_t  success;                /* 1=success, 0=failed */
    uint8_t  reserved[2];
} wcnc_home_complete_packet_t;

/* --- Probe Result --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    int32_t  position_steps[WCNC_MAX_AXES]; /* Position at probe contact */
    uint8_t  success;                /* 1=contact made, 0=no contact */
    uint8_t  reserved[3];
} wcnc_probe_result_packet_t;

/* --- ACK (for critical commands) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint32_t acked_sequence;         /* Sequence number being acknowledged */
    uint8_t  acked_type;             /* Packet type being acknowledged */
    uint8_t  reserved[3];
} wcnc_ack_packet_t;

/* --- E-Stop (minimal packet for fastest processing) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    /* No payload - header alone triggers E-Stop */
} wcnc_estop_packet_t;

/* --- Feed Hold / Resume / Reset (header-only packets) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
} wcnc_control_packet_t;

/* --- Handshake Request (PC -> ESP32, TCP) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint32_t host_version;           /* Host software version */
    char     host_name[WCNC_DEVICE_NAME_LEN]; /* Host identifier */
} wcnc_handshake_req_t;

/* --- Handshake Response (ESP32 -> PC, TCP) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint32_t firmware_version;
    uint8_t  num_axes;
    uint8_t  capabilities;           /* WCNC_CAP_* flags */
    uint16_t buffer_capacity;        /* Segment ring buffer size */
    uint32_t max_step_rate;          /* Max aggregate steps/sec */
    char     device_name[WCNC_DEVICE_NAME_LEN];
    /* --- v1.1 extension --- */
    uint8_t  device_mode;            /* 0=motion controller, 1=I/O module */
    uint8_t  encoder_ppr_hi;         /* Encoder PPR high byte */
    uint8_t  encoder_ppr_lo;         /* Encoder PPR low byte */
    uint8_t  io_channel_count;       /* Number of I/O channels (I/O module mode) */
} wcnc_handshake_resp_t;

/* --- Config Get/Set (TCP) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint16_t key;                    /* wcnc_config_key_t */
    uint16_t value_type;             /* wcnc_value_type_t */
    uint8_t  value[WCNC_CONFIG_VALUE_LEN];
} wcnc_config_packet_t;

/* --- Ping / Pong (TCP keepalive) --- */

typedef struct WCNC_PACKED {
    wcnc_header_t header;
    uint32_t ping_id;                /* Echoed back in pong */
} wcnc_ping_packet_t;

#ifdef _MSC_VER
  #pragma pack(pop)
#endif

/* ===================================================================
 * CRC-16/CCITT Calculation
 * =================================================================== */

static inline uint16_t wcnc_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        int j;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* Compute checksum for a complete packet.
 * The checksum field in the header must be 0 during computation. */
static inline uint16_t wcnc_compute_checksum(const void *packet, size_t total_len)
{
    return wcnc_crc16((const uint8_t *)packet, total_len);
}

/* Validate a received packet's checksum.
 * Returns 1 if valid, 0 if invalid. */
static inline int wcnc_validate_packet(const void *packet, size_t total_len)
{
    if (total_len < sizeof(wcnc_header_t)) return 0;

    const wcnc_header_t *hdr = (const wcnc_header_t *)packet;
    if (hdr->magic != WCNC_MAGIC) return 0;
    if (hdr->version != WCNC_PROTOCOL_VERSION) return 0;

    /* Save and zero checksum field for computation */
    uint16_t received_crc = hdr->checksum;
    uint8_t *buf = (uint8_t *)packet;
    size_t crc_offset = offsetof(wcnc_header_t, checksum);
    uint8_t saved[2] = { buf[crc_offset], buf[crc_offset + 1] };
    buf[crc_offset] = 0;
    buf[crc_offset + 1] = 0;

    uint16_t computed_crc = wcnc_crc16(buf, total_len);

    /* Restore */
    buf[crc_offset] = saved[0];
    buf[crc_offset + 1] = saved[1];

    return (computed_crc == received_crc) ? 1 : 0;
}

/* Fill header fields and compute checksum.
 * Call this after all payload fields are set. */
static inline void wcnc_finalize_packet(void *packet, uint8_t type,
                                         uint16_t payload_len,
                                         uint32_t sequence,
                                         uint32_t timestamp_us)
{
    wcnc_header_t *hdr = (wcnc_header_t *)packet;
    hdr->magic = WCNC_MAGIC;
    hdr->version = WCNC_PROTOCOL_VERSION;
    hdr->packet_type = type;
    hdr->payload_length = payload_len;
    hdr->sequence = sequence;
    hdr->timestamp_us = timestamp_us;
    hdr->checksum = 0;
    hdr->checksum = wcnc_crc16((const uint8_t *)packet,
                                sizeof(wcnc_header_t) + payload_len);
}

#ifdef __cplusplus
}
#endif

#endif /* WIFI_CNC_PROTOCOL_H */
