/*
 * WiFi CNC Controller - G-Code Parser
 *
 * GRBL/FluidNC-compatible G-code line parser and modal state machine.
 * Parses ASCII G-code lines into structured blocks for execution.
 */

#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define GCODE_MAX_LINE_LENGTH   256
#define GCODE_MAX_AXES          6   /* X, Y, Z, A, B, C */
#define GCODE_MAX_WCS           6   /* G54..G59 */

/* ===================================================================
 * Error Codes (GRBL-compatible numbering)
 * =================================================================== */

typedef enum {
    GCODE_OK                    = 0,
    GCODE_ERR_EXPECTED_CMD      = 1,
    GCODE_ERR_BAD_NUMBER        = 2,
    GCODE_ERR_INVALID_STATEMENT = 3,
    GCODE_ERR_NEGATIVE_VALUE    = 4,
    GCODE_ERR_SETTING_DISABLED  = 5,
    GCODE_ERR_STEP_PULSE_MIN    = 6,
    GCODE_ERR_SETTINGS_READ     = 7,
    GCODE_ERR_IDLE_ERROR        = 8,
    GCODE_ERR_ALARM_LOCK        = 9,
    GCODE_ERR_SOFT_LIMIT        = 10,
    GCODE_ERR_OVERFLOW          = 11,
    GCODE_ERR_MAX_STEP_RATE     = 12,
    GCODE_ERR_UNSUPPORTED_CMD   = 20,
    GCODE_ERR_MODAL_GROUP       = 21,
    GCODE_ERR_UNDEFINED_FEED    = 22,
    GCODE_ERR_INVALID_GCODE     = 23,
    GCODE_ERR_INVALID_TARGET    = 24,
    GCODE_ERR_ARC_RADIUS        = 25,
    GCODE_ERR_NO_ARC_AXIS       = 26,
    GCODE_ERR_UNUSED_WORDS      = 27,
    GCODE_ERR_INVALID_LINE      = 30,
    GCODE_ERR_PROBE_FAIL        = 31,
} gcode_error_t;

/* ===================================================================
 * Motion Modes
 * =================================================================== */

typedef enum {
    MOTION_NONE             = -1,
    MOTION_RAPID            = 0,    /* G0 */
    MOTION_LINEAR           = 1,    /* G1 */
    MOTION_CW_ARC           = 2,    /* G2 */
    MOTION_CCW_ARC          = 3,    /* G3 */
    MOTION_PROBE_TOWARD     = 38,   /* G38.2 */
    MOTION_PROBE_TOWARD_NE  = 39,   /* G38.3 (no error on miss) */
    MOTION_PROBE_AWAY       = 40,   /* G38.4 */
    MOTION_PROBE_AWAY_NE    = 41,   /* G38.5 */
    MOTION_CANNED_CANCEL    = 80,   /* G80 */
} gcode_motion_mode_t;

/* ===================================================================
 * Plane Selection
 * =================================================================== */

typedef enum {
    PLANE_XY = 17,
    PLANE_ZX = 18,
    PLANE_YZ = 19,
} gcode_plane_t;

/* ===================================================================
 * Word Presence Bits
 * =================================================================== */

#define WORD_F  (1u << 0)
#define WORD_I  (1u << 1)
#define WORD_J  (1u << 2)
#define WORD_K  (1u << 3)
#define WORD_L  (1u << 4)
#define WORD_N  (1u << 5)
#define WORD_P  (1u << 6)
#define WORD_R  (1u << 7)
#define WORD_S  (1u << 8)
#define WORD_T  (1u << 9)
#define WORD_X  (1u << 10)
#define WORD_Y  (1u << 11)
#define WORD_Z  (1u << 12)
#define WORD_A  (1u << 13)
#define WORD_B  (1u << 14)
#define WORD_C  (1u << 15)
#define WORD_H  (1u << 16)

/* Mask for any axis word present */
#define WORD_ANY_AXIS  (WORD_X | WORD_Y | WORD_Z | WORD_A | WORD_B | WORD_C)

/* ===================================================================
 * Parsed Block (result of parsing one G-code line)
 * =================================================================== */

typedef struct {
    uint32_t word_bits;                     /* Which words were present */

    /* G-code motion command (if any) */
    gcode_motion_mode_t motion_mode;        /* Updated if motion G on this line */
    bool motion_mode_set;                   /* true if motion G was on this line */

    /* Non-modal G-codes (one per line max) */
    int non_modal_command;                  /* G4, G10, G28, G30, G53, G92, etc. -1=none */

    /* Modal state changes (-1 = unchanged) */
    int wcs_select;                         /* G54=0 .. G59=5 */
    int plane_select;                       /* G17/18/19 */
    int distance_mode;                      /* 0=abs(G90), 1=inc(G91) */
    int arc_distance_mode;                  /* 0=abs(G90.1), 1=inc(G91.1) */
    int feed_rate_mode;                     /* 93=inverse, 94=normal */
    int units_mode;                         /* 20=inch, 21=mm */
    int tool_length_mode;                   /* 43=G43.1, 49=G49 */
    int path_control_mode;                  /* 61=exact stop, 64=path blending */

    /* M-code commands (-1 = unchanged/none) */
    int spindle_mode;                       /* 3=CW, 4=CCW, 5=off */
    int coolant_mode;                       /* 7=mist, 8=flood, 9=off */
    int program_flow;                       /* 0=M0, 1=M1, 2=M2, 30=M30 */

    /* Word values */
    float f_value;                          /* F word (feed rate) */
    float values[GCODE_MAX_AXES];           /* X, Y, Z, A, B, C */
    float ijk[3];                           /* I, J, K (arc center offsets) */
    float r_value;                          /* R (arc radius) */
    float p_value;                          /* P (dwell, G10 param) */
    float s_value;                          /* S (spindle speed) */
    int   l_value;                          /* L (G10 sub-command) */
    int   t_value;                          /* T (tool number) */
    int   n_value;                          /* N (line number) */
    int   h_value;                          /* H (tool number for G43) */
} gcode_block_t;

/* ===================================================================
 * Persistent Modal State
 * =================================================================== */

typedef struct {
    /* Motion mode */
    gcode_motion_mode_t motion_mode;

    /* Modal groups */
    int             wcs_index;              /* 0..5 for G54..G59 */
    gcode_plane_t   plane;                  /* G17/G18/G19 */
    bool            incremental;            /* G91 mode */
    bool            arc_incremental;        /* G91.1 mode (default: true for GRBL compat) */
    bool            inverse_time;           /* G93 mode */
    bool            inches;                 /* G20 mode */
    bool            exact_stop;             /* G61 mode (false = G64 path blending) */
    float           path_tolerance;         /* G64 P value (0 = use junction_deviation) */

    /* Spindle / coolant */
    int             spindle;                /* 0=off, 3=CW, 4=CCW */
    float           spindle_speed;          /* RPM (S word) */
    int             coolant;                /* 0=off, 7=mist, 8=flood */

    /* Feed rate */
    float           feed_rate;              /* mm/min (or inch/min in G20) */

    /* Tool */
    int             tool;                   /* Tool number */
    float           tlo;                    /* Tool length offset (G43.1) */
    bool            tlo_active;             /* G43.1 active */

    /* Coordinate offsets */
    float           wcs[GCODE_MAX_WCS][GCODE_MAX_AXES];    /* G54-G59 offsets */
    float           g92_offset[GCODE_MAX_AXES];             /* G92 offset */
    float           g28_position[GCODE_MAX_AXES];           /* G28.1 stored pos */
    float           g30_position[GCODE_MAX_AXES];           /* G30.1 stored pos */

    /* Current position in machine coordinates (mm) */
    float           position[GCODE_MAX_AXES];

    /* Probe result */
    float           probe_position[GCODE_MAX_AXES];
    bool            probe_succeeded;
} gcode_state_t;

/* ===================================================================
 * API Functions
 * =================================================================== */

/* Initialize parser state to power-on defaults:
 * G54, G17, G90, G91.1, G94, G21, G64, M5, M9 */
void gcode_parser_init(gcode_state_t *state);

/* Parse a single G-code line into a block structure.
 * Returns GCODE_OK or an error code.
 * Does NOT modify modal state -- that happens during execution. */
gcode_error_t gcode_parse_line(const char *line, gcode_block_t *block,
                                const gcode_state_t *state);

/* Validate a parsed block against current modal state.
 * Checks for undefined feed rate, invalid targets, arc errors, etc. */
gcode_error_t gcode_validate_block(const gcode_block_t *block,
                                    const gcode_state_t *state);

/* Update modal state from an executed block.
 * Call this after successfully executing the motion/action. */
void gcode_update_state(gcode_state_t *state, const gcode_block_t *block);

#endif /* GCODE_PARSER_H */
