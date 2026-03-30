/*
 * WiFi CNC Controller - G-Code Parser
 *
 * GRBL/FluidNC-compatible G-code line parser.
 * Parses ASCII G-code into structured blocks and manages modal state.
 */

#include "gcode_parser.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* ===================================================================
 * Modal group definitions for conflict detection
 *
 * Each G-code belongs to exactly one modal group. Two G-codes from
 * the same group on a single line is error 21.
 * =================================================================== */

typedef enum {
    GROUP_MOTION        = 0,    /* G0, G1, G2, G3, G38.x, G80 */
    GROUP_PLANE         = 1,    /* G17, G18, G19 */
    GROUP_DISTANCE      = 2,    /* G90, G91 */
    GROUP_ARC_DISTANCE  = 3,    /* G90.1, G91.1 */
    GROUP_FEED_MODE     = 4,    /* G93, G94 */
    GROUP_UNITS         = 5,    /* G20, G21 */
    GROUP_TLO           = 6,    /* G43.1, G49 */
    GROUP_WCS           = 7,    /* G54-G59 */
    GROUP_PATH_CONTROL  = 8,    /* G61, G64 */
    GROUP_NON_MODAL     = 9,    /* G4, G10, G28, G30, G53, G92 */
    GROUP_SPINDLE       = 10,   /* M3, M4, M5 */
    GROUP_COOLANT       = 11,   /* M7, M8, M9 */
    GROUP_PROGRAM       = 12,   /* M0, M1, M2, M30 */
    GROUP_COUNT
} modal_group_t;

/* ===================================================================
 * Initialization
 * =================================================================== */

void gcode_parser_init(gcode_state_t *state)
{
    memset(state, 0, sizeof(*state));

    /* Power-on defaults (GRBL-compatible) */
    state->motion_mode = MOTION_RAPID;      /* G0 */
    state->wcs_index = 0;                   /* G54 */
    state->plane = PLANE_XY;                /* G17 */
    state->incremental = false;             /* G90 (absolute) */
    state->arc_incremental = true;          /* G91.1 (incremental IJK) */
    state->inverse_time = false;            /* G94 (units/min) */
    state->inches = false;                  /* G21 (mm) */
    state->exact_stop = false;              /* G64 (path blending) */
    state->path_tolerance = 0.0f;
    state->spindle = 0;                     /* M5 (off) */
    state->spindle_speed = 0.0f;
    state->coolant = 0;                     /* M9 (off) */
    state->feed_rate = 0.0f;
    state->tool = 0;
    state->tlo = 0.0f;
    state->tlo_active = false;
    state->probe_succeeded = false;
}

/* ===================================================================
 * Block Initialization
 * =================================================================== */

static void block_init(gcode_block_t *block)
{
    memset(block, 0, sizeof(*block));
    block->motion_mode = MOTION_NONE;
    block->motion_mode_set = false;
    block->non_modal_command = -1;
    block->wcs_select = -1;
    block->plane_select = -1;
    block->distance_mode = -1;
    block->arc_distance_mode = -1;
    block->feed_rate_mode = -1;
    block->units_mode = -1;
    block->tool_length_mode = -1;
    block->path_control_mode = -1;
    block->spindle_mode = -1;
    block->coolant_mode = -1;
    block->program_flow = -1;
    block->l_value = -1;
    block->t_value = -1;
    block->n_value = -1;
    block->h_value = -1;
}

/* ===================================================================
 * G-word Dispatch
 * =================================================================== */

static gcode_error_t handle_g_word(gcode_block_t *block, float value,
                                    uint8_t group_seen[GROUP_COUNT])
{
    /* Round to nearest integer * 10 to handle sub-commands (G38.2 = 382) */
    int g_int = (int)lroundf(value * 10.0f);

    switch (g_int) {
    /* --- Motion group --- */
    case 0:     /* G0 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_RAPID;
        block->motion_mode_set = true;
        break;
    case 10:    /* G1 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_LINEAR;
        block->motion_mode_set = true;
        break;
    case 20:    /* G2 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_CW_ARC;
        block->motion_mode_set = true;
        break;
    case 30:    /* G3 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_CCW_ARC;
        block->motion_mode_set = true;
        break;
    case 382:   /* G38.2 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_PROBE_TOWARD;
        block->motion_mode_set = true;
        break;
    case 383:   /* G38.3 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_PROBE_TOWARD_NE;
        block->motion_mode_set = true;
        break;
    case 384:   /* G38.4 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_PROBE_AWAY;
        block->motion_mode_set = true;
        break;
    case 385:   /* G38.5 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_PROBE_AWAY_NE;
        block->motion_mode_set = true;
        break;
    case 800:   /* G80 */
        if (group_seen[GROUP_MOTION]++) return GCODE_ERR_MODAL_GROUP;
        block->motion_mode = MOTION_CANNED_CANCEL;
        block->motion_mode_set = true;
        break;

    /* --- Plane group --- */
    case 170:   /* G17 */
        if (group_seen[GROUP_PLANE]++) return GCODE_ERR_MODAL_GROUP;
        block->plane_select = 17;
        break;
    case 180:   /* G18 */
        if (group_seen[GROUP_PLANE]++) return GCODE_ERR_MODAL_GROUP;
        block->plane_select = 18;
        break;
    case 190:   /* G19 */
        if (group_seen[GROUP_PLANE]++) return GCODE_ERR_MODAL_GROUP;
        block->plane_select = 19;
        break;

    /* --- Units group --- */
    case 200:   /* G20 (inches) */
        if (group_seen[GROUP_UNITS]++) return GCODE_ERR_MODAL_GROUP;
        block->units_mode = 20;
        break;
    case 210:   /* G21 (mm) */
        if (group_seen[GROUP_UNITS]++) return GCODE_ERR_MODAL_GROUP;
        block->units_mode = 21;
        break;

    /* --- Non-modal G-codes --- */
    case 40:    /* G4 (dwell) */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 4;
        break;
    case 100:   /* G10 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 10;
        break;
    case 280:   /* G28 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 28;
        break;
    case 281:   /* G28.1 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 281;
        break;
    case 300:   /* G30 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 30;
        break;
    case 301:   /* G30.1 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 301;
        break;
    case 530:   /* G53 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 53;
        break;

    /* --- WCS group --- */
    case 540:   /* G54 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 0;
        break;
    case 550:   /* G55 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 1;
        break;
    case 560:   /* G56 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 2;
        break;
    case 570:   /* G57 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 3;
        break;
    case 580:   /* G58 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 4;
        break;
    case 590:   /* G59 */
        if (group_seen[GROUP_WCS]++) return GCODE_ERR_MODAL_GROUP;
        block->wcs_select = 5;
        break;

    /* --- Path control group --- */
    case 610:   /* G61 (exact stop) */
        if (group_seen[GROUP_PATH_CONTROL]++) return GCODE_ERR_MODAL_GROUP;
        block->path_control_mode = 61;
        break;
    case 640:   /* G64 (path blending) */
        if (group_seen[GROUP_PATH_CONTROL]++) return GCODE_ERR_MODAL_GROUP;
        block->path_control_mode = 64;
        break;

    /* --- Distance mode --- */
    case 900:   /* G90 */
        if (group_seen[GROUP_DISTANCE]++) return GCODE_ERR_MODAL_GROUP;
        block->distance_mode = 0;
        break;
    case 901:   /* G90.1 */
        if (group_seen[GROUP_ARC_DISTANCE]++) return GCODE_ERR_MODAL_GROUP;
        block->arc_distance_mode = 0;
        break;
    case 910:   /* G91 */
        if (group_seen[GROUP_DISTANCE]++) return GCODE_ERR_MODAL_GROUP;
        block->distance_mode = 1;
        break;
    case 911:   /* G91.1 */
        if (group_seen[GROUP_ARC_DISTANCE]++) return GCODE_ERR_MODAL_GROUP;
        block->arc_distance_mode = 1;
        break;

    /* --- Coordinate offset --- */
    case 920:   /* G92 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 92;
        break;
    case 921:   /* G92.1 */
        if (group_seen[GROUP_NON_MODAL]++) return GCODE_ERR_MODAL_GROUP;
        block->non_modal_command = 921;
        break;

    /* --- Feed rate mode --- */
    case 930:   /* G93 (inverse time) */
        if (group_seen[GROUP_FEED_MODE]++) return GCODE_ERR_MODAL_GROUP;
        block->feed_rate_mode = 93;
        break;
    case 940:   /* G94 (units/min) */
        if (group_seen[GROUP_FEED_MODE]++) return GCODE_ERR_MODAL_GROUP;
        block->feed_rate_mode = 94;
        break;

    /* --- Tool length offset --- */
    case 430:   /* G43 (tool length offset from tool table) */
        if (group_seen[GROUP_TLO]++) return GCODE_ERR_MODAL_GROUP;
        block->tool_length_mode = 430;
        break;
    case 431:   /* G43.1 (dynamic tool length offset) */
        if (group_seen[GROUP_TLO]++) return GCODE_ERR_MODAL_GROUP;
        block->tool_length_mode = 43;
        break;
    case 490:   /* G49 */
        if (group_seen[GROUP_TLO]++) return GCODE_ERR_MODAL_GROUP;
        block->tool_length_mode = 49;
        break;

    /* --- Cutter compensation (recognized but not implemented) --- */
    case 400:   /* G40 (cancel cutter compensation - always active, no-op) */
        /* G41/G42 not supported, but G40 must be accepted for Mach3 compat */
        break;

    default:
        return GCODE_ERR_UNSUPPORTED_CMD;
    }

    return GCODE_OK;
}

/* ===================================================================
 * M-word Dispatch
 * =================================================================== */

static gcode_error_t handle_m_word(gcode_block_t *block, float value,
                                    uint8_t group_seen[GROUP_COUNT])
{
    int m_int = (int)lroundf(value);

    switch (m_int) {
    /* --- Program flow --- */
    case 0:     /* M0 (pause) */
        if (group_seen[GROUP_PROGRAM]++) return GCODE_ERR_MODAL_GROUP;
        block->program_flow = 0;
        break;
    case 1:     /* M1 (optional stop) */
        if (group_seen[GROUP_PROGRAM]++) return GCODE_ERR_MODAL_GROUP;
        block->program_flow = 1;
        break;
    case 2:     /* M2 (end) */
        if (group_seen[GROUP_PROGRAM]++) return GCODE_ERR_MODAL_GROUP;
        block->program_flow = 2;
        break;
    case 30:    /* M30 (end + rewind) */
        if (group_seen[GROUP_PROGRAM]++) return GCODE_ERR_MODAL_GROUP;
        block->program_flow = 30;
        break;

    /* --- Spindle --- */
    case 3:     /* M3 (CW) */
        if (group_seen[GROUP_SPINDLE]++) return GCODE_ERR_MODAL_GROUP;
        block->spindle_mode = 3;
        break;
    case 4:     /* M4 (CCW) */
        if (group_seen[GROUP_SPINDLE]++) return GCODE_ERR_MODAL_GROUP;
        block->spindle_mode = 4;
        break;
    case 5:     /* M5 (off) */
        if (group_seen[GROUP_SPINDLE]++) return GCODE_ERR_MODAL_GROUP;
        block->spindle_mode = 5;
        break;

    /* --- Coolant --- */
    case 7:     /* M7 (mist) */
        if (group_seen[GROUP_COOLANT]++) return GCODE_ERR_MODAL_GROUP;
        block->coolant_mode = 7;
        break;
    case 8:     /* M8 (flood) */
        if (group_seen[GROUP_COOLANT]++) return GCODE_ERR_MODAL_GROUP;
        block->coolant_mode = 8;
        break;
    case 9:     /* M9 (off) */
        if (group_seen[GROUP_COOLANT]++) return GCODE_ERR_MODAL_GROUP;
        block->coolant_mode = 9;
        break;

    /* --- Tool change --- */
    case 6:     /* M6 (tool change) */
        if (group_seen[GROUP_PROGRAM]++) return GCODE_ERR_MODAL_GROUP;
        block->program_flow = 6;
        break;

    default:
        return GCODE_ERR_UNSUPPORTED_CMD;
    }

    return GCODE_OK;
}

/* ===================================================================
 * Line Parser
 * =================================================================== */

gcode_error_t gcode_parse_line(const char *line, gcode_block_t *block,
                                const gcode_state_t *state)
{
    block_init(block);

    /* Modal group conflict tracker */
    uint8_t group_seen[GROUP_COUNT];
    memset(group_seen, 0, sizeof(group_seen));

    const char *ptr = line;

    /* Skip leading whitespace */
    while (*ptr == ' ' || *ptr == '\t') ptr++;

    /* Empty line or comment-only line */
    if (*ptr == '\0' || *ptr == '\n' || *ptr == '\r' || *ptr == ';') {
        return GCODE_OK;
    }

    /* Main parsing loop: extract letter-value pairs */
    while (*ptr) {
        /* Skip whitespace */
        while (*ptr == ' ' || *ptr == '\t') ptr++;

        /* End of line / comment */
        if (*ptr == '\0' || *ptr == '\n' || *ptr == '\r' || *ptr == ';') break;

        /* Skip inline comments (parentheses) */
        if (*ptr == '(') {
            int depth = 1;
            ptr++;
            while (*ptr && depth > 0) {
                if (*ptr == '(') depth++;
                else if (*ptr == ')') depth--;
                ptr++;
            }
            continue;
        }

        /* Must be a letter */
        char letter = toupper((unsigned char)*ptr);
        if (letter < 'A' || letter > 'Z') {
            return GCODE_ERR_EXPECTED_CMD;
        }
        ptr++;

        /* Parse the number following the letter */
        char *end;
        float value = strtof(ptr, &end);
        if (end == ptr) {
            return GCODE_ERR_BAD_NUMBER;
        }
        ptr = end;

        /* Dispatch by letter */
        gcode_error_t err;
        switch (letter) {
        case 'G':
            err = handle_g_word(block, value, group_seen);
            if (err != GCODE_OK) return err;
            break;

        case 'M':
            err = handle_m_word(block, value, group_seen);
            if (err != GCODE_OK) return err;
            break;

        case 'F':
            block->f_value = value;
            block->word_bits |= WORD_F;
            break;

        case 'S':
            block->s_value = value;
            block->word_bits |= WORD_S;
            break;

        case 'T':
            block->t_value = (int)lroundf(value);
            block->word_bits |= WORD_T;
            break;

        case 'H':
            block->h_value = (int)lroundf(value);
            block->word_bits |= WORD_H;
            break;

        case 'N':
            block->n_value = (int)lroundf(value);
            block->word_bits |= WORD_N;
            break;

        case 'X':
            block->values[0] = value;
            block->word_bits |= WORD_X;
            break;
        case 'Y':
            block->values[1] = value;
            block->word_bits |= WORD_Y;
            break;
        case 'Z':
            block->values[2] = value;
            block->word_bits |= WORD_Z;
            break;
        case 'A':
            block->values[3] = value;
            block->word_bits |= WORD_A;
            break;
        case 'B':
            block->values[4] = value;
            block->word_bits |= WORD_B;
            break;
        case 'C':
            block->values[5] = value;
            block->word_bits |= WORD_C;
            break;

        case 'I':
            block->ijk[0] = value;
            block->word_bits |= WORD_I;
            break;
        case 'J':
            block->ijk[1] = value;
            block->word_bits |= WORD_J;
            break;
        case 'K':
            block->ijk[2] = value;
            block->word_bits |= WORD_K;
            break;

        case 'R':
            block->r_value = value;
            block->word_bits |= WORD_R;
            break;

        case 'P':
            block->p_value = value;
            block->word_bits |= WORD_P;
            break;

        case 'L':
            block->l_value = (int)lroundf(value);
            block->word_bits |= WORD_L;
            break;

        default:
            /* Unknown letter -- ignore for forward compatibility */
            break;
        }
    }

    /* If no motion mode was specified on this line, inherit from state */
    if (!block->motion_mode_set) {
        block->motion_mode = state->motion_mode;
    }

    return GCODE_OK;
}

/* ===================================================================
 * Block Validation
 * =================================================================== */

gcode_error_t gcode_validate_block(const gcode_block_t *block,
                                    const gcode_state_t *state)
{
    /* Check for undefined feed rate on G1/G2/G3 */
    gcode_motion_mode_t mode = block->motion_mode;
    if (mode == MOTION_LINEAR || mode == MOTION_CW_ARC || mode == MOTION_CCW_ARC) {
        float feed = (block->word_bits & WORD_F) ? block->f_value : state->feed_rate;
        if (feed <= 0.0f) {
            return GCODE_ERR_UNDEFINED_FEED;
        }
    }

    /* Arc validation (G2/G3) */
    if (mode == MOTION_CW_ARC || mode == MOTION_CCW_ARC) {
        bool has_ijk = (block->word_bits & (WORD_I | WORD_J | WORD_K)) != 0;
        bool has_r = (block->word_bits & WORD_R) != 0;

        /* Must have either IJK or R, not both */
        if (!has_ijk && !has_r) {
            return GCODE_ERR_NO_ARC_AXIS;
        }
        if (has_ijk && has_r) {
            return GCODE_ERR_INVALID_GCODE;
        }

        /* Must have target coordinates */
        if (!(block->word_bits & WORD_ANY_AXIS)) {
            return GCODE_ERR_NO_ARC_AXIS;
        }

        /* R=0 is invalid */
        if (has_r && block->r_value == 0.0f) {
            return GCODE_ERR_ARC_RADIUS;
        }
    }

    /* G4 (dwell) requires P */
    if (block->non_modal_command == 4) {
        if (!(block->word_bits & WORD_P)) {
            return GCODE_ERR_INVALID_GCODE;
        }
        if (block->p_value < 0.0f) {
            return GCODE_ERR_NEGATIVE_VALUE;
        }
    }

    /* G10 requires L and P */
    if (block->non_modal_command == 10) {
        if (!(block->word_bits & WORD_L) || !(block->word_bits & WORD_P)) {
            return GCODE_ERR_INVALID_GCODE;
        }
        if (block->l_value != 2 && block->l_value != 20) {
            return GCODE_ERR_INVALID_GCODE;
        }
        int p = (int)lroundf(block->p_value);
        /* P0 = current WCS, P1..P6 = G54..G59 */
        if (p < 0 || p > GCODE_MAX_WCS) {
            return GCODE_ERR_INVALID_GCODE;
        }
    }

    /* G53 only with G0 or G1 */
    if (block->non_modal_command == 53) {
        if (mode != MOTION_RAPID && mode != MOTION_LINEAR) {
            return GCODE_ERR_INVALID_GCODE;
        }
    }

    /* G43.1 requires Z (tool length) */
    if (block->tool_length_mode == 43) {
        if (!(block->word_bits & WORD_Z)) {
            return GCODE_ERR_INVALID_GCODE;
        }
    }

    /* Probe moves need at least one axis */
    if (mode >= MOTION_PROBE_TOWARD && mode <= MOTION_PROBE_AWAY_NE) {
        if (!(block->word_bits & WORD_ANY_AXIS)) {
            return GCODE_ERR_INVALID_TARGET;
        }
        /* Probe requires feed rate */
        float feed = (block->word_bits & WORD_F) ? block->f_value : state->feed_rate;
        if (feed <= 0.0f) {
            return GCODE_ERR_UNDEFINED_FEED;
        }
    }

    return GCODE_OK;
}

/* ===================================================================
 * State Update
 * =================================================================== */

void gcode_update_state(gcode_state_t *state, const gcode_block_t *block)
{
    /* Update motion mode */
    if (block->motion_mode_set) {
        state->motion_mode = block->motion_mode;
    }

    /* WCS */
    if (block->wcs_select >= 0) {
        state->wcs_index = block->wcs_select;
    }

    /* Plane */
    if (block->plane_select >= 0) {
        state->plane = (gcode_plane_t)block->plane_select;
    }

    /* Distance mode */
    if (block->distance_mode >= 0) {
        state->incremental = (block->distance_mode == 1);
    }

    /* Arc distance mode */
    if (block->arc_distance_mode >= 0) {
        state->arc_incremental = (block->arc_distance_mode == 1);
    }

    /* Feed rate mode */
    if (block->feed_rate_mode >= 0) {
        state->inverse_time = (block->feed_rate_mode == 93);
    }

    /* Units */
    if (block->units_mode >= 0) {
        state->inches = (block->units_mode == 20);
    }

    /* Path control */
    if (block->path_control_mode >= 0) {
        state->exact_stop = (block->path_control_mode == 61);
        if (block->path_control_mode == 64) {
            state->exact_stop = false;
            /* G64 P value is stored in p_value when path_control_mode is 64 */
            if (block->word_bits & WORD_P) {
                state->path_tolerance = block->p_value;
            } else {
                state->path_tolerance = 0.0f;
            }
        }
    }

    /* Feed rate */
    if (block->word_bits & WORD_F) {
        state->feed_rate = block->f_value;
    }

    /* Spindle */
    if (block->spindle_mode >= 0) {
        if (block->spindle_mode == 5) {
            state->spindle = 0;
        } else {
            state->spindle = block->spindle_mode;
        }
    }
    if (block->word_bits & WORD_S) {
        state->spindle_speed = block->s_value;
    }

    /* Coolant */
    if (block->coolant_mode >= 0) {
        if (block->coolant_mode == 9) {
            state->coolant = 0;
        } else {
            state->coolant = block->coolant_mode;
        }
    }

    /* Tool */
    if (block->word_bits & WORD_T) {
        state->tool = block->t_value;
    }

    /* Tool length offset */
    if (block->tool_length_mode == 43) {
        /* G43.1: dynamic TLO from Z value */
        state->tlo = block->values[2]; /* Z value */
        state->tlo_active = true;
    } else if (block->tool_length_mode == 430) {
        /* G43: TLO from tool table (no table, offset=0) */
        state->tlo_active = true;
        state->tlo = 0.0f;
    } else if (block->tool_length_mode == 49) {
        state->tlo = 0.0f;
        state->tlo_active = false;
    }

    /* Program end resets modal state to defaults */
    if (block->program_flow == 2 || block->program_flow == 30) {
        int wcs = state->wcs_index;     /* Preserve WCS */
        float pos[GCODE_MAX_AXES];
        memcpy(pos, state->position, sizeof(pos));

        /* Preserve coordinate systems */
        float wcs_save[GCODE_MAX_WCS][GCODE_MAX_AXES];
        memcpy(wcs_save, state->wcs, sizeof(wcs_save));
        float g92_save[GCODE_MAX_AXES];
        memcpy(g92_save, state->g92_offset, sizeof(g92_save));

        gcode_parser_init(state);

        state->wcs_index = wcs;
        memcpy(state->position, pos, sizeof(pos));
        memcpy(state->wcs, wcs_save, sizeof(wcs_save));
        memcpy(state->g92_offset, g92_save, sizeof(g92_save));
    }
}
