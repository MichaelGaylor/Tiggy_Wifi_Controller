#!/bin/bash
#
# WiFi CNC Controller - One-Command LinuxCNC Installer
#
# Usage:  chmod +x install.sh && ./install.sh
#
# What this does:
#   1. Checks prerequisites (LinuxCNC, halcompile)
#   2. Compiles and installs the wifi_cnc HAL component
#   3. Runs an interactive setup wizard (pure bash, no Python)
#   4. Generates LinuxCNC config files (INI, HAL, tool table)
#   5. Generates wifi_cnc.conf (ESP32 hardware settings)
#   6. Copies everything to ~/linuxcnc/configs/
#   7. Ready to launch!
#

set -e

# Auto-launch in a terminal if double-clicked from file manager
if [ ! -t 0 ] && [ ! -t 1 ]; then
    SELF="$(readlink -f "$0")"
    for term in x-terminal-emulator gnome-terminal konsole xfce4-terminal lxterminal mate-terminal xterm; do
        if command -v "$term" &>/dev/null; then
            case "$term" in
                gnome-terminal) exec "$term" -- bash "$SELF" ;;
                *)              exec "$term" -e bash "$SELF" ;;
            esac
        fi
    done
    echo "ERROR: No terminal emulator found. Please run from a terminal:" >&2
    echo "  cd $(dirname "$SELF") && ./install.sh" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMP_SOURCE="$SCRIPT_DIR/wifi_cnc.c"

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'  # No Color

banner() {
    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${BOLD}     WiFi CNC Controller - LinuxCNC Installer     ${NC}${CYAN}║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════════╝${NC}"
    echo ""
}

info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
err()   { echo -e "${RED}[ERROR]${NC} $1"; }
step()  { echo -e "\n${BOLD}>> $1${NC}"; }

# Prompt with default value
ask() {
    local prompt="$1"
    local default="$2"
    local result
    if [ -n "$default" ]; then
        read -rp "  $prompt [$default]: " result
        echo "${result:-$default}"
    else
        read -rp "  $prompt: " result
        echo "$result"
    fi
}

# Prompt for a number with default and validation
ask_number() {
    local prompt="$1"
    local default="$2"
    local result
    while true; do
        result=$(ask "$prompt" "$default")
        # Validate it's a number (integer or float, possibly negative)
        if echo "$result" | grep -qE '^-?[0-9]+\.?[0-9]*$'; then
            echo "$result"
            return
        fi
        echo -e "  ${RED}Please enter a valid number${NC}" >&2
    done
}

ask_int() {
    local prompt="$1"
    local default="$2"
    local result
    while true; do
        result=$(ask "$prompt" "$default")
        if echo "$result" | grep -qE '^[0-9]+$'; then
            echo "$result"
            return
        fi
        echo -e "  ${RED}Please enter a valid integer${NC}" >&2
    done
}

# Yes/No prompt (returns 0 for yes, 1 for no)
ask_yn() {
    local prompt="$1"
    local default="$2"
    local result
    result=$(ask "$prompt (y/n)" "$default")
    case "$result" in
        [yY]|[yY][eE][sS]) return 0 ;;
        *) return 1 ;;
    esac
}

# ===================================================================
# Step 0: Banner and checks
# ===================================================================

banner

step "Checking prerequisites..."

# Check we're on Linux
if [ "$(uname)" != "Linux" ]; then
    err "This installer is for Linux only."
    exit 1
fi

# Check the source file exists
if [ ! -f "$COMP_SOURCE" ]; then
    err "Cannot find wifi_cnc.c at: $COMP_SOURCE"
    err "Please run this script from the linuxcnc/ directory."
    exit 1
fi

# Check LinuxCNC is installed
if ! command -v halcompile &>/dev/null; then
    err "halcompile not found. Is LinuxCNC installed?"
    echo ""
    echo "  Install LinuxCNC first:"
    echo "    Debian/Ubuntu:  sudo apt install linuxcnc-uspace linuxcnc-uspace-dev"
    echo "    Or visit:       https://linuxcnc.org/downloads/"
    echo ""
    exit 1
fi

info "LinuxCNC found: $(which halcompile)"

# Check for protocol header
PROTO_HEADER="$SCRIPT_DIR/../protocol/wifi_cnc_protocol.h"
if [ ! -f "$PROTO_HEADER" ]; then
    err "Cannot find protocol header at: $PROTO_HEADER"
    err "Please ensure the full project is present."
    exit 1
fi

info "Protocol header found"

# ===================================================================
# Step 1: Compile and install HAL component
# ===================================================================

step "Compiling WiFi CNC HAL component..."

# halcompile needs the source file to include the protocol header.
# We create a temporary copy with an adjusted include path if needed.
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

# Copy source and protocol header to temp build location
cp "$COMP_SOURCE" "$TEMP_DIR/wifi_cnc.c"
mkdir -p "$TEMP_DIR/protocol"
cp "$PROTO_HEADER" "$TEMP_DIR/protocol/"

# Fix the include path for the flat build directory
sed -i 's|#include "../protocol/wifi_cnc_protocol.h"|#include "protocol/wifi_cnc_protocol.h"|' \
    "$TEMP_DIR/wifi_cnc.c"

info "Compiling with halcompile (may require sudo)..."
if sudo halcompile --install --userspace "$TEMP_DIR/wifi_cnc.c"; then
    info "HAL component installed successfully!"
else
    err "Compilation failed. Check the errors above."
    echo ""
    echo "  Common fixes:"
    echo "    - Install dev packages: sudo apt install linuxcnc-uspace-dev build-essential"
    echo "    - Check GCC is installed: gcc --version"
    echo ""
    exit 1
fi

# ===================================================================
# Step 2: Interactive Configuration Wizard
# ===================================================================

step "Machine Configuration Wizard"
echo ""
echo -e "  ${CYAN}Answer the questions below to configure your machine.${NC}"
echo -e "  ${CYAN}Press Enter to accept the default value shown in [brackets].${NC}"
echo ""

# -----------------------------------------------
# 2a. Basic Machine Settings
# -----------------------------------------------

echo -e "  ${BOLD}=== Basic Machine Settings ===${NC}"
echo ""

MACHINE_NAME=$(ask "Machine name" "WiFi-CNC")

echo ""
echo -e "  ${YELLOW}ESP32 Connection:${NC}"
echo "    Leave blank or type 'auto' for automatic discovery."
echo "    Or enter a specific IP address (e.g., 192.168.4.1)"
ESP_IP=$(ask "ESP32 IP address" "auto")

NUM_JOINTS=$(ask_int "Number of axes (1-6)" "3")
if [ "$NUM_JOINTS" -lt 1 ]; then NUM_JOINTS=1; fi
if [ "$NUM_JOINTS" -gt 6 ]; then NUM_JOINTS=6; fi

echo ""
echo "  Units:"
echo "    1) Millimeters (mm)"
echo "    2) Inches (inch)"
UNITS_CHOICE=$(ask "Select units (1 or 2)" "1")
if [ "$UNITS_CHOICE" = "2" ]; then
    LINEAR_UNITS="inch"
    UNIT_LABEL="inch"
else
    LINEAR_UNITS="mm"
    UNIT_LABEL="mm"
fi

# -----------------------------------------------
# 2b. Per-axis Settings
# -----------------------------------------------

AXIS_NAMES=(X Y Z A B C)
declare -a AX_SCALE AX_MAXVEL AX_MAXACCEL AX_MINLIM AX_MAXLIM

for (( i=0; i<NUM_JOINTS; i++ )); do
    NAME="${AXIS_NAMES[$i]}"
    echo ""
    echo -e "  ${BOLD}--- $NAME Axis ---${NC}"

    if [ $i -lt 3 ]; then
        DEF_MIN="-500.0"
        DEF_MAX="500.0"
    else
        DEF_MIN="-99999.0"
        DEF_MAX="99999.0"
    fi

    AX_SCALE[$i]=$(ask_number "  Steps per $UNIT_LABEL" "200.0")
    AX_MAXVEL[$i]=$(ask_number "  Max velocity ($UNIT_LABEL/sec)" "50.0")
    AX_MAXACCEL[$i]=$(ask_number "  Max acceleration ($UNIT_LABEL/sec^2)" "500.0")
    AX_MINLIM[$i]=$(ask_number "  Min travel ($UNIT_LABEL)" "$DEF_MIN")
    AX_MAXLIM[$i]=$(ask_number "  Max travel ($UNIT_LABEL)" "$DEF_MAX")
done

# -----------------------------------------------
# 2c. Signal Timing
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Signal Timing ===${NC}"
echo -e "  ${CYAN}Stepper driver timing parameters. Defaults work for most drivers.${NC}"
echo ""

STEP_PULSE_US=$(ask_int "Step pulse width (microseconds)" "5")
DIR_SETUP_US=$(ask_int "Direction setup time (microseconds)" "5")
STEP_IDLE_DELAY_MS=$(ask_int "Step idle delay (milliseconds, 0=always on)" "0")

# -----------------------------------------------
# 2d. Signal Inversion
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Signal Inversion ===${NC}"
echo -e "  ${CYAN}Bitmask values: bit 0=X, bit 1=Y, bit 2=Z, bit 3=A, bit 4=B, bit 5=C${NC}"
echo -e "  ${CYAN}Example: 7 = invert X+Y+Z (bits 0+1+2), 0 = no inversion${NC}"
echo ""

INVERT_STEP=$(ask_int "Invert step signals (bitmask)" "0")
INVERT_DIR=$(ask_int "Invert direction signals (bitmask)" "0")
INVERT_LIMIT=$(ask_int "Invert limit switch signals (bitmask)" "0")
INVERT_HOME=$(ask_int "Invert home switch signals (bitmask)" "0")
INVERT_ESTOP=$(ask_int "Invert E-stop signal (0 or 1)" "0")
INVERT_PROBE=$(ask_int "Invert probe signal (0 or 1)" "0")

# -----------------------------------------------
# 2e. Homing
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Homing Configuration ===${NC}"
echo -e "  ${CYAN}LinuxCNC controls homing moves. These settings configure the ESP32${NC}"
echo -e "  ${CYAN}switch interpretation. Use moderate speeds for WiFi reliability.${NC}"
echo ""

HOMING_DIR_MASK=$(ask_int "Homing direction mask (bitmask, 1=negative direction)" "0")
HOMING_SEEK_RATE=$(ask_int "Homing seek rate (steps/sec, fast search)" "500")
HOMING_FEED_RATE=$(ask_int "Homing feed rate (steps/sec, slow latch)" "50")
HOMING_PULLOFF=$(ask_int "Homing pulloff (steps, distance from switch)" "200")

# -----------------------------------------------
# 2f. Spindle / Laser
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Spindle Configuration ===${NC}"
echo ""

SPINDLE_PWM_FREQ=$(ask_int "Spindle PWM frequency (Hz)" "1000")
SPINDLE_MAX_RPM=$(ask_int "Spindle max RPM" "24000")

echo ""
echo -e "  ${CYAN}Laser mode: when enabled, spindle PWM tracks feed rate.${NC}"
echo -e "  ${CYAN}During rapids (G0), power goes to 0. During cuts (G1/G2/G3),${NC}"
echo -e "  ${CYAN}power scales with actual vs programmed feed rate.${NC}"
LASER_MODE=0
if ask_yn "Enable laser mode" "n"; then
    LASER_MODE=1
fi

# -----------------------------------------------
# 2g. Charge Pump
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Charge Pump ===${NC}"
echo -e "  ${CYAN}A fixed-frequency output that proves the software is alive.${NC}"
echo -e "  ${CYAN}External relay boards monitor this signal and cut power if it stops.${NC}"
echo -e "  ${CYAN}Set to 0 to disable. Typical value: 12500 Hz.${NC}"
echo ""

CHARGE_PUMP_FREQ=$(ask_int "Charge pump frequency (Hz, 0=disabled)" "0")

# -----------------------------------------------
# 2h. Limit Switch Mode
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Limit Switches ===${NC}"
echo -e "  ${CYAN}Shared limits: one switch per axis triggers both min and max limit.${NC}"
echo -e "  ${CYAN}Separate limits: different switches for min and max on each axis.${NC}"
echo ""

SHARED_LIMITS=1
if ask_yn "Use shared limit switches (one switch per axis)" "y"; then
    SHARED_LIMITS=1
else
    SHARED_LIMITS=0
fi

# -----------------------------------------------
# 2i. Axis Cloning
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== Axis Cloning ===${NC}"
echo -e "  ${CYAN}Clone an axis to duplicate its motion on another axis.${NC}"
echo -e "  ${CYAN}Common use: dual motors on a gantry axis (e.g., A clones X).${NC}"
echo -e "  ${CYAN}Reversed: mirror the cloned axis direction (opposite side of gantry).${NC}"
echo ""

declare -a CLONE_MASTER CLONE_REVERSED
for (( i=0; i<NUM_JOINTS; i++ )); do
    CLONE_MASTER[$i]="none"
    CLONE_REVERSED[$i]=0
done

HAS_CLONES=0
if ask_yn "Do you have any cloned/slaved axes" "n"; then
    HAS_CLONES=1
    for (( i=0; i<NUM_JOINTS; i++ )); do
        NAME="${AXIS_NAMES[$i]}"
        echo ""
        echo -e "  ${BOLD}$NAME Axis:${NC}"

        # Build list of other axes
        OTHERS="none"
        for (( j=0; j<NUM_JOINTS; j++ )); do
            if [ $j -ne $i ]; then
                OTHERS="$OTHERS/${AXIS_NAMES[$j]}"
            fi
        done

        CLONE_OF=$(ask "  Clone of ($OTHERS)" "none")
        if [ "$CLONE_OF" != "none" ] && [ -n "$CLONE_OF" ]; then
            # Validate it's a valid axis name
            VALID=0
            for (( j=0; j<NUM_JOINTS; j++ )); do
                if [ "${AXIS_NAMES[$j]}" = "$CLONE_OF" ] && [ $j -ne $i ]; then
                    VALID=1
                    break
                fi
            done
            if [ "$VALID" = "1" ]; then
                CLONE_MASTER[$i]="$CLONE_OF"
                REV=$(ask_int "  Reverse direction (0=same, 1=reversed)" "0")
                CLONE_REVERSED[$i]=$REV
            else
                echo -e "  ${YELLOW}Invalid axis name, skipping clone for $NAME${NC}" >&2
            fi
        fi
    done
fi

# -----------------------------------------------
# 2j. I/O Expansion Module (optional)
# -----------------------------------------------

echo ""
echo -e "  ${BOLD}=== I/O Expansion Module ===${NC}"
echo -e "  ${CYAN}A second ESP32 running in I/O module mode can provide${NC}"
echo -e "  ${CYAN}additional digital I/O (pendant buttons, relays, etc.).${NC}"
echo ""

IO_MOD_ENABLED=0
IO_MOD_IP=""
if ask_yn "Do you have an I/O expansion module" "n"; then
    IO_MOD_ENABLED=1
    IO_MOD_IP=$(ask "  I/O module IP address" "192.168.4.2")
fi

# ===================================================================
# Step 3: Generate Config Files
# ===================================================================

step "Generating LinuxCNC configuration files..."

CONFIG_DIR="$HOME/linuxcnc/configs/${MACHINE_NAME// /_}"
mkdir -p "$CONFIG_DIR"

# Build coordinate string and kinematics
COORDS=""
COORDS_SPACE=""
for (( i=0; i<NUM_JOINTS; i++ )); do
    COORDS="${COORDS}${AXIS_NAMES[$i]}"
    if [ $i -gt 0 ]; then
        COORDS_SPACE="${COORDS_SPACE} "
    fi
    COORDS_SPACE="${COORDS_SPACE}${AXIS_NAMES[$i]}"
done

# Find the maximum velocity across all axes
MAX_VEL="${AX_MAXVEL[0]}"
for (( i=1; i<NUM_JOINTS; i++ )); do
    if (( $(echo "${AX_MAXVEL[$i]} > $MAX_VEL" | bc -l 2>/dev/null || echo 0) )); then
        MAX_VEL="${AX_MAXVEL[$i]}"
    fi
done

# Default linear velocity (25% of first axis max)
DEF_VEL=$(echo "${AX_MAXVEL[0]} * 0.25" | bc -l 2>/dev/null || echo "12.5")
# Truncate to 1 decimal
DEF_VEL=$(printf "%.1f" "$DEF_VEL")
MAX_VEL=$(printf "%.1f" "$MAX_VEL")

# Determine --ip argument for loadusr
if [ "$ESP_IP" = "auto" ] || [ -z "$ESP_IP" ]; then
    LOADUSR_IP_ARG=""
else
    LOADUSR_IP_ARG=" --ip=$ESP_IP"
fi

# -----------------------------------------------
# 3a. wifi_cnc.conf (ESP32 hardware settings)
# -----------------------------------------------

CONF_FILE="$CONFIG_DIR/wifi_cnc.conf"
cat > "$CONF_FILE" << CONFEOF
# WiFi CNC Controller - Hardware Configuration
# Generated by install.sh
# Machine: $MACHINE_NAME
#
# Edit this file to change ESP32 settings without re-running the wizard.
# Lines starting with # or ; are comments.
# Restart LinuxCNC after making changes.

[connection]
esp_ip = $ESP_IP
num_joints = $NUM_JOINTS

[timing]
step_pulse_us = $STEP_PULSE_US
dir_setup_us = $DIR_SETUP_US
step_idle_delay_ms = $STEP_IDLE_DELAY_MS

[inversion]
# Bitmask: bit 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C
invert_step = $INVERT_STEP
invert_dir = $INVERT_DIR
invert_limit = $INVERT_LIMIT
invert_home = $INVERT_HOME
# Single value (0 or 1)
invert_estop = $INVERT_ESTOP
invert_probe = $INVERT_PROBE

[homing]
# Bitmask: 1=negative direction for that axis
homing_dir_mask = $HOMING_DIR_MASK
# Rates in steps/sec
homing_seek_rate = $HOMING_SEEK_RATE
homing_feed_rate = $HOMING_FEED_RATE
# Pulloff distance in steps
homing_pulloff = $HOMING_PULLOFF

[spindle]
spindle_pwm_freq = $SPINDLE_PWM_FREQ
spindle_max_rpm = $SPINDLE_MAX_RPM
# 0=normal spindle, 1=laser (PWM tracks feed rate, 0 during rapids)
laser_mode = $LASER_MODE

[charge_pump]
# Frequency in Hz. 0=disabled. Typical: 12500
charge_pump_freq = $CHARGE_PUMP_FREQ

[misc]
# 1=one switch per axis (triggers both limits), 0=separate min/max switches
shared_limits = $SHARED_LIMITS

[cloning]
# Clone an axis to mirror another axis's motion.
# clone_X_of = none means X moves independently.
# clone_A_of = X means A copies X's motion. clone_A_reversed = 1 mirrors it.
CONFEOF

for (( i=0; i<NUM_JOINTS; i++ )); do
    NAME="${AXIS_NAMES[$i]}"
    cat >> "$CONF_FILE" << CLONEEOF
clone_${NAME}_of = ${CLONE_MASTER[$i]}
clone_${NAME}_reversed = ${CLONE_REVERSED[$i]}
CLONEEOF
done

# I/O module section
if [ "$IO_MOD_ENABLED" = "1" ]; then
    cat >> "$CONF_FILE" << IOEOF

[io_module]
# I/O expansion module: second ESP32 in I/O module mode
io_module_enabled = 1
io_module_ip = $IO_MOD_IP
IOEOF
fi

info "Created: $CONF_FILE"

# -----------------------------------------------
# 3b. INI File
# -----------------------------------------------

INI_FILE="$CONFIG_DIR/wifi_cnc.ini"
cat > "$INI_FILE" << INIEOF
# WiFi CNC Controller - LinuxCNC Configuration
# Generated by install.sh
# Machine: $MACHINE_NAME

[EMC]
MACHINE = $MACHINE_NAME
VERSION = 1.1

[DISPLAY]
DISPLAY = axis
CYCLE_TIME = 0.100
POSITION_OFFSET = RELATIVE
POSITION_FEEDBACK = ACTUAL
MAX_FEED_OVERRIDE = 1.5
MAX_SPINDLE_OVERRIDE = 1.2
MIN_SPINDLE_OVERRIDE = 0.5
DEFAULT_LINEAR_VELOCITY = $DEF_VEL
MAX_LINEAR_VELOCITY = $MAX_VEL
EDITOR = gedit

[FILTER]
PROGRAM_EXTENSION = .py Python Script
py = python3

[RS274NGC]
PARAMETER_FILE = linuxcnc.var

[EMCMOT]
EMCMOT = motmod
COMM_TIMEOUT = 1.0
BASE_PERIOD = 0
SERVO_PERIOD = 1000000

[TASK]
TASK = milltask
CYCLE_TIME = 0.010

[HAL]
HALFILE = wifi_cnc.hal
POSTGUI_HALFILE = postgui.hal

[HALUI]

[EMCIO]
EMCIO = io
CYCLE_TIME = 0.100
TOOL_TABLE = tool.tbl

[KINS]
KINEMATICS = trivkins coordinates=$COORDS
JOINTS = $NUM_JOINTS

[TRAJ]
COORDINATES = $COORDS_SPACE
LINEAR_UNITS = $LINEAR_UNITS
ANGULAR_UNITS = degree
MAX_LINEAR_VELOCITY = $MAX_VEL
DEFAULT_LINEAR_VELOCITY = $DEF_VEL
INIEOF

# Per-axis and per-joint INI sections
for (( i=0; i<NUM_JOINTS; i++ )); do
    NAME="${AXIS_NAMES[$i]}"
    if [ $i -lt 3 ]; then
        AXIS_TYPE="LINEAR"
    else
        AXIS_TYPE="ANGULAR"
    fi
    STEPGEN_ACCEL=$(printf "%.1f" "$(echo "${AX_MAXACCEL[$i]} * 1.25" | bc -l 2>/dev/null || echo "${AX_MAXACCEL[$i]}")")

    # Homing velocities: moderate speeds for WiFi reliability
    # HOME_SEARCH_VEL = 0 means no homing for this joint (default safe)
    # Customer should set non-zero values once machine is verified working
    HOME_SEARCH_VEL="0.0"
    HOME_LATCH_VEL="0.0"

    cat >> "$INI_FILE" << AXEOF

[AXIS_$NAME]
MAX_VELOCITY = $(printf "%.1f" "${AX_MAXVEL[$i]}")
MAX_ACCELERATION = $(printf "%.1f" "${AX_MAXACCEL[$i]}")
MIN_LIMIT = $(printf "%.1f" "${AX_MINLIM[$i]}")
MAX_LIMIT = $(printf "%.1f" "${AX_MAXLIM[$i]}")

[JOINT_$i]
TYPE = $AXIS_TYPE
HOME = 0.0
MIN_LIMIT = $(printf "%.1f" "${AX_MINLIM[$i]}")
MAX_LIMIT = $(printf "%.1f" "${AX_MAXLIM[$i]}")
MAX_VELOCITY = $(printf "%.1f" "${AX_MAXVEL[$i]}")
MAX_ACCELERATION = $(printf "%.1f" "${AX_MAXACCEL[$i]}")
SCALE = $(printf "%.1f" "${AX_SCALE[$i]}")
STEPGEN_MAXACCEL = $STEPGEN_ACCEL
FERROR = 5.0
MIN_FERROR = 1.0
HOME_OFFSET = 0.0
# Homing speeds: set non-zero to enable homing for this joint.
# Keep speeds moderate over WiFi. Typical: SEARCH=10, LATCH=2
HOME_SEARCH_VEL = $HOME_SEARCH_VEL
HOME_LATCH_VEL = $HOME_LATCH_VEL
HOME_SEQUENCE = 0
AXEOF
done

info "Created: $INI_FILE"

# -----------------------------------------------
# 3c. HAL File
# -----------------------------------------------

HAL_FILE="$CONFIG_DIR/wifi_cnc.hal"
cat > "$HAL_FILE" << HALEOF
# WiFi CNC Controller - HAL Configuration
# Generated by install.sh

# Load the motion controller
loadrt [KINS]KINEMATICS
loadrt motmod servo_period_nsec=[EMCMOT]SERVO_PERIOD num_joints=[KINS]JOINTS

# Load the WiFi CNC userspace component
loadusr -W wifi_cnc${LOADUSR_IP_ARG} --joints=$NUM_JOINTS --config=wifi_cnc.conf

# Add motion controller functions to servo thread
addf motion-command-handler servo-thread
addf motion-controller servo-thread
HALEOF

# Per-joint connections
for (( i=0; i<NUM_JOINTS; i++ )); do
    NAME="${AXIS_NAMES[$i]}"
    cat >> "$HAL_FILE" << JOINTEOF

# --- Joint $i ($NAME Axis) ---
setp wifi-cnc.joint.$i.scale    [JOINT_$i]SCALE
setp wifi-cnc.joint.$i.maxvel   [JOINT_$i]MAX_VELOCITY
setp wifi-cnc.joint.$i.maxaccel [JOINT_$i]STEPGEN_MAXACCEL

net j${i}pos-cmd  joint.$i.motor-pos-cmd  => wifi-cnc.joint.$i.pos-cmd
net j${i}pos-fb   wifi-cnc.joint.$i.pos-fb   => joint.$i.motor-pos-fb
net j${i}enable   joint.$i.amp-enable-out    => wifi-cnc.joint.$i.enable
net j${i}neg-lim  wifi-cnc.joint.$i.neg-lim  => joint.$i.neg-lim-sw-in
net j${i}pos-lim  wifi-cnc.joint.$i.pos-lim  => joint.$i.pos-lim-sw-in
net j${i}home-sw  wifi-cnc.joint.$i.home-sw  => joint.$i.home-sw-in
JOINTEOF
done

# E-stop, probe, charge pump, and tool change
cat >> "$HAL_FILE" << ESTOPEOF

# --- E-Stop Chain ---
net estop-loop iocontrol.0.user-enable-out => iocontrol.0.emc-enable-in
net estop-out  iocontrol.0.user-enable-out => wifi-cnc.enable

# --- Probe ---
net probe-in   wifi-cnc.probe              => motion.probe-input

# --- Charge Pump ---
# Charge pump activates when machine is enabled, disables on E-stop.
# The ESP32 also disables charge pump automatically if connection is lost.
net charge-pump iocontrol.0.user-enable-out => wifi-cnc.charge-pump

# --- Tool change (loopback for now) ---
net tool-change      iocontrol.0.tool-change      => iocontrol.0.tool-changed
net tool-prep-loop   iocontrol.0.tool-prepare      => iocontrol.0.tool-prepared
ESTOPEOF

info "Created: $HAL_FILE"

# --- Post-GUI HAL ---
POSTGUI_FILE="$CONFIG_DIR/postgui.hal"
cat > "$POSTGUI_FILE" << 'POSTEOF'
# Post-GUI HAL commands
# Add custom signal connections here (loaded after GUI starts).
POSTEOF

# Add I/O module pendant wiring if enabled
if [ "$IO_MOD_ENABLED" = "1" ]; then
    cat >> "$POSTGUI_FILE" << 'PENDEOF'

# --- I/O Expansion Module: Pendant Jog Wiring ---
# Uncomment and adjust these lines to wire I/O module inputs
# to pendant jog buttons via halui.
#
# loadusr -W halui  # halui is usually loaded by AXIS GUI already
#
# Example: Wire I/O module inputs 0-5 to jog buttons
# net pendant-jog-x-plus  wifi-cnc.io-module.in-00 => halui.joint.0.plus
# net pendant-jog-x-minus wifi-cnc.io-module.in-01 => halui.joint.0.minus
# net pendant-jog-y-plus  wifi-cnc.io-module.in-02 => halui.joint.1.plus
# net pendant-jog-y-minus wifi-cnc.io-module.in-03 => halui.joint.1.minus
# net pendant-jog-z-plus  wifi-cnc.io-module.in-04 => halui.joint.2.plus
# net pendant-jog-z-minus wifi-cnc.io-module.in-05 => halui.joint.2.minus
#
# Example: Wire I/O module inputs to cycle start / feed hold
# net pendant-cycle-start wifi-cnc.io-module.in-06 => halui.program.run
# net pendant-feed-hold   wifi-cnc.io-module.in-07 => halui.program.pause
#
# Example: Wire I/O module outputs from LinuxCNC
# net relay-flood    wifi-cnc.io-module.out-00 <= halui.flood.is-on
# net relay-mist     wifi-cnc.io-module.out-01 <= halui.mist.is-on

PENDEOF
    info "Created: $POSTGUI_FILE (with I/O module pendant wiring template)"
else
    info "Created: $POSTGUI_FILE"
fi

# --- Tool Table ---
TOOL_FILE="$CONFIG_DIR/tool.tbl"
echo "T1 P1 Z0.000 D0.125 ;Default tool" > "$TOOL_FILE"
info "Created: $TOOL_FILE"

# ===================================================================
# Step 4: Done!
# ===================================================================

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║${BOLD}          Installation Complete!                   ${NC}${GREEN}║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  ${BOLD}Config location:${NC}  $CONFIG_DIR/"
echo ""
echo -e "  ${BOLD}Files created:${NC}"
echo "    wifi_cnc.ini   - Main LinuxCNC configuration"
echo "    wifi_cnc.hal   - HAL wiring (joints, E-Stop, charge pump, tools)"
echo "    wifi_cnc.conf  - ESP32 hardware settings (timing, inversion, etc.)"
echo "    postgui.hal    - Post-GUI HAL (edit for custom pins)"
echo "    tool.tbl       - Tool table"
echo ""
echo -e "  ${BOLD}To launch LinuxCNC:${NC}"
echo -e "    ${CYAN}linuxcnc $CONFIG_DIR/wifi_cnc.ini${NC}"
echo ""
if [ "$ESP_IP" = "auto" ] || [ -z "$ESP_IP" ]; then
    echo -e "  ${BOLD}ESP32 Connection:${NC} Auto-discovery enabled"
    echo "    The controller will automatically find your ESP32 on the network."
    echo "    Make sure the ESP32 is powered on and connected to the same network."
else
    echo -e "  ${BOLD}ESP32 Connection:${NC} $ESP_IP"
    echo "    Make sure the ESP32 is powered on at this address."
fi
echo ""
echo -e "  ${BOLD}To edit settings without re-running the wizard:${NC}"
echo -e "    ${CYAN}nano $CONFIG_DIR/wifi_cnc.conf${NC}"
echo ""
echo -e "  ${BOLD}To recompile after code changes (developer):${NC}"
echo -e "    ${CYAN}$SCRIPT_DIR/build.sh${NC}"
echo ""
echo -e "  ${YELLOW}To re-run this setup wizard:${NC}  $SCRIPT_DIR/install.sh"
echo ""
