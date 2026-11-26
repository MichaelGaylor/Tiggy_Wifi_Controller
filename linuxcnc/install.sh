#!/bin/bash
#
# Tiggy motion controller - installer for LinuxCNC
#
#   ./install.sh            build and install the driver, then set up a machine
#   ./install.sh --driver   only build and install the driver (after an update)
#
# The driver is built by LinuxCNC's own halcompile and installed with the
# other HAL components (this needs your password, for sudo).  The set-up
# wizard then writes a configuration to ~/linuxcnc/configs.

set -e

# Started by a double click on a desktop: open a terminal for the questions.
# (Run from a script or over a connection without a screen, it just carries on.)
if [ ! -t 0 ] && [ ! -t 1 ] && [ -z "$TIGGY_IN_TERMINAL" ] && [ -n "${DISPLAY}${WAYLAND_DISPLAY}" ]; then
    SELF="$(readlink -f "$0")"
    export TIGGY_IN_TERMINAL=1
    for term in x-terminal-emulator gnome-terminal konsole xfce4-terminal lxterminal mate-terminal xterm; do
        if command -v "$term" >/dev/null 2>&1; then
            case "$term" in
                gnome-terminal) exec "$term" -- bash -c "\"$SELF\" $*; echo; read -rp 'Press Enter to close' _" ;;
                *)              exec "$term" -e bash -c "\"$SELF\" $*; echo; read -rp 'Press Enter to close' _" ;;
            esac
        fi
    done
    echo "No terminal program was found. Open a terminal and run: $SELF" >&2
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"

fail() { echo; echo "ERROR: $1" >&2; shift; for line in "$@"; do echo "       $line" >&2; done; exit 1; }

echo "Tiggy motion controller - LinuxCNC installer"
echo

[ "$(uname)" = "Linux" ] || fail "This installer is for Linux."
[ -f "$HERE/tiggy.c" ] || fail "tiggy.c is not next to this script." "Unpack the whole download and run install.sh from there."

# The protocol definition: next to the driver in the download, in ../protocol in the source tree
PROTOCOL="$HERE/wifi_cnc_protocol.h"
[ -f "$PROTOCOL" ] || PROTOCOL="$HERE/../protocol/wifi_cnc_protocol.h"
[ -f "$PROTOCOL" ] || fail "wifi_cnc_protocol.h was not found." "Unpack the whole download and run install.sh from there."

command -v halcompile >/dev/null 2>&1 || fail "LinuxCNC's halcompile was not found. Install LinuxCNC with its development files:" \
    "sudo apt install linuxcnc-uspace linuxcnc-uspace-dev" \
    "(on the LinuxCNC live image they are already there)"
command -v python3 >/dev/null 2>&1 || fail "python3 was not found: sudo apt install python3"

echo "Building the driver..."
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cp "$HERE/tiggy.c" "$PROTOCOL" "$BUILD/"
# The driver finds the protocol definition beside itself.  Newer versions of
# halcompile can be told where to look as well; older ones look there anyway.
INCLUDE="--extra-compile-args=-I$BUILD"
if ! ( cd "$BUILD" && halcompile "$INCLUDE" --compile tiggy.c ) > "$BUILD/build.log" 2>&1; then
    INCLUDE=""
    if ! ( cd "$BUILD" && halcompile --compile tiggy.c ) > "$BUILD/build.log" 2>&1; then
        cat "$BUILD/build.log"
        fail "The driver did not build. If a compiler is missing: sudo apt install build-essential"
    fi
fi
echo "Installing it (sudo may ask for your password)..."
( cd "$BUILD" && sudo halcompile $INCLUDE --install tiggy.c ) > "$BUILD/install.log" 2>&1 || {
    cat "$BUILD/install.log"
    fail "The driver could not be installed."
}
echo "The driver is installed."

if [ "$1" = "--driver" ]; then
    echo "Restart LinuxCNC to use the new driver."
    exit 0
fi

echo
python3 "$HERE/tiggy_setup.py"
