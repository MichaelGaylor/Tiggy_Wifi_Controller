#!/bin/bash
# Tests the Tiggy driver inside a real LinuxCNC against a real controller.
#
# Builds and installs the driver with halcompile (sudo), writes a plain
# three-axis configuration with the set-up wizard and runs LinuxCNC with
# board_session.py in place of its screen: homing, MDI, a program with a
# pause, jogging, abort, E-stop and a long run, with LinuxCNC's position
# compared to the controller's own step count after each.  Needs
# linuxcnc-uspace, linuxcnc-uspace-dev and python3, and no other program
# (LinuxCNC, Mach3) talking to the controller.
#
#   linuxcnc/test/run_on_board.sh <address> --motion [minutes of long run, default 2]
#
# --motion says that the axes are free to move: a bare board, or a machine
# with 60 mm clear on X and Y and 10 mm below Z.  The controller keeps the
# settings LinuxCNC gives it (step pulse 5 us, 800 steps/mm at 50 mm/s) until
# another host sets its own.
#
# Exit status 0 when every step passed.

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

ADDRESS=""
MOTION=""
MINUTES="2"
for arg in "$@"; do
    case "$arg" in
        --motion) MOTION=1 ;;
        -*) echo "ERROR: unknown option $arg"; exit 2 ;;
        *) if [ -z "$ADDRESS" ]; then ADDRESS="$arg"; else MINUTES="$arg"; fi ;;
    esac
done
if [ -z "$ADDRESS" ] || [ -z "$MOTION" ]; then
    sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
fi

for tool in linuxcnc halcompile python3; do
    command -v "$tool" >/dev/null 2>&1 || { echo "ERROR: $tool was not found"; exit 2; }
done
if pgrep -x linuxcncsvr >/dev/null 2>&1; then
    echo "ERROR: LinuxCNC is running on this computer: close it first"
    exit 2
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "=== driver ==="
TIGGY_IN_TERMINAL=1 bash "$ROOT/linuxcnc/install.sh" --driver || exit 2

python3 "$ROOT/linuxcnc/tiggy_setup.py" --defaults --name TiggyBoard --address "$ADDRESS" \
    --no-toolchange-prompt --dir "$WORK/cfg" > /dev/null || exit 2
ini="$(ls "$WORK"/cfg/*.ini)"
chmod +x "$HERE/board_session.py"
# No screen: the test session takes its place, and there is no start-up picture
sed -i -e "s|^DISPLAY = axis|DISPLAY = $HERE/board_session.py|" -e "/^INTRO_/d" "$ini"

export TIGGY_TEST_RESULT="$WORK/result.txt"
export TIGGY_TEST_MINUTES="$MINUTES"
rm -f "$HOME/linuxcnc_debug.txt" "$HOME/linuxcnc_print.txt"
echo "=== LinuxCNC against the controller at $ADDRESS ==="
code=0
# -r: without a terminal LinuxCNC would otherwise put its output in
# temporary files and delete them
timeout $((900 + ${MINUTES%.*} * 120)) linuxcnc -r "$ini" > "$WORK/linuxcnc.log" 2>&1 || code=$?

echo
cat "$TIGGY_TEST_RESULT" 2>/dev/null || echo "LinuxCNC produced no results"
if ! grep -q "^ALL PASSED" "$TIGGY_TEST_RESULT" 2>/dev/null; then
    echo "--- LinuxCNC ended with status $code; its output: ---"
    tail -n 70 "$WORK/linuxcnc.log"
    for f in "$HOME/linuxcnc_debug.txt" "$HOME/linuxcnc_print.txt"; do
        if [ -s "$f" ]; then
            echo "--- $f ---"
            tail -n 25 "$f"
        fi
    done
    exit 1
fi
exit 0
