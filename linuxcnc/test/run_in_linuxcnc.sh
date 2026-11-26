#!/bin/bash
# Tests the Tiggy driver inside a real LinuxCNC, without a machine.
#
# Builds and installs the driver with halcompile (sudo), writes a
# configuration with the set-up wizard, starts the firmware simulator as the
# controller and runs LinuxCNC with linuxcnc_session.py in place of its
# screen.  Needs linuxcnc-uspace, linuxcnc-uspace-dev, python3 and a compiler.
#
#   linuxcnc/test/run_in_linuxcnc.sh
#
# Exit status 0 when every step passed.

set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
WORK="$(mktemp -d)"
SIM_PID=""
cleanup() { [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null; rm -rf "$WORK"; }
trap cleanup EXIT

for tool in linuxcnc halcompile python3; do
    command -v "$tool" >/dev/null 2>&1 || { echo "ERROR: $tool was not found"; exit 2; }
done

echo "=== firmware simulator ==="
sh "$ROOT/firmware/test/pc/build.sh" > "$WORK/fwsim.log" 2>&1 || { cat "$WORK/fwsim.log"; exit 2; }

echo "=== driver ==="
TIGGY_IN_TERMINAL=1 bash "$ROOT/linuxcnc/install.sh" --driver

chmod +x "$HERE/linuxcnc_session.py"
export TIGGY_TEST_RESULT="$WORK/result.txt"
STATUS=0

run_plan() {    # name of the plan, folder of its configuration
    local ini
    ini="$(ls "$2"/*.ini)"
    # No screen: the test session takes its place, and no tool-change prompt
    sed -i "s|^DISPLAY = axis|DISPLAY = $HERE/linuxcnc_session.py|" "$ini"
    python3 "$ROOT/tools/firmware_sim.py" --bind 127.0.0.1 --quiet &
    SIM_PID=$!
    sleep 2
    echo "=== LinuxCNC: $1 ==="
    TIGGY_TEST_PLAN="$1" linuxcnc "$ini" > "$WORK/linuxcnc_$1.log" 2>&1 || true
    kill "$SIM_PID" 2>/dev/null || true
    wait "$SIM_PID" 2>/dev/null || true
    SIM_PID=""
    if ! grep -q "^ALL PASSED ($1)" "$TIGGY_TEST_RESULT" 2>/dev/null; then
        STATUS=1
        echo "--- LinuxCNC's output ---"
        tail -n 60 "$WORK/linuxcnc_$1.log"
    fi
}

# A plain three-axis machine
python3 "$ROOT/linuxcnc/tiggy_setup.py" --defaults --name TiggyTest --address 127.0.0.1 \
    --no-toolchange-prompt --dir "$WORK/basic" > /dev/null
run_plan basic "$WORK/basic"

# The same with home switches: X and Y home and limit at the low end, Z home at the top
printf '%s\n' TiggySwitches 127.0.0.1 mm XYZ n \
    800 50 500 0 300 n b n n \
    800 50 500 0 300 n b n n \
    800 50 500 -100 0 n h n y \
    5 5 n  o  n n  n n | python3 "$ROOT/linuxcnc/tiggy_setup.py" --dir "$WORK/switches" > /dev/null
run_plan switches "$WORK/switches"

echo
cat "$TIGGY_TEST_RESULT" 2>/dev/null || echo "LinuxCNC produced no results"
exit $STATUS
