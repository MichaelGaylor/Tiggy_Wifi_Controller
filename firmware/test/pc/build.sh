#!/bin/sh
# Builds the firmware's motion code for the PC with gcc or clang:
#   build/libfwsim.so       used by tools/firmware_sim.py
#   build/test_timed_exec   unit test of the step timing arithmetic
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
M="$HERE/../../main/motion"
OUT="$HERE/build"
mkdir -p "$OUT"
CC="${CC:-cc}"
OPTS="-std=c99 -O2 -Wall -Wextra -I$HERE -I$M"

echo "=== libfwsim.so ==="
$CC $OPTS -shared -fPIC -o "$OUT/libfwsim.so" "$HERE/fwsim.c" "$M/motion_core.c" "$M/step_engine.c" "$M/timed_exec.c" -lm

echo "=== test_timed_exec ==="
$CC $OPTS -o "$OUT/test_timed_exec" "$HERE/../test_timed_exec.c" "$M/timed_exec.c"
"$OUT/test_timed_exec"

echo "BUILD SUCCEEDED: $OUT/libfwsim.so"
