#!/bin/sh
# Builds the LinuxCNC driver for the PC, around a stand-in for LinuxCNC, so
# that it can be tested against the firmware simulator without LinuxCNC:
#   build/libtiggy_rig.so   used by tools/linuxcnc_driver_test.py
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/build"
mkdir -p "$OUT"
CC="${CC:-cc}"

echo "=== libtiggy_rig.so ==="
$CC -std=gnu99 -O2 -Wall -Wextra -shared -fPIC -fvisibility=hidden \
    -I"$HERE/shim" -I"$HERE/../../protocol" \
    -o "$OUT/libtiggy_rig.so" "$HERE/../tiggy.c" "$HERE/rig.c" "$HERE/shim/shim.c" -lm -lpthread

echo "BUILD SUCCEEDED: $OUT/libtiggy_rig.so"
