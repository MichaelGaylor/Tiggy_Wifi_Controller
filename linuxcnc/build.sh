#!/bin/bash
#
# WiFi CNC Controller - Quick Recompile Script (Developer Use)
#
# Usage:  chmod +x build.sh && ./build.sh
#
# This script recompiles and installs the wifi_cnc HAL component
# without running the full setup wizard. Use this during development
# and fault-finding for quick iteration.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
COMP_SOURCE="$SCRIPT_DIR/wifi_cnc.c"
PROTO_HEADER="$SCRIPT_DIR/../protocol/wifi_cnc_protocol.h"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

echo ""
echo -e "${BOLD}WiFi CNC - Quick Recompile${NC}"
echo "=========================="
echo ""

# Check prerequisites
if [ "$(uname)" != "Linux" ]; then
    echo -e "${RED}[ERROR]${NC} This script is for Linux only."
    exit 1
fi

if [ ! -f "$COMP_SOURCE" ]; then
    echo -e "${RED}[ERROR]${NC} Cannot find wifi_cnc.c at: $COMP_SOURCE"
    exit 1
fi

if ! command -v halcompile &>/dev/null; then
    echo -e "${RED}[ERROR]${NC} halcompile not found. Is LinuxCNC installed?"
    echo ""
    echo "  Install: sudo apt install linuxcnc-uspace linuxcnc-uspace-dev"
    echo ""
    exit 1
fi

if [ ! -f "$PROTO_HEADER" ]; then
    echo -e "${RED}[ERROR]${NC} Cannot find protocol header at: $PROTO_HEADER"
    exit 1
fi

# Create temp build directory
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

# Copy source and protocol header
cp "$COMP_SOURCE" "$TEMP_DIR/wifi_cnc.c"
mkdir -p "$TEMP_DIR/protocol"
cp "$PROTO_HEADER" "$TEMP_DIR/protocol/"

# Fix the include path for the flat build directory
sed -i 's|#include "../protocol/wifi_cnc_protocol.h"|#include "protocol/wifi_cnc_protocol.h"|' \
    "$TEMP_DIR/wifi_cnc.c"

echo -e "${BOLD}Compiling...${NC}"
if sudo halcompile --install --userspace "$TEMP_DIR/wifi_cnc.c"; then
    echo ""
    echo -e "${GREEN}[OK]${NC} wifi_cnc HAL component installed successfully"
    echo ""
    echo -e "  ${YELLOW}Note:${NC} If LinuxCNC is running, restart it to load the new version."
    echo ""
else
    echo ""
    echo -e "${RED}[FAIL]${NC} Compilation failed. Check errors above."
    echo ""
    echo "  Common fixes:"
    echo "    sudo apt install linuxcnc-uspace-dev build-essential"
    echo ""
    exit 1
fi
