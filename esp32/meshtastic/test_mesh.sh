#!/bin/bash
# Quick test script to rebuild, flash both boards, and monitor

set -e

echo "=== Building firmware ==="
. $HOME/.esp/v6.0/esp-idf/export.sh > /dev/null 2>&1
cd /home/tobe/src/esp32/meshtastic

idf.py build

echo "=== Flashing board 1 on /dev/ttyUSB1 ==="
idf.py -p /dev/ttyUSB1 flash

echo "=== Flashing board 2 on /dev/ttyUSB2 ==="
idf.py -p /dev/ttyUSB2 flash

echo "=== Monitoring board 1 for 30 seconds ==="
timeout 30 idf.py -p /dev/ttyUSB1 monitor || true

echo "=== Done ==="
