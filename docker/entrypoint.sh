#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

MODE=${1:-topics}

# Start the dummy publishers in the background
ros2 launch ros2_tui_test dummy_publishers.launch.py \
    enable_debug:=false > /tmp/publishers.log 2>&1 &
PUBLISHERS_PID=$!

echo "[ros2_tui] Waiting for publishers to come up..."
sleep 5

echo "[ros2_tui] Launching $MODE TUI (Ctrl-C or 'q' to exit)"
/ros2_tui/target/release/"$MODE"

# Clean up background publishers when the TUI exits
kill "$PUBLISHERS_PID" 2>/dev/null || true
