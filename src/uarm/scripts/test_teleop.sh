#!/bin/bash

# === Setup ROS environment ===
source /opt/ros/noetic/setup.bash

# Directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "$SCRIPT_DIR/../devel/setup.bash"

echo "[INFO] ROS environment loaded, starting nodes..."

LEFT_PORT=${LEFT_PORT:-/dev/ttyUSB0}
RIGHT_PORT=${RIGHT_PORT:-/dev/ttyUSB1}
LEFT_TOPIC=${LEFT_TOPIC:-/arm_left}
RIGHT_TOPIC=${RIGHT_TOPIC:-/arm_right}

echo "[INFO] Left arm:  port=${LEFT_PORT}, topic=${LEFT_TOPIC}"
echo "[INFO] Right arm: port=${RIGHT_PORT}, topic=${RIGHT_TOPIC}"

# === Start left servo_reader.py ===
rosrun UArm servo_reader.py _serial_port:=${LEFT_PORT} _arm_topic:=${LEFT_TOPIC} __name:=servo_reader_left &
PID_LEFT=$!
echo "[INFO] servo_reader.py (left) started with PID ${PID_LEFT}"

# === Start right servo_reader.py ===
rosrun UArm servo_reader.py _serial_port:=${RIGHT_PORT} _arm_topic:=${RIGHT_TOPIC} __name:=servo_reader_right &
PID_RIGHT=$!
echo "[INFO] servo_reader.py (right) started with PID ${PID_RIGHT}"



# === Setup cleanup logic for Ctrl+C ===
trap "echo '[INFO] Ctrl+C received. Shutting down all nodes...'; kill $PID_LEFT $PID_RIGHT; exit" SIGINT

# === Wait for all child processes to finish ===
wait
