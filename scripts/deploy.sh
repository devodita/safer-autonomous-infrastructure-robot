#!/usr/bin/env bash
# deploy.sh — Build and deploy SAFER workspace to the robot over SSH.
# Usage: ./scripts/deploy.sh [ROBOT_IP] [ROBOT_USER]

set -euo pipefail

ROBOT_IP="${1:-192.168.1.100}"
ROBOT_USER="${2:-safer}"
REMOTE_WS="/home/${ROBOT_USER}/safer-autonomous-infrastructure-robot"
LOCAL_WS="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Syncing workspace to ${ROBOT_USER}@${ROBOT_IP}:${REMOTE_WS}"
rsync -avz --exclude='build/' --exclude='devel/' --exclude='.git/' \
  "${LOCAL_WS}/" "${ROBOT_USER}@${ROBOT_IP}:${REMOTE_WS}/"

echo "==> Building on robot..."
ssh "${ROBOT_USER}@${ROBOT_IP}" bash <<EOF
  source /opt/ros/noetic/setup.bash
  cd ${REMOTE_WS}
  catkin_make -DCMAKE_BUILD_TYPE=Release
  echo "Build complete."
EOF

echo "==> Deploy finished."
