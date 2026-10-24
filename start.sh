#!/usr/bin/env bash
set -euo pipefail
PANDA_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ROS setup scripts may reference unset variables.
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "$PANDA_WORKSPACE/install/setup.bash" ]]; then
  echo "请先编译：colcon build --base-paths src --symlink-install"
  exit 1
fi
source "$PANDA_WORKSPACE/install/setup.bash"
set -u
PANDA_INSTANCE_KEY="$(printf '%s' "${UID}:${ROS_DOMAIN_ID:-0}:${IGN_PARTITION:-default}" | sha256sum)"
PANDA_INSTANCE_KEY="${PANDA_INSTANCE_KEY%% *}"
exec 9>"/tmp/panda_sim_${UID}_${PANDA_INSTANCE_KEY}.lock"
if ! flock -n 9; then
  echo "Panda 已在运行，请先在原启动终端按 Ctrl+C。"
  exit 1
fi
exec ros2 launch panda_planning panda_world.launch.xml "$@"
