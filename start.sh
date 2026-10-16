#!/usr/bin/env bash
set -e
PANDA_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$PANDA_WORKSPACE/install/setup.bash" ]]; then
    echo "请先在工作区编译：/usr/bin/python3 -m colcon build --base-paths src --symlink-install"
    exit 1
fi
source "$PANDA_WORKSPACE/install/setup.bash"
exec ros2 launch panda_planning panda_world.launch.py "$@"
