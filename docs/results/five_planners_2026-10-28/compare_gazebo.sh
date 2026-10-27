#!/usr/bin/env bash
# Serial full-task benchmark. Each trial starts a fresh isolated headless world.
set -eo pipefail
PANDA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PANDA_ROOT/install/setup.bash"
PANDA_RUNS="${RUNS:-3}"
PANDA_ALGORITHMS="${ALGORITHMS:-rrt rrt_connect rrt_star astar prm_astar}"
PANDA_PADS="${COMPLIANT_PADS:-false}"
PANDA_PAD_K="${PAD_STIFFNESS:-10000}"
PANDA_PAD_D="${PAD_DAMPING:-1.4}"
PANDA_PAD_TRAVEL="${PAD_TRAVEL:-0.0015}"
PANDA_SCENE="${SCENE:-world}"
PANDA_TIMING="${CARTESIAN_TIMING:-bounded}"
PANDA_BUDGET="${TIMEOUT:-30.0}"
PANDA_OUTPUT="${OUTPUT_DIR:-$PANDA_ROOT/results/gazebo_$(date +%Y%m%d_%H%M%S)}"
PANDA_DOMAIN_BASE="${BENCHMARK_ROS_DOMAIN_ID:-87}"
export ROS_DOMAIN_ID="$PANDA_DOMAIN_BASE"
PANDA_PARTITION_BASE="${BENCHMARK_IGN_PARTITION:-panda_benchmark_20261025}"
export IGN_PARTITION="$PANDA_PARTITION_BASE"
export ROS_LOG_DIR="$PANDA_OUTPUT/ros_logs"
[[ "$PANDA_RUNS" =~ ^[1-9][0-9]*$ ]] || { echo "RUNS 必须为正整数"; exit 1; }
mkdir -p "$PANDA_OUTPUT"
cp "$PANDA_ROOT/src/panda_planning/worlds/$PANDA_SCENE.urdf" "$PANDA_OUTPUT/world.urdf"
cp "$PANDA_ROOT/src/panda_planning/worlds/$PANDA_SCENE.sdf" "$PANDA_OUTPUT/world.sdf"
cp "$PANDA_ROOT/src/panda_ompl/config/controller.yaml" "$PANDA_OUTPUT/controller.yaml"
xacro "$PANDA_ROOT/src/franka_description/robots/panda/panda_sim.urdf.xacro" compliant_pads:="$PANDA_PADS" pad_stiffness:="$PANDA_PAD_K" pad_damping:="$PANDA_PAD_D" pad_travel:="$PANDA_PAD_TRAVEL" > "$PANDA_OUTPUT/robot.urdf"
xacro "$PANDA_ROOT/src/franka_description/robots/panda/panda_sim.urdf.xacro" compliant_pads:=false > "$PANDA_OUTPUT/planning_robot.urdf"
cp "$PANDA_ROOT/src/panda_planning/launch/panda_world.launch.xml" "$PANDA_OUTPUT/launch.xml"
python3 - "$PANDA_ROOT" "$PANDA_OUTPUT" <<'SNAPSHOT'
from pathlib import Path
import sys,hashlib
root,out=map(Path,sys.argv[1:])
rows=[hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(root)) for p in sorted((root/'src').rglob('*')) if p.is_file()]
manifest='\n'.join(rows)+'\n'
target=out/'source_sha256.txt'
if target.exists() and target.read_text()!=manifest:raise SystemExit('源码已变化，不能在同一数据集中恢复试验')
target.write_text(manifest)
SNAPSHOT
{
  printf 'date_utc=%s\nruns_per_algorithm=%s\nplanning_budget_s=%s\nros_domain=%s\nign_partition=%s\n' "$(date -u +%FT%TZ)" "$PANDA_RUNS" "$PANDA_BUDGET" "$ROS_DOMAIN_ID" "$IGN_PARTITION"
  printf 'compliant_pads=%s\npad_stiffness=%s\npad_damping=%s\npad_travel=%s\n' "$PANDA_PADS" "$PANDA_PAD_K" "$PANDA_PAD_D" "$PANDA_PAD_TRAVEL"
  printf 'scene=%s\n' "$PANDA_SCENE"
  printf 'cartesian_timing=%s\nalgorithms=%s\n' "$PANDA_TIMING" "$PANDA_ALGORITHMS"
  uname -srmo
  lscpu
  dpkg-query -W ros-humble-ompl ros-humble-moveit-core libfcl0.7 ignition-fortress 2>/dev/null || true
} > "$PANDA_OUTPUT/environment.txt"
launch_pid=""
stop_trial() {
  if [[ -n "$launch_pid" ]]; then
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for ((k=0;k<50;k++)); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep .1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
    launch_pid=""
  fi
}
trap stop_trial EXIT
trap 'exit 130' INT TERM
# Interleave algorithms to reduce the effect of changing background load.
for ((seed=1;seed<=PANDA_RUNS;seed++)); do
  for algorithm in $PANDA_ALGORITHMS; do
    prefix="$PANDA_OUTPUT/${algorithm}_seed${seed}"
    if [[ "${RESUME:-0}" == 1 && -f "${prefix}_task.csv" ]]; then
      printf 'KEEP completed algorithm=%s seed=%s\n' "$algorithm" "$seed"
      continue
    fi
    case "$algorithm" in rrt) offset=0;; rrt_connect) offset=1;; rrt_star) offset=2;; astar) offset=3;; prm_astar) offset=4;; *) echo "不支持算法 $algorithm"; exit 1;; esac
    export ROS_DOMAIN_ID=$((PANDA_DOMAIN_BASE + (seed-1)*5 + offset))
    export IGN_PARTITION="${PANDA_PARTITION_BASE}_${algorithm}_${seed}"
    printf '%s,%s,%s,%s\n' "$algorithm" "$seed" "$ROS_DOMAIN_ID" "$IGN_PARTITION" >> "$PANDA_OUTPUT/trial_domains.csv"
    if [[ -f "${prefix}_launch.log" ]]; then
      mv "${prefix}_launch.log" "${prefix}_startup_retry_$(date -u +%H%M%S).log"
    fi
    printf 'START algorithm=%s seed=%s\n' "$algorithm" "$seed"
    setsid "$PANDA_ROOT/start.sh" gui:=false compliant_pads:="$PANDA_PADS" pad_stiffness:="$PANDA_PAD_K" pad_damping:="$PANDA_PAD_D" pad_travel:="$PANDA_PAD_TRAVEL" scene:="$PANDA_SCENE" planner_algorithm:="$algorithm" seed:="$seed" planning_timeout:="$PANDA_BUDGET" cartesian_timing:="$PANDA_TIMING" > "${prefix}_launch.log" 2>&1 &
    launch_pid=$!
    ready=false
    for ((k=0;k<120;k++)); do
      if [[ -f "${prefix}_launch.log" ]] && grep -q '启动完成：三个控制器已激活' "${prefix}_launch.log"; then ready=true;break;fi
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep .5
    done
    if [[ "$ready" != true ]]; then
      printf 'STARTUP_FAILED algorithm=%s seed=%s log=%s\n' "$algorithm" "$seed" "${prefix}_launch.log" | tee -a "$PANDA_OUTPUT/infrastructure_failures.txt"
      stop_trial
      exit 1 # Do not report infrastructure failure as a planner timeout.
    fi
    timeout 300 ros2 run panda_ompl gazebo_trial_node --ros-args \
      -p algorithm:="$algorithm" -p seed:="$seed" -p output_prefix:="$prefix" > "${prefix}_runner.log" 2>&1
    cat "${prefix}_runner.log"
    stop_trial
    sleep 1
  done
done
printf 'DONE raw_results=%s\n' "$PANDA_OUTPUT"
