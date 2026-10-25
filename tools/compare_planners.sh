#!/usr/bin/env bash
# Offline only. Fresh process per seed/algorithm/ablation avoids OMPL RNG reuse.
set -euo pipefail
PANDA_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PANDA_RUNS="${RUNS:-10}"
PANDA_BUDGET="${TIMEOUT:-5.0}"
PANDA_OUTPUT="${OUTPUT_DIR:-$PANDA_ROOT/results/$(date +%Y%m%d_%H%M%S)}"
[[ "$PANDA_RUNS" =~ ^[1-9][0-9]*$ ]] || { echo "RUNS 必须为正整数"; exit 1; }
mkdir -p "$PANDA_OUTPUT"
set +u
source /opt/ros/humble/setup.bash
source "$PANDA_ROOT/install/setup.bash"
set -u
xacro "$PANDA_ROOT/src/franka_description/robots/panda/panda_sim.urdf.xacro" > "$PANDA_OUTPUT/robot.urdf"
# Preserve the exact inputs with the CSV. Edit these before running a new query.
cp "$PANDA_ROOT/src/panda_ompl/config/benchmark.yaml" "$PANDA_OUTPUT/query.yaml"
cp "$PANDA_ROOT/src/panda_planning/worlds/world.urdf" "$PANDA_OUTPUT/world.urdf"
for algorithm in ${ALGORITHMS:-rrt rrt_connect rrt_star astar}; do
  for mode in raw shortcut bspline; do
    shortcut=false; bspline=false
    [[ "$mode" == raw ]] || shortcut=true
    [[ "$mode" != bspline ]] || bspline=true
    for ((seed=1;seed<=PANDA_RUNS;seed++)); do
      ros2 run panda_ompl benchmark_node --ros-args --params-file "$PANDA_OUTPUT/query.yaml" \
        -p urdf_file:="$PANDA_OUTPUT/robot.urdf" -p world_file:="$PANDA_OUTPUT/world.urdf" \
        -p planner_algorithm:="$algorithm" -p seed:="$seed" -p timeout:="$PANDA_BUDGET" \
        -p shortcut:="$shortcut" -p bspline:="$bspline" -p output:="$PANDA_OUTPUT/results.csv"
    done
  done
done
printf 'CSV 和场景快照：%s\n' "$PANDA_OUTPUT"

# Fields 1..17 are numeric/simple tokens; quoted error text is the final field.
awk -F, '
NR==1 {next}
{key=$1 "," $4 "," $5; n[key]++; wall[key]+=$9;
 if($6==1){ok[key]++; search[key]+=$7; length[key]+=$13; duration[key]+=$14}}
END {
 print "planner,shortcut,bspline,trials,successes,success_rate,mean_search_success_s,mean_total_wall_all_s,mean_trajectory_length_success_rad,mean_duration_success_s";
 for(k in n) {
   if(ok[k]>0) printf "%s,%d,%d,%.4f,%.6f,%.6f,%.6f,%.6f\n",k,n[k],ok[k],ok[k]/n[k],search[k]/ok[k],wall[k]/n[k],length[k]/ok[k],duration[k]/ok[k];
   else printf "%s,%d,0,0,NA,%.6f,NA,NA\n",k,n[k],wall[k]/n[k];
 }}' "$PANDA_OUTPUT/results.csv" > "$PANDA_OUTPUT/summary.csv"
printf '汇总：%s/summary.csv\n' "$PANDA_OUTPUT"
