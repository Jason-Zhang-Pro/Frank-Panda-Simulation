# A* 关节空间规划

`astar_planner.cpp` 实现 A*，通过 `planner_algorithm=astar` 接入 PlanningEngine。它不是 OMPL 的 RRT 实现，也不是在三维末端空间中寻路。

状态是七个关节角，以每次请求的实际起点为网格原点，步长 0.15 rad，每个节点具有 14 个单关节正负方向邻居。边代价和启发函数均采用七维欧氏距离，f=g+h。精确目标作为额外节点，与距离不超过 0.30 rad 且连接无碰撞的网格节点相连，不将目标角度简单取整。

边以不大于 0.02 rad 的关节空间距离离散检查，调用相同的 MoveIt/FCL 检查关节限位、自碰撞、障碍及携物碰撞。搜索受 planning_timeout 和 500000 节点上限约束，超时、内存上限或图耗尽均明确报错。成功后进入现有 shortcut、B 样条及实际控制插值复检，抓放直线阶段仍采用 IK 与约束 B 样条。

A* 的最短路径性质仅针对定义的离散图及其边代价；网格可能遗漏连续空间中的窄通道，不能称为连续七维空间的最优路径。当前不采用启发式权重或目标直连替代整段搜索。网格邻接、步长与目标连接半径会影响结果，应随对比配置固定。

## 启动

```bash
cd ~/panda_control_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
./start.sh planner_algorithm:=astar planning_timeout:=30.0
```

另一个 source 后的终端：

```bash
ros2 service call /pick_and_place std_srvs/srv/Trigger '{}'
ros2 topic echo /planner_status --qos-durability transient_local
```

只运行 A* 三种子的物理采集：

```bash
ALGORITHMS=astar RUNS=3 TIMEOUT=30.0 ./tools/compare_gazebo.sh
```

五算法统一采集：

```bash
ALGORITHMS='rrt rrt_connect rrt_star astar prm_astar' RUNS=3 TIMEOUT=30.0 ./tools/compare_gazebo.sh
```

默认对比工具已包含五算法，实际记录见[五算法统一报告](five_planner_comparison.md)。已通过编译与离线调用，并纳入五算法物理任务记录。本次三次物理任务均在初始接近搜索超时，不能据此判断连续空间不可达。A* 为确定性图搜索，同一静态请求重复种子不产生不同随机搜索；Gazebo 中实际请求状态仍可能变化。
