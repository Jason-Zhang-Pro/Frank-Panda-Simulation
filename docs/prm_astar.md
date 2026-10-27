# 静态桌面场景与惰性 PRM＋A*

## 场景和启动

保留原 world 场景。新增工作桌、桌腿、挡板、立柱、上方横梁、抓取台与放置台。URDF 规划几何和 SDF 仿真几何由同一组定义生成；目标方块沿用现有质量、摩擦和接触传感器。

```bash
cd ~/panda_control_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select panda_ompl panda_planning \
  --symlink-install --cmake-args -Dompl_DIR=/opt/ros/humble/share/ompl/cmake
source install/setup.bash
./start.sh planner_algorithm:=prm_astar planning_timeout:=30.0
```

start.sh 对 prm_astar 默认选择 tabletop。可显式覆盖场景，使用同一场景对比 RRT-Connect：

```bash
./start.sh scene:=tabletop planner_algorithm:=rrt_connect planning_timeout:=30.0
```

提供三个固定放置台位置配置，每次启动选择一个：

| scene | 放置方块目标中心 / m |
|---|---|
| tabletop | (0.32, -0.55, 0.35) |
| tabletop_center | (0.58, -0.35, 0.35) |
| tabletop_outer | (0.62, -0.55, 0.35) |

```bash
./start.sh planner_algorithm:=prm_astar scene:=tabletop_center planning_timeout:=30.0
```

每个场景具有一个活动放置台；切换位置需要重启场景，不是在运行中移动障碍。机械臂仍固定于世界原点，工作桌位于其前方。

另一个 source 后的终端执行：

```bash
ros2 service call /pick_and_place std_srvs/srv/Trigger '{}'
ros2 topic echo /planner_status --qos-durability transient_local
```

## 规划实现

prm_astar_planner.cpp 实现七维关节空间的惰性概率路线图与 A*。首次全局请求建立 1800 个候选姿态，固定采样种子 42：一半关节限位内均匀采样，一半围绕标准 home 姿态的截断正态采样。每个点连接半径 2.5 rad 内最近的 20 个点，取无向边并集。起点和精确终点在每次查询中临时接入，必要时使用相同半径的直连边。

候选图不预先声明无碰撞。A* 用七维欧氏边长和启发函数搜索候选图，再沿候选路径以不大于 0.02 rad 的间隔调用 MoveIt/FCL。发现碰撞则在本次查询中禁用该边并重新搜索，直到路径全部通过或预算耗尽。

复用的是采样点与候选连接，碰撞有效性只在单次查询内缓存。空手、携物、手指位置、允许接触及方块位姿变化后均重新检查，不能复用空手时的有效结论。路线图属于 PlanningEngine 实例，重启节点或切换世界会重建；暂不保存到磁盘。首次建图包含在搜索预算内，失败建图不会提交半成品。有限路线图无路径不表示连续空间不可达。

抓取下降、抬升、放置下降和撤离仍使用现有 IK 直线与约束 B 样条；全局接近、携物搬运和归位使用 PRM＋A*，并接相同 shortcut、B 样条、实际插值复检及 PD＋重力补偿。

## 时间记录和比较

日志分别输出 roadmap 的 build、query、reused、nodes、edges。第一次 build 包含采样及候选邻接构建；query 包含临时接入、A* 与惰性碰撞验证。复用查询 build 为 0。总 search 包含首次建图及查询，不含后续 shortcut 和轨迹生成。离线 benchmark CSV 同样输出这些字段。

同一进程的后续全局规划复用图，所以一次抓放内部的多个查询也可展示复用。重新启动的独立试验会重建图，不能把冷启动结果称为热查询数据。PRM 固定采样种子不随任务 seed 改变；物理状态变化仍可能改变搜索结果。

```bash
SCENE=tabletop ALGORITHMS='rrt_connect prm_astar' RUNS=3 TIMEOUT=30.0 \
  ./tools/compare_gazebo.sh
```

对比必须显式选同一个 SCENE。图构建、路径查询、轨迹执行和完整抓放成功分开统计，不将首次建图成本隐藏。独立桌面场景尚未运行统一验证；原复杂场景的实际结果见[五算法报告](five_planner_comparison.md)。

## 夹爪力参数

当前闭合增益为 400 N/m，闭合时每指力上限为 10 N，模型两指 effort 限值同步为 10 N；张开仍限幅 5 N。配置项为 controller.yaml 的 gripper_close_force_limit，启动时读取。历史实验快照保持原始参数，不用新限值改写历史记录。调整后需重启 Gazebo 重新加载模型，本次统一试验采用柔顺指垫与中心稳定控制，结果见五算法报告。
