# Franka Panda：C++ 避障规划与物理抓放

ROS 2 Humble / Gazebo Fortress 下的七自由度机械臂项目。通过 **OMPL 路径搜索 → shortcut → B 样条与时间缩放 → PD＋重力补偿**，完成抓取、携物避障、放置和归位。算法、控制和启动节点使用 C++，场景与启动配置使用 URDF / SDF / XML。

我围绕 Panda 抓放任务，完成了规划库接入、碰撞场景构建、轨迹生成、力矩控制和任务状态机的集成，并通过仿真实验迭代运动平滑性与执行效率。RRT 系列算法调用 OMPL 实现，重点工作在于工程集成、约束处理与实验分析。

## Gazebo 实测进展

对 RRT、RRT-Connect、RRT* 各运行三个种子，共 **9 次 Gazebo 完整任务尝试**。统一 30 秒搜索预算、场景、手臂 PD、shortcut、B 样条与 bounded 抓放时间。

| 算法 | 携物轨迹生成成功 | 完整任务成功 | 成功任务墙钟均值 / s |
|---|---:|---:|---:|
| RRT | 2/3 | 1/3 | 110.42 |
| RRT-Connect | 3/3 | 2/3 | 112.83 |
| RRT* | 0/3 | 0/3 | — |

失败全部保留，完整成功要求抓取、放置、松爪与归位全部完成。每种算法仅三个样本，结论限定于当前场景和配置。[实验方法、逐任务结果与原始数据](docs/planner_comparison.md)。

抓放时间通过实际 Hermite 插值的 Bernstein 包络复检，不提高速度/加速度限值。[时间分配实现](docs/cartesian_timing.md)。后续扩大样本、定位失败并探索 A*。

## 技术栈与库调用

| 库／框架 | 项目中的具体用途 |
|---|---|
| **OMPL** | 调用 `RRT`、`RRTConnect`、`RRTstar`；构建七维关节状态空间、设置搜索预算与路径长度优化目标 |
| **MoveIt Core** | 使用 `RobotModel`、`RobotState`、`PlanningScene` 构建机器人和世界模型，计算 FK/Jacobian，管理携物附着和允许接触规则；不依赖 `move_group` 服务 |
| **FCL** | 通过 MoveIt 碰撞环境调用，检查机器人自碰撞、障碍碰撞和携带物体碰撞 |
| **Eigen** | 坐标变换、矩阵运算、数值 IK，以及实验方案中的阻尼雅可比伪逆求解 |

在这些库之上，我实现了抓放状态机、shortcut 路径简化、B 样条轨迹生成与时间缩放、控制插值复检、PD＋重力补偿，以及多算法对比工具。搜索算法直接使用 OMPL；碰撞判断通过 MoveIt/FCL 完成，避免混淆库能力与项目实现。

## 架构

```text
任务服务 /pick_and_place
        ↓
planner_node：抓放状态机、规划与实际到位确认
        ↓
PlanningEngine：OMPL RRT / RRT-Connect / RRT*
        ↓                     ↔ MoveIt RobotModel / RobotState / PlanningScene / FCL
shortcut：无碰撞远端直连
        ↓
B 样条 → 时间缩放 → 实际控制插值碰撞及限速复检
        ↓ /planned_trajectory
control_node：五次 Hermite 插值 → 关节 PD＋重力补偿、夹爪力控制
        ↓ /panda_effort_controller/commands
ros2_control → Gazebo 物理仿真 → /joint_states
```

B 样条负责圆滑几何路径，时间缩放负责速度与加速度，控制器的五次 Hermite 插值负责按位置/速度/加速度边界条件执行。二者不是互相替代的算法。

## 构建与启动

需要 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress，以及与当前环境匹配的 `gz_ros2_control` 插件。模型当前沿用 `/opt/ros/humble/lib/libgz_ros2_control-system.so`，换发行版时需调整。依赖详情见 [架构](docs/architecture.md)。

```bash
source /opt/ros/humble/setup.bash
cd ~/panda_control_ws
rosdep install --from-paths src --ignore-src -r -y
```

`panda_planning` 为资源型 ament_cmake 包。首次构建直接执行 `colcon build`；已有构建产物时，可先停止旧实例并清理生成目录，避免残留配置：

```bash
# 仅在工作区根目录执行；删除的是可重新生成的构建产物。
rm -rf build install log
colcon build --base-paths src --symlink-install \
  --cmake-args -Dompl_DIR=/opt/ros/humble/share/ompl/cmake
source install/setup.bash
./start.sh
```

`start_cpp.sh` 保留为兼容入口。默认 RRT-Connect；切换算法：

```bash
./start.sh planner_algorithm:=rrt
./start.sh planner_algorithm:=rrt_star
./start.sh planner_algorithm:=astar planning_timeout:=30.0
# 后台或启动完成后保持暂停：
./start.sh gui:=false auto_play:=false
```

启动时 Gazebo 保持暂停，C++ `startup_node` 等待三个控制器配置完成，通过小步仿真激活并确认保持力矩后再播放。控制、规划、启动协调或 Gazebo 服务退出会触发 launch 关闭。启动脚本用锁避免重复实例；不要同时从其他终端绕过入口启动第二套仿真。

## 执行与查看状态

另一个已 source 工作区的终端：

```bash
ros2 service call /pick_and_place std_srvs/srv/Trigger '{}'
ros2 topic echo /planner_status --qos-durability transient_local
```

服务接受不代表任务完成。`planning` / `planned` / `executing` / `succeeded` 是阶段状态，`pick_place_succeeded` 才表示抓取、放置、松开和归位全部完成。超时、碰撞或接触失败有错误状态；携物失败保持夹爪指令，不强行继续。

独立关节目标接口保留 `/ompl_plan`、`/ompl_execute`、`/ompl_move`；使用 `goal_joint_positions` 参数指定七个关节目标。抓放或携物状态禁止独立运动。`/open_gripper`、`/close_gripper` 仅在空闲、非携物状态可用。

日志 `cartesian_bspline` 表示抓放平滑成功；`stop_fallback` 表示约束或碰撞复检失败，已回退分段停车轨迹。规划结果通过 `/ompl_planned_trajectory` 发布，便于外部可视化。机器人、障碍、方块和放置台在同一 Gazebo 场景中可见。

## 常用参数

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `planner_algorithm` | `rrt_connect` | `rrt` / `rrt_connect` / `rrt_star` / `astar` |
| `planning_timeout` | 15 s | 搜索预算；不含 IK 与后处理全部时间 |
| `shortcut_enabled` | true | 碰撞检测通过才直连远端路点 |
| `cartesian_timing` | `bounded` | 抓放时间优化；`conservative` 恢复旧时间分配 |
| `trajectory_smoothing` | `bspline` | `stop` 可恢复原分段停车 |
| `velocity_scale` | 0.3 | URDF 关节速度上限比例 |
| `acceleration_limit` | 0.9 rad/s² | 参考加速度上限 |
| `grasp_wait` | 1.4 s | 最短闭合等待，仍需实际接触 |
| `release_wait` | 0.5 s | 最短释放等待，仍需实际张开 |

规划参数通过 `ros2 param set /panda_ompl_planner ...` 修改，下次规划读取；等待参数下次任务开始读取。PD 和夹爪参数在控制实现及 `config/controller.yaml` 中。控制器只允许一份力矩发布者；目前是反馈驱动、最多约 100 Hz，未启用实验速度滤波或参考缓冲。

## 目录

```text
src/
├── franka_description/             Panda 模型、网格、ros2_control 配置与上游许可证
├── panda_planning/                资源包：launch XML、world URDF/SDF
└── panda_ompl/
    ├── src/
    │   ├── planner_node.cpp       ROS 接口、任务状态机、规划调度
    │   ├── planning_engine.cpp    OMPL 选择、FK/IK、FCL、shortcut、旧时间分配
    │   ├── astar_planner.cpp      七维关节网格 A* 搜索
    │   ├── bspline_trajectory.cpp B 样条、时间缩放、实际插值复检
    │   ├── control_node.cpp       关节 PD、重力补偿、夹爪与反馈超时
    │   ├── startup_node.cpp       暂停单步激活与启动协调
    │   ├── benchmark_node.cpp     离线单查询、CSV 指标
    │   ├── gazebo_trial_node.cpp  完整物理抓放采集与任务结果记录
    │   └── cartesian_servo.cpp    未启用的雅可比抓放实验
    ├── include/panda_ompl/        规划及控制模型接口
    └── config/                   控制增益、对比起终点
 tools/compare_gazebo.sh           主对比：独立 Gazebo 世界的完整任务试验
 tools/compare_planners.sh         补充：离线单查询与后处理消融
 docs/                            架构、比较方法、实验记录、后续计划
```

[算法对比与运行方法](docs/planner_comparison.md) · [工程架构与边界](docs/architecture.md) · [探索与优化记录](docs/experiments.md) · [A* 后续计划](docs/roadmap.md) · [第三方声明](THIRD_PARTY_NOTICES.md)

碰撞复检是离散采样，不是连续安全证明；参考轨迹无碰撞不等于带跟踪误差的物理运动绝对无碰撞。当前用于静态仿真学习，不用于实体机械臂直接部署。

## A* 算法组

新增 `astar`，与 OMPL 三种采样算法使用相同 MoveIt/FCL、shortcut、B 样条和抓放状态机。A* 为项目实现的七维关节网格搜索，详见 [A* 方法与运行](docs/astar.md)。现有九次 Gazebo 实验仅包含三种 RRT 系列算法，A* 尚未采集完整任务数据。
