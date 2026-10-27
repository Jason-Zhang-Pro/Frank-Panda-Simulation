# Franka Panda：C++ 避障规划与物理抓放

ROS 2 Humble / Gazebo Fortress 下的七自由度机械臂项目。通过 **五算法路径搜索 → shortcut → B 样条与时间缩放 → PD＋重力补偿**，完成抓取、携物避障、放置和归位。算法、控制和启动节点使用 C++，场景与启动配置使用 URDF / SDF / XML。

我围绕 Panda 抓放任务，完成了规划库接入、碰撞场景构建、轨迹生成、力矩控制和任务状态机的集成，并通过仿真实验迭代运动平滑性与执行效率。RRT 系列算法调用 OMPL 实现，重点工作在于工程集成、约束处理与实验分析。

## 夹爪滑移修复

针对搬运时方块相对夹爪滑动和转动，我加入双指接触确认后的夹持力保持、夹口中心稳定，以及每侧四片的弹簧阻尼柔顺指垫，解决了夹爪搬运方块时的滑动问题。方块仍依靠物理接触搬运，没有焊接、固定约束或位姿瞬移。

通过接触后力保持、双指中心稳定与分块柔顺指垫，完成稳定夹持、抬升、避障搬运与释放。方块依靠真实物理接触保持，没有固定约束或位姿瞬移。柔顺指垫采用分块弹簧阻尼模型。

## Gazebo 五算法统一对照实验

在同一修正后的原场景，统一分块柔顺指垫、控制参数、shortcut与B样条，对五种算法各运行三个种子，共15次独立任务；单次全局搜索预算30秒。已提交的成功与失败全部保留。

| 算法 | 携物轨迹生成 | 完整任务成功 | 成功任务墙钟均值 / s |
|---|---:|---:|---:|
| RRT | 3/3 | 1/3 | 103.14 |
| RRT-Connect | 3/3 | 3/3 | 110.61 |
| RRT* | 1/3 | 0/3 | — |
| A* | 未进入 | 0/3 | — |
| PRM＋A* | 3/3 | 3/3 | 86.65 |

完整成功要求抓取、放置、松开及归位七段均到位。每算法仅三个样本，物理初始化与接触非确定性回放，结论限定于当前配置。PRM初始接近直接连接通过，不能据此宣称它在任意复杂场景都更优。建图、查询、轨迹执行和完整任务结果分开记录。

[独立实验报告与全部原始数据](docs/five_planner_comparison.md)。

## 技术栈与库调用

| 库／框架 | 项目中的具体用途 |
|---|---|
| **OMPL** | 调用 `RRT`、`RRTConnect`、`RRTstar`；构建七维关节状态空间、设置搜索预算与路径长度优化目标 |
| **MoveIt Core** | 使用 `RobotModel`、`RobotState`、`PlanningScene` 构建机器人和世界模型，计算 FK/Jacobian，管理携物附着和允许接触规则；不依赖 `move_group` 服务 |
| **FCL** | 通过 MoveIt 碰撞环境调用，检查机器人自碰撞、障碍碰撞和携带物体碰撞 |
| **Eigen** | 坐标变换、矩阵运算、数值 IK，以及实验方案中的阻尼雅可比伪逆求解 |

在这些库之上，我实现了抓放状态机、shortcut 路径简化、B 样条轨迹生成与时间缩放、控制插值复检、PD＋重力补偿，以及多算法对比工具。RRT系列调用OMPL，网格A*与惰性PRM＋A*由项目实现；碰撞判断通过MoveIt/FCL完成，避免混淆库能力与项目实现。

## 架构

```text
任务服务 /pick_and_place
        ↓
planner_node：抓放状态机、规划与实际到位确认
        ↓
PlanningEngine：RRT / RRT-Connect / RRT* / A* / PRM＋A*
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
./start.sh compliant_pads:=true
```

`start_cpp.sh` 保留为兼容入口。默认 RRT-Connect；切换算法：

```bash
./start.sh scene:=world planner_algorithm:=rrt compliant_pads:=true
./start.sh scene:=world planner_algorithm:=rrt_star compliant_pads:=true
./start.sh scene:=world planner_algorithm:=astar planning_timeout:=30.0 compliant_pads:=true
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
| `planner_algorithm` | `rrt_connect` | `rrt` / `rrt_connect` / `rrt_star` / `astar` / `prm_astar` |
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
    │   ├── prm_astar_planner.cpp  候选路线图复用、A* 与惰性碰撞检测
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

[算法对比与运行方法](docs/planner_comparison.md) · [工程架构与边界](docs/architecture.md) · [探索与优化记录](docs/experiments.md) · [后续计划](docs/roadmap.md) · [第三方声明](THIRD_PARTY_NOTICES.md)

碰撞复检是离散采样，不是连续安全证明；参考轨迹无碰撞不等于带跟踪误差的物理运动绝对无碰撞。当前用于静态仿真学习，不用于实体机械臂直接部署。

## 新增规划器与指垫模型

网格 `astar` 与惰性 `prm_astar` 已接入同一套碰撞检测、shortcut、B样条和抓放流程。PRM复用候选节点与连接，每次根据当前空手／携物状态重新验证候选路径。首次建图和查询耗时分开记录。

```bash
# 使用与统一实验相同的原复杂场景与柔顺指垫
./start.sh scene:=world planner_algorithm:=prm_astar \
  planning_timeout:=30.0 compliant_pads:=true
```

`scene:=tabletop` 可切换到独立桌面场景；本次五算法结果使用 `world`，不代表桌面场景的验证结果。`compliant_pads:=false` 可恢复刚性指垫；启动默认仍为false，复现实验需显式设为true。

[A*方法](docs/astar.md) · [PRM＋A*与场景](docs/prm_astar.md) · [夹爪控制](docs/gripper_control.md) · [柔顺指垫](docs/compliant_pads.md)
