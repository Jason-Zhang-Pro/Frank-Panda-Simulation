# 工程架构与库调用

| 依赖 | 代码中的用途 |
|---|---|
| ROS 2 `rclcpp`、标准消息和服务 | 独立规划/控制/启动节点、状态话题、Trigger 任务服务 |
| OMPL | 7D 状态空间、RRT / RRT-Connect / RRT*、路径长度目标、搜索预算与种子 |
| MoveIt Core | RobotModel / RobotState、FK、Jacobian、PlanningScene、附着物体与碰撞矩阵 |
| FCL（由 MoveIt 碰撞环境调用） | 自碰撞、静态世界、夹爪和携物检测；独立于物理仿真计算规划碰撞 |
| URDF / SRDFdom | 机器人解析、关节限位、七轴规划组与语义模型 |
| Eigen | 坐标变换、IK 数值求解、实验阻尼雅可比伪逆 |
| ros2_control / effort_controllers | 接收七轴与双指力矩命令，JointStateBroadcaster 发布反馈 |
| Gazebo Fortress / ros_gz | 世界物理、接触与动态方块位姿、时钟及世界控制桥接 |

没有启动 MoveIt move_group；使用 MoveIt 的 C++ 核心库构建模型和碰撞场景。重力补偿由 URDF 质量/质心及各关节世界轴矩计算，包含活动夹爪；未加入携带方块质量、惯性或科里奥利补偿。参考轨迹通过五次 Hermite 插值执行，positions-only 旧接口仍可接受。

## 碰撞与抓放

`world.urdf` 是算法碰撞环境，`world.sdf` 是同一任务的物理场景；更改障碍时必须同步维护。包含相邻/固定刚性部件豁免、基座安装接触、8 mm 环境间隙、携带方块附着几何及阶段支撑面规则。附着仅发生于规划模型，不会把 Gazebo 方块焊接或瞬移到夹爪。任务要求实际双指接触和物体抬升/放置证据。

全局路径经 farthest-visible shortcut；B 样条圆滑后使用全段五次时间律减速起停，发出完整位置/速度/加速度。复检使用与控制端一致的 Hermite 多项式，Bernstein 包络检查位置、速度、加速度界限，再离散检查碰撞。抓放先以不超过 3 mm 空间间隔求局部 IK，并检查平滑后 TCP 距目标线段不超过 2 mm、姿态误差不超过 0.025 rad；这是采样与容差约束，不是精确直线证明。

平滑失败逐步细分控制点；仍失败回退原分段停车轨迹。不能保证所有阶段都连续通过或每次都用 B 样条。任务阶段、夹爪开合、接触确认仍有必要停顿。

## 节点与启动

- `planner_node`：后台搜索、抓放状态机、实际到位与错误状态，保留 `/ompl_*` 接口。五种全局规划算法可通过 `planner_algorithm` 切换。
- `control_node`：约 100 Hz 仿真反馈驱动，保持起始/结束位置；轨迹拒绝、反馈超时、时间回退有处理。唯一七轴力矩发布者。
- `startup_node`：通过桥接 ControlWorld 服务暂停单步，等待 inactive 配置，激活控制器并确认初始姿态与保持力矩；失败暂停世界并退出，launch 随之关闭。成功后保留节点作为启动协调进程。
- `gazebo_trial_node`：在隔离世界提交一次完整抓放，采集实际状态、物体位姿、任务终止与累计路径指标。
- `benchmark_node`：离线读取 URDF/world，输出单次查询记录；不会发布机器人命令。

项目节点通过 C++ 库接口协作，场景和启动配置分别采用 URDF/SDF 与 XML。

## 五算法与柔顺指垫

RRT、RRT-Connect、RRT*使用OMPL；astar_planner.cpp实现七维网格A*；prm_astar_planner.cpp实现惰性候选路线图与A*，图在同一PlanningEngine内复用，但碰撞有效性只在单次查询中缓存。固定图种子42，不与OMPL任务种子混淆。

compliant_pads=true时每侧四片被动弹簧阻尼指垫在Gazebo中压缩，规划使用未压缩的完整刚性包络。物理接触确认后启用目标夹持力与双指中心控制，仍无物体固定约束。七轴PD和关节阻尼不因该指垫改动。详见[指垫](compliant_pads.md)与[夹爪控制](gripper_control.md)。

## 当前验证与边界

本次完成同一场景、控制与后处理下15次Gazebo任务。RRT为1/3完整成功，RRT-Connect为3/3，RRT*为0/3，网格A*为0/3，PRM＋A*为3/3。失败均为搜索未找到精确路径或超时；另有任务前启动故障单独保存。详见[独立实验](five_planner_comparison.md)。

离散碰撞检测、跟踪误差、接触摩擦和采样频率仍会影响实际表现。未测量连续滑移量、jerk或抗扭极限，无实际接触力闭环、完整软体仿真、硬实时保证、动态障碍重规划或实体机器人验证。柔顺参数未以实物标定，少量成功不能等同于普遍可靠性证明。
