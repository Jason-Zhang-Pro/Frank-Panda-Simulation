# 物理夹取与 RRT 搬运放置

本文包含先前简单场景实验记录；当前复杂场景的完整归位尚未验证通过，详见 [验证记录](validation.md)。文中的 `<workspace>` 表示本仓库克隆目录。

先关闭旧仿真，重新启动以加载可运动夹爪、动态方块和新的青色放置台：

```bash
cd <workspace>
./start.sh
```

启动完成后，新终端执行：

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 service call /pick_and_place std_srvs/srv/Trigger '{}'
```

`/grasp_target` 是同一完整流程的别名。不要连续调用；服务 success 仅表示接受任务，最终结果看：

```bash
ros2 topic echo /planner_status
```

流程：张开 → 移至方块上方 → 竖直下降 → 闭合 → 确认双指接触 → 实际抬起约 0.10 m → RRT-Connect 携物搬运 → 竖直下降 → 松开 → 检查方块落在放置台 → 退回 → RRT 避障归位。

新增放置台中心 x=0.32、y=-0.55 m，台面 z=0.32 m；方块放下后的中心约为 (0.32,-0.55,0.35)。这些坐标来自 worlds/world.urdf 的 place_pedestal。台面在 Gazebo 中呈青色。

搬运与归位默认使用双树 RRT-Connect，也可通过 planner_algorithm 参数切回单树 RRT。不添加人为的中间退让点。接近、抬升、放下是完成物理夹取必需的动作阶段；短距离竖直动作使用连续逆解，并逐点检查机械臂和物体碰撞。整条轨迹保留关节速度、加速度限制。

RRT 默认随机采样，每次结果可能不同。默认规划超时 15 s，最多 12000 次扩展。基础 RRT 在这个场景可能找不到搬运路径；此时保持夹持并报告 pick_place_failed，不强行运动。若失败时仍在携物状态，重新启动仿真恢复初始状态后再测试。可用 `ros2 param set /panda_planner rrt_seed 3` 选择可重复的随机种子；负数表示不固定种子。

携物时碰撞检测包含方块的实际尺寸和夹持相对位姿，不只检查机械臂。只对有意接触的两个手指与方块，以及抬起/放下时指定的支撑台面允许接触；其他碰撞继续拒绝。采用离散检测，当前主要用于静态障碍仿真；不是连续碰撞证明或动态障碍避障。

红色方块不再 static，质量 0.05 kg，依靠真实接触与摩擦夹持，不焊接、不瞬移。通过 Gazebo 的动态位姿与接触传感器验证：双指触碰、实际抬起、最终位置。失败时不报告搬运成功。

手动夹爪命令（空闲且未携物时）：

```bash
ros2 service call /open_gripper std_srvs/srv/Trigger '{}'
ros2 service call /close_gripper std_srvs/srv/Trigger '{}'
```

控制节点仍只有一个，同时发布手臂和夹爪两路力矩。新增 panda_gripper_controller 在启动时一同激活。原先调好的 KD 保留；肩关节 2 的软件力矩限幅从 40 调到 70 N·m，以满足竖直夹取姿态的重力负载，仍低于 URDF 的 87 N·m。手指单指力矩接口实际单位为线力 N，限幅 5 N，参考开口速度限制为单指 0.02 m/s。

模型改动在 franka_description/robots/common/franka_hand_actuated.xacro 和 panda_sim.urdf.xacro；场景、夹取流程、规划控制代码均仍在 panda_planning 包中。

测试范围：已验证可运动夹爪、真实双指接触与抬升、物体碰撞检测及支撑接触规则。直接 RRT 搬运存在超时，不保证每次整套动作都完成。没有为提高成功率加入人为绕行点。


## 完成后自动归位

物体放稳、夹爪张开并退出放置区域后，自动以当前选定规划器搜索回初始关节姿态的路径，再按原五次插值轨迹执行。不是直接把关节位置跳回去。

默认 home_joint_positions（rad）：[0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398]。可通过 panda_planner 的同名参数修改，但仍会检查限位与碰撞。

返回期间显示 returning_home，等控制节点确认到达且关节误差达标，才发布 pick_place_succeeded。归位失败单独显示 pick_place_home_failed，表示物体已放置，但机械臂没有成功归位；保持当前控制目标，夹爪仍张开。


## 切换规划器

当前默认 planner_algorithm=rrt_connect；接近、搬运和归位的关节路径搜索均使用它。竖直抬升/下降仍使用连续逆解和碰撞复检。轨迹插值、力矩控制与碰撞裕量保持原样。

在任务开始前切换（不要在正在执行时切换）：

```bash
ros2 param set /panda_planner planner_algorithm rrt_connect
# 对比原单树 RRT：
ros2 param set /panda_planner planner_algorithm rrt
```

RRT-Connect 分别区分 timeout 和 iteration_limit 失败原因。仍不保证每次都找到路径，也不是最短路径算法。


先前简单场景的 RRT-Connect 验证记录：固定随机种子 3 的隔离 Gazebo 测试已完成真实夹取、抬升、直接携物搬运、放置松开、退回及 RRT-Connect 归位，最终最大关节误差约 0.0005 rad。没有添加人为绕行点；单次通过不代表每个随机种子都成功。


## Shortcut 与夹取力度

默认开启 shortcut：RRT/RRT-Connect 找到路径后，尝试直接连接非相邻的路径点，连接通过原碰撞检测才删除中间点。起终点不变，不添加人为绕行点。携物状态同样检查方块；生成轨迹后再次复检。

只对搜索生成的接近、搬运和归位路径使用 shortcut。竖直抬升、下降和退出放置区域的连续逆解路径保持不变，避免破坏这些动作的方向要求。

日志显示 `shortcut: 原点数 → 新点数 个路径点，碰撞复检通过`。参数：shortcut_enabled=true、shortcut_attempts=150、shortcut_timeout=2.0 秒。关闭对比：

```bash
ros2 param set /panda_planner shortcut_enabled false
# 恢复：
ros2 param set /panda_planner shortcut_enabled true
```

夹爪闭合增益 GRIPPER_CLOSE_KP 从 100 提高到 200，仍保留模型单指 5 N 线力指令限幅；张开使用原增益 100。在方块接触位置单指约 0.03 m 时，原静态指令约 3 N，新指令达到 5 N 限幅。该值是控制命令，不等同于精确测量的接触力。开口速度及手臂控制参数保持不变。
