# RRT、RRT-Connect 与 RRT* 的统一 Gazebo 对照实验

本实验对 RRT、RRT-Connect、RRT* 各运行种子 1/2/3，共九次完整任务尝试，比较路径搜索、轨迹生成和实际抓放归位结果。夹爪闭合增益统一为 **400 N/m**，每指输出上限为 **5 N**。

## 统一条件

每次全局搜索预算 30 秒，整套任务上限 240 秒。统一场景、机械臂 PD、shortcut、B 样条、bounded 抓放时间、速度比例 0.3 和参考加速度上限 0.9 rad/s²。各次重建隔离的无 GUI Gazebo 世界，按种子交错运行算法，不改动原有仿真实例。完整任务要求真实抓取、放置、松爪和归位七段全部到位。全部有效失败保留，不以成功重跑替换。

物理接触、初始反馈和调度并非确定性回放，相同种子不保证相同状态。每种算法仅三个样本，结论限定于当前场景与配置；没有测量物体晃动幅值或接触力。

## 实验结果

| 算法 | 携物轨迹生成成功 | 完整任务成功 | 成功任务墙钟均值 / s |
|---|---:|---:|---:|
| RRT | 2/3 | 1/3 | 110.42 |
| RRT-Connect | 3/3 | 2/3 | 112.83 |
| RRT* | 0/3 | 0/3 | — |

成功任务耗时只对成功样本求均值，不能用失败前缀耗时进行效率排名。

## 结果分析

RRT 的携物轨迹生成成功 2/3，完整任务成功 1/3。种子 1 完成全部流程；种子 2 完成抓放，但归位搜索未找到精确解；种子 3 在携物搜索中未找到精确解。

RRT-Connect 的三次携物轨迹均通过复检，两次完成抓放归位。种子 3 在松爪后检查到方块位置不正确，末次记录坐标约为 (0.3530, -0.5557, 0.3499) m。放置判据要求方块与目标的位置距离不超过 0.025 m。仅凭终止记录不能区分滑动、接触或释放过程中的具体成因。

RRT* 三次均在携物搜索中未找到精确解。当前场景下，RRT-Connect 的搜索与任务完成表现较好；这不代表其在其他场景中具有相同优势。后续将扩大样本，并增加接触力、物体相对夹爪位姿与释放过程的记录。

## 同类初始接近段

均值基于每算法三个初始接近请求，长度为七维关节空间累计欧氏长度。RRT* solve 包含持续优化，首解由回调记录，无回调时用返回时间近似。

| 算法 | solve / s | 首解 / s | 原始长度 / rad | shortcut 后 / rad |
|---|---:|---:|---:|---:|
| RRT | 0.3447 | 0.3447 | 5.2032 | 2.2212 |
| RRT-Connect | 0.0044 | 0.0047 | 2.3497 | 2.2211 |
| RRT* | 30.0323 | 1.1683 | 5.2032 | 2.2221 |

共同 shortcut 会显著影响最终长度，不能将最终长度全部归因于搜索算法。未做平滑消融、jerk 或振荡 RMS 测量。

## 逐任务结果

墙钟从提交任务至终止，不包含冷启动。失败段数与时间仅覆盖任务前缀。

| 算法 | 种子 | 完整成功 | 墙钟 / s | 生成 / 到位段数 | 终止状态 |
|---|---:|---|---:|---:|---|
| RRT | 1 | 是 | 110.42 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT-Connect | 1 | 是 | 114.79 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT* | 1 | 否 | 85.93 | 3 / 3 | pick_place_failed: failed: rrt_star 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT | 2 | 否 | 101.82 | 6 / 6 | pick_place_home_failed: failed: rrt 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT-Connect | 2 | 是 | 110.87 | 7 / 7 | pick_place_succeeded: 真实抓取、放置、松开和避障归位全部完成 |
| RRT* | 2 | 否 | 85.95 | 3 / 3 | pick_place_failed: failed: rrt_star 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT | 3 | 否 | 55.95 | 3 / 3 | pick_place_failed: failed: rrt 未找到精确路径：Approximate solution；保持夹爪指令 |
| RRT-Connect | 3 | 否 | 49.00 | 5 / 5 | pick_place_failed: 方块实际放置位置不正确；保持夹爪指令 |
| RRT* | 3 | 否 | 85.86 | 3 / 3 | pick_place_failed: failed: rrt_star 未找到精确路径：Approximate solution；保持夹爪指令 |

## 携物搜索与最终轨迹复检

| 算法 | 种子 | 轨迹生成 | solve / s | 结果 |
|---|---:|---|---:|---|
| RRT | 1 | 通过 | 19.8960 | 最终轨迹复检通过 |
| RRT-Connect | 1 | 通过 | 0.0158 | 最终轨迹复检通过 |
| RRT* | 1 | 失败 | 30.0799 | failed: rrt_star 未找到精确路径：Approximate solution |
| RRT | 2 | 通过 | 7.8380 | 最终轨迹复检通过 |
| RRT-Connect | 2 | 通过 | 0.0528 | 最终轨迹复检通过 |
| RRT* | 2 | 失败 | 30.0142 | failed: rrt_star 未找到精确路径：Approximate solution |
| RRT | 3 | 失败 | 30.0881 | failed: rrt 未找到精确路径：Approximate solution |
| RRT-Connect | 3 | 通过 | 0.0562 | 最终轨迹复检通过 |
| RRT* | 3 | 失败 | 30.0042 | failed: rrt_star 未找到精确路径：Approximate solution |

## 数据与复现

[逐任务](results/planner_study_2026-10-25/trials.csv) · [逐阶段](results/planner_study_2026-10-25/phases.csv) · [汇总](results/planner_study_2026-10-25/summary.csv) · [原始日志与输入快照](results/planner_study_2026-10-25/README.md)

```bash
source /opt/ros/humble/setup.bash
cd ~/panda_control_ws
source install/setup.bash
CARTESIAN_TIMING=bounded RUNS=3 TIMEOUT=30.0 \
  ALGORITHMS='rrt rrt_connect rrt_star' \
  BENCHMARK_ROS_DOMAIN_ID=60 \
  BENCHMARK_IGN_PARTITION=panda_planner_study ./tools/compare_gazebo.sh
```

当前 controller.yaml 的 gripper_close_kp=400.0，每指力上限仍为 5 N。环境与库版本、源码和输入哈希随原始数据保存。碰撞复检为离散采样，参考轨迹通过不代表实际物理运动绝对无碰撞。下一步继续定位失败阶段、扩大样本，并探索 A*。
